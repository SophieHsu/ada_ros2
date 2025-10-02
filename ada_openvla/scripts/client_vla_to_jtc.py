#!/usr/bin/env python3
"""
Local ROS2 client for Option A (server-side inference):
- Grabs images from /camera/camera/color/image_raw
- Calls HTTP inference server (OpenVLA-OFT) for actions
- Converts actions (dx,dy,dz,dRx,dRy,dRz,grip) -> IK -> JointTrajectory (arm + hand)
- Processes each IK solution immediately (one position at a time)
- If inference/IK lags, arm holds last point (no jitter forward)

Requires:
  - Active FollowJointTrajectory action server for arm (JointTrajectoryController)
  - (Optional) Active FollowJointTrajectory for hand (gripper)

Run:
  ssh -N -L 8000:127.0.0.1:8000 user@GPU_SERVER
  ros2 run <your_pkg> vla_client_jtc.py --server_url http://127.0.0.1:8000/infer --instruction "pick up the standing coke can"
"""

import argparse, base64, io, math, threading, time, json
import numpy as np
import requests
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient

from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_multiply as qmul
# from cv_bridge import CvBridge  # Commented out due to NumPy 2.x compatibility issues


def imgmsg_to_cv2(msg, desired_encoding="rgb8"):
    """
    Manual implementation of cv_bridge's imgmsg_to_cv2 function
    to avoid NumPy 2.x compatibility issues with cv_bridge.
    """
    if msg.encoding == "rgb8" and desired_encoding == "rgb8":
        # Direct conversion for rgb8
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))
        return img
    elif msg.encoding == "bgr8" and desired_encoding == "rgb8":
        # Convert bgr8 to rgb8
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img
    elif msg.encoding == "bgra8" and desired_encoding == "rgb8":
        # Convert bgra8 to rgb8
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        return img
    elif msg.encoding == "rgba8" and desired_encoding == "rgb8":
        # Convert rgba8 to rgb8
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        return img
    else:
        # Fallback: try to decode with OpenCV
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, -1))
        if desired_encoding == "rgb8":
            if msg.encoding == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "bgra8":
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            elif msg.encoding == "rgba8":
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        return img


def axis_angle_to_quat(ax):
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], np.float64)
    axis = (ax / angle).astype(np.float64)
    s = math.sin(0.5 * angle)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(0.5 * angle)], np.float64)


class VLAClientJTC(Node):
    def __init__(self, args):
        super().__init__("vla_client_jtc")
        self.args = args

        # --- ROS I/O ---
        # self.bridge = CvBridge()  # Commented out due to NumPy 2.x compatibility issues
        self.latest_rgb = None
        self.create_subscription(Image, self.args.camera_topic, self._on_img, 10)
        self.joint_map = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)

        # TF + IK
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self, spin_thread=True)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

        # Action clients
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   f"/{self.args.arm_controller}/follow_joint_trajectory")
        self.hand_ac = ActionClient(self, FollowJointTrajectory,
                                    f"/{self.args.hand_controller}/follow_joint_trajectory") if self.args.hand_controller else None

        # State tracking
        self.last_sent_positions = None
        self.q_prev_list = None

        # Pose integrator (base frame)
        self.pos = None
        self.quat = None

        # Action intake queue (from server)
        self._pending_actions = []    # list[[7 floats], ...]
        self._pending_lock = threading.Lock()

        # Start background puller thread
        self._stop = False
        self._pull_thread = threading.Thread(target=self._pull_loop, daemon=True)
        self._pull_thread.start()

        # Control timer to consume actions → IK → send JTC chunks
        self.create_timer(self.args.control_tick_s, self._control_tick)

    # ---------------- subscriptions ----------------
    def _on_img(self, msg: Image):
        try:
            self.latest_rgb = imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception:
            return

    def _on_js(self, msg: JointState):
        self.joint_map = {n: p for n, p in zip(msg.name, msg.position)}

    # ---------------- server pull loop (background thread) ----------------
    def _pull_loop(self):
        hz = max(0.5, float(self.args.infer_hz))
        period = 1.0 / hz
        self.get_logger().info(f"Starting inference server pull loop at {hz} Hz")
        while not self._stop:
            t0 = time.time()
            try:
                if self.latest_rgb is not None:
                    self.get_logger().debug("Got camera image, sending to inference server...")
                    # JPEG encode
                    ok, enc = cv2.imencode(".jpg", self.latest_rgb[:, :, ::-1],
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    if ok:
                        b64 = base64.b64encode(enc.tobytes()).decode("utf-8")
                        payload = {
                            "image_b64": b64,
                            "instruction": self.args.instruction,
                            "state": self._build_proprio_8d_if_available(),  # optional
                            "return_chunk": self.args.return_chunk
                        }
                        r = requests.post(self.args.server_url, json=payload, timeout=2.5)
                        r.raise_for_status()
                        acts = r.json().get("actions", [])
                        if acts:
                            self.get_logger().info(f"Received {len(acts)} actions from inference server (chunk mode)")
                            with self._pending_lock:
                                self._pending_actions.extend(acts)
            except Exception as e:
                # Don't spam logs; print occasionally
                self.get_logger().warn(f"Inference server error: {e}")
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))

    def _build_proprio_8d_if_available(self):
        """Optional: build [x,y,z, r,p,y, 0.0, grip] if TF + gripper are available, else []"""
        try:
            if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                            timeout=Duration(seconds=0.1)):
                return []
            tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                             timeout=Duration(seconds=0.1))
            p = tf.transform.translation
            q = tf.transform.rotation
            # roll/pitch/yaw from quaternion
            import tf_transformations as tft
            rpy = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            # crude gripper openness if names known
            grip = 0.0
            if all(n in self.joint_map for n in self.args.finger_joints):
                fo = np.array(self.args.finger_open, dtype=np.float32)
                fc = np.array(self.args.finger_closed, dtype=np.float32)
                cur = np.array([self.joint_map[n] for n in self.args.finger_joints], np.float32)
                # normalize per joint and average
                g_i = np.clip((cur - fo) / np.maximum(1e-6, (fc - fo)), 0, 1)
                grip = float(np.mean(g_i))
            return [p.x, p.y, p.z, rpy[0], rpy[1], rpy[2], 0.0, grip]
        except Exception:
            return []

    # ---------------- IK & trajectory helpers ----------------
    def _ordered_current_joints(self):
        return [float(self.joint_map.get(n, 0.0)) for n in self.args.arm_joints]

    def _nearest_constraints(self, q_prev):
        cons = Constraints()
        for name, q in zip(self.args.arm_joints, q_prev):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(q)
            jc.tolerance_above = 0.25   # tune (rad)
            jc.tolerance_below = 0.25   # tune (rad)
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        return cons

    def _ordered_seed_from_map(self, seed_positions):
        return [float(seed_positions.get(n, self.joint_map.get(n, 0.0))) for n in self.args.arm_joints]

    def ik(self, pos, quat, seed_positions):
        self.get_logger().info(f"IK request: pos={pos}, quat={quat}")
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.args.move_group
        req.ik_request.ik_link_name = self.args.eef_frame
        req.ik_request.timeout = Duration(seconds=self.args.ik_timeout_s).to_msg()
        if self.q_prev_list is not None:
            req.ik_request.constraints = self._nearest_constraints(self.q_prev_list)
        else:
            print("No constraints applied (first IK call)")
            req.ik_request.constraints = Constraints()
        # Target pose
        ps = PoseStamped()
        ps.header.frame_id = self.args.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat.tolist()
        req.ik_request.pose_stamped = ps
        
        # Print the IK target position and orientation
        print(f"IK Target - Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        print(f"IK Target - Orientation (quat): [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")

        # Seed (ordered)
        seed_vec = self._ordered_seed_from_map(seed_positions)
        req.ik_request.robot_state.joint_state.name = list(self.args.arm_joints)
        req.ik_request.robot_state.joint_state.position = seed_vec
        
        # Print the seed joint positions
        print(f"IK Seed - Joint positions: {[f'{j:.4f}' for j in seed_vec]}")

        t0 = time.time()
        self.get_logger().info(f"Sending IK request to service...")
        fut = self.ik_cli.call_async(req)
        self.get_logger().info(f"Waiting for IK response with timeout {self.args.ik_timeout_s}s...")
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.args.ik_timeout_s)
        dt = time.time() - t0
        print("IK request sent: took ", dt, "seconds")
        if not fut.done():
            self.get_logger().error(f"IK service call timed out after {self.args.ik_timeout_s} seconds")
            return None
        res = fut.result()
        self.get_logger().info(f"IK response received: error_code={res.error_code.val}")
        if not res or res.error_code.val != res.error_code.SUCCESS:
            print(f"IK FAILED - Error code: {res.error_code.val if res else 'No response'}")
            self.get_logger().warn(f"IK failed with error code: {res.error_code.val}")
            return None
        names = res.solution.joint_state.name
        posns = res.solution.joint_state.position
        solution_dict = {n: p for n, p in zip(names, posns)}
        print(f"IK SUCCESS - Solution: {[f'{solution_dict[j]:.4f}' for j in self.args.arm_joints]}")
        self.get_logger().info(f"IK succeeded: {len(names)} joints")
        return solution_dict

    def grip_map(self, g):
        g = float(np.clip(g, 0.0, 1.0))
        fo = np.array(self.args.finger_open, dtype=np.float64)
        fc = np.array(self.args.finger_closed, dtype=np.float64)
        return (fo + g * (fc - fo)).tolist()

    def _cartesian_dt(self, dpos, drot):
        lin = float(np.linalg.norm(dpos)) / max(self.args.v_lin_max, 1e-9)
        ang = float(np.linalg.norm(drot)) / max(self.args.v_ang_max, 1e-9)
        return max(lin, ang, self.args.min_dt)

    def _joint_dt(self, q_prev, q_curr):
        if q_prev is None:
            return self.args.min_dt
        dq = np.abs(np.array(q_curr) - np.array(q_prev))
        dq_max = float(np.max(dq)) if dq.size > 0 else 0.0
        return max(dq_max / max(self.args.joint_vel_max, 1e-9), self.args.min_dt)

    def _send_single_point_and_wait(self, q_sol, g_val, dt_val):
        """
        Send a single joint position with timing.
        Adds a short continuity point at t = continuity_hold_s, then
        schedules the target point at t = continuity_hold_s + dt_val.
        """
        jt = JointTrajectory()
        jt.joint_names = self.args.arm_joints

        # continuity point
        if self.last_sent_positions is None:
            self.last_sent_positions = self._ordered_current_joints()

        tfs0 = self.args.continuity_hold_s * self.args.time_scale
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(self.last_sent_positions)
        pt0.time_from_start = Duration(seconds=tfs0).to_msg()
        jt.points.append(pt0)

        # target point
        t_target = tfs0 + dt_val * self.args.time_scale
        pt = JointTrajectoryPoint()
        pt.positions = [q_sol[n] for n in self.args.arm_joints]
        pt.time_from_start = Duration(seconds=t_target).to_msg()
        jt.points.append(pt)

        self.last_sent_positions = pt.positions

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        self.get_logger().info(f"[ARM] Single point: dur ~{dt_val:.2f}s (+{tfs0:.2f}s hold)")
        g1 = self.arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, g1)
        gh = g1.result()
        if not gh.accepted:
            self.get_logger().error("Arm trajectory goal was rejected")
            return
        g2 = gh.get_result_async()
        rclpy.spin_until_future_complete(self, g2)

        # Gripper (optional)
        if self.hand_ac:
            gtraj = JointTrajectory()
            gtraj.joint_names = self.args.finger_joints
            # continuity hold
            gp0 = JointTrajectoryPoint()
            gp0.positions = self.grip_map(g_val)
            gp0.time_from_start = Duration(seconds=tfs0).to_msg()
            gtraj.points.append(gp0)

            # target point
            gp = JointTrajectoryPoint()
            gp.positions = self.grip_map(g_val)
            gp.time_from_start = Duration(seconds=t_target).to_msg()
            gtraj.points.append(gp)

            self.get_logger().info(f"[GRIP] Single point")
            gsend = self.hand_ac.send_goal_async(FollowJointTrajectory.Goal(trajectory=gtraj))
            rclpy.spin_until_future_complete(self, gsend)
            ghandle = gsend.result()
            if ghandle.accepted:
                rclpy.spin_until_future_complete(self, ghandle.get_result_async())

    def _send_chunk_and_wait(self, q_list, g_list, dt_list):
        if len(q_list) == 0:
            return

        jt = JointTrajectory()
        jt.joint_names = self.args.arm_joints

        # continuity point
        if self.last_sent_positions is None:
            self.last_sent_positions = self._ordered_current_joints()

        tfs0 = self.args.continuity_hold_s * self.args.time_scale
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(self.last_sent_positions)
        pt0.time_from_start = Duration(seconds=tfs0).to_msg()
        jt.points.append(pt0)

        t_acc = tfs0
        for qsol, dtk in zip(q_list, dt_list):
            t_acc += dtk * self.args.time_scale
            p = JointTrajectoryPoint()
            p.positions = [qsol[n] for n in self.args.arm_joints]
            p.time_from_start = Duration(seconds=t_acc).to_msg()
            jt.points.append(p)

        self.last_sent_positions = jt.points[-1].positions

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        self.get_logger().info(f"[ARM] Chunk: {len(q_list)} pts, ~{t_acc - tfs0:.2f}s (+{tfs0:.2f}s hold)")
        g1 = self.arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, g1)
        gh = g1.result()
        if not gh.accepted:
            self.get_logger().error("Arm trajectory goal was rejected")
            return
        g2 = gh.get_result_async()
        rclpy.spin_until_future_complete(self, g2)

        # Gripper (optional)
        if self.hand_ac and len(g_list) > 0:
            gtraj = JointTrajectory()
            gtraj.joint_names = self.args.finger_joints
            # continuity hold
            gp0 = JointTrajectoryPoint()
            gp0.positions = self.grip_map(g_list[0])
            gp0.time_from_start = Duration(seconds=tfs0).to_msg()
            gtraj.points.append(gp0)

            t_acc = tfs0
            for g, dtk in zip(g_list, dt_list):
                t_acc += dtk * self.args.time_scale
                gp = JointTrajectoryPoint()
                gp.positions = self.grip_map(g)
                gp.time_from_start = Duration(seconds=t_acc).to_msg()
                gtraj.points.append(gp)

            gsend = self.hand_ac.send_goal_async(FollowJointTrajectory.Goal(trajectory=gtraj))
            rclpy.spin_until_future_complete(self, gsend)
            ghandle = gsend.result()
            if ghandle.accepted:
                rclpy.spin_until_future_complete(self, ghandle.get_result_async())
    
    def current_eef_pose(self):
        if not self.tfbuf.can_transform(
            self.args.base_frame, self.args.eef_frame, Time(), timeout=Duration(seconds=2.0)
        ):
            raise RuntimeError(f"TF {self.args.base_frame}->{self.args.eef_frame} not available yet.")
        tf = self.tfbuf.lookup_transform(
            self.args.base_frame, self.args.eef_frame, Time(), timeout=Duration(seconds=0.5)
        )
        p = tf.transform.translation; q = tf.transform.rotation
        pos = np.array([p.x, p.y, p.z], np.float64)
        quat = np.array([q.x, q.y, q.z, q.w], np.float64)
        return pos, quat

    # ---------------- main control tick ----------------
    def _control_tick(self):
        self.get_logger().info(f"Control tick: pending_actions={len(self._pending_actions)}")
        
        # Init pose once
        if self.pos is None or self.quat is None:
            self.get_logger().info("Initializing pose from TF...")
            # block briefly for TF
            if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                            timeout=Duration(seconds=0.5)):
                self.get_logger().warn(f"TF transform not available: {self.args.base_frame} -> {self.args.eef_frame}")
                return
            self.get_logger().info("TF transform available, getting pose...")
            tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                             timeout=Duration(seconds=0.2))
            t = tf.transform.translation
            q = tf.transform.rotation
            self.pos = np.array([t.x, t.y, t.z], np.float64)
            self.quat = np.array([q.x, q.y, q.z, q.w], np.float64)

            # also prime joint "previous"
            self.q_prev_list = self._ordered_current_joints()
            self.last_sent_positions = list(self.q_prev_list)
            self.get_logger().info(f"Pose initialized: pos={self.pos}, quat={self.quat}")

            # wait for servers
            if not self.ik_cli.service_is_ready():
                self.get_logger().info("Waiting for IK service …")
                self.ik_cli.wait_for_service(timeout_sec=10.0)
            self.get_logger().info("Waiting for arm JTC action …")
            if self.arm_ac.server_is_ready():
                self.get_logger().info("Arm JTC action server is already ready")
            else:
                if not self.arm_ac.wait_for_server(timeout_sec=10.0):
                    self.get_logger().error("Arm JTC action server not available after 10 seconds")
                    return
            if self.hand_ac:
                self.get_logger().info("Waiting for hand JTC action …")
                if self.hand_ac.server_is_ready():
                    self.get_logger().info("Hand JTC action server is already ready")
                else:
                    if not self.hand_ac.wait_for_server(timeout_sec=10.0):
                        self.get_logger().error("Hand JTC action server not available after 10 seconds")
                        return

        # drain ONE action from queue per control tick
        pulled = []
        with self._pending_lock:
            if self._pending_actions:
                # take only 1 action per tick to ensure proper sequencing
                pulled = [self._pending_actions[0]]
                del self._pending_actions[0]

        if not pulled:
            # Debug: show that we're waiting for actions
            if len(self._pending_actions) == 0:
                self.get_logger().info("Waiting for actions from inference server...")
            return
        
        self.get_logger().info(f"Processing 1 action from queue (remaining: {len(self._pending_actions)})")

        pos, quat = self.current_eef_pose()

        # Process only one action per control tick
        a = np.asarray(pulled[0], np.float64)
        dpos, drot, g = a[:3], a[3:6], float(a[6])

        # integrate desired pose
        target_pos = pos + dpos
        target_quat = np.array(qmul(quat, axis_angle_to_quat(drot)), np.float64)

        # IK
        seed = dict(self.joint_map)
        q_sol = self.ik(target_pos, target_quat, seed)
        if q_sol is None:
            self.get_logger().warn("IK failed for one step; skipping")
            return
        
        self.get_logger().debug(f"IK succeeded: {q_sol}")

        q_curr_list = [q_sol[n] for n in self.args.arm_joints]
        dt_cart = self._cartesian_dt(dpos, drot)
        dt_joint = self._joint_dt(self.q_prev_list, q_curr_list)
        dt_i = min(max(dt_cart, dt_joint), self.args.max_dt)

        # Update q_prev_list with the planned solution (where robot will be after this move)
        self.q_prev_list = q_curr_list

        # Send immediately - process one position at a time
        self._send_single_point_and_wait(q_sol, g, dt_i)

    # ---------------- teardown ----------------
    def destroy_node(self):
        self._stop = True
        try:
            if self._pull_thread.is_alive():
                self._pull_thread.join(timeout=0.2)
        except Exception:
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server_url", required=True, help="HTTP endpoint, e.g., http://127.0.0.1:8000/infer")
    ap.add_argument("--instruction", default="pick up the standing coke can")
    ap.add_argument("--return_chunk", type=bool, default=False, help="Ask server for a full chunk each call")
    ap.add_argument("--infer_hz", type=float, default=1.0, help="requests per second to server")
    ap.add_argument("--max_intake_per_tick", type=int, default=8, help="limit actions consumed per control tick")

    # Cameras / frames
    ap.add_argument("--camera_topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--base_frame",   default="j2n6s200_link_base")
    ap.add_argument("--eef_frame",    default="j2n6s200_end_effector")

    # Controllers & joints
    ap.add_argument("--move_group", default="jaco_arm")
    ap.add_argument("--arm_controller", default="jaco_arm_servo_controller")  # MUST be a JointTrajectoryController!
    ap.add_argument("--arm_joints", nargs="+", default=[
        "j2n6s200_joint_1","j2n6s200_joint_2","j2n6s200_joint_3",
        "j2n6s200_joint_4","j2n6s200_joint_5","j2n6s200_joint_6"
    ])
    ap.add_argument("--hand_controller", default="hand_controller")
    ap.add_argument("--finger_joints", nargs="+", default=["j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"])
    ap.add_argument("--finger_open",   nargs="+", type=float, default=[0.0, 0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0, 1.0])

    # Timing / speed
    ap.add_argument("--continuity_hold_s", type=float, default=0.05, help="Initial hold per point at last position to ensure continuity")
    ap.add_argument("--control_tick_s", type=float, default=0.05)  # 20 Hz control tick

    ap.add_argument("--v_lin_max", type=float, default=0.05)
    ap.add_argument("--v_ang_max", type=float, default=0.3)
    ap.add_argument("--joint_vel_max", type=float, default=0.5)
    ap.add_argument("--min_dt", type=float, default=0.08)
    ap.add_argument("--max_dt", type=float, default=0.6)
    ap.add_argument("--time_scale", type=float, default=1.0)

    ap.add_argument("--ik_timeout_s", type=float, default=1.5)
    ap.add_argument("--constraint_tol_rad", type=float, default=0.25, help="Joint constraint tolerance in radians (how close IK must stay to previous joint positions)")
    ap.add_argument("--disable_ik_constraints", action="store_true", help="Disable IK constraints to avoid joint limit issues")

    args = ap.parse_args()

    rclpy.init()
    node = None
    try:
        node = VLAClientJTC(args)
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
