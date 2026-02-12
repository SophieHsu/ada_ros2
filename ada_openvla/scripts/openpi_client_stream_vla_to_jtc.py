#!/usr/bin/env python3
"""
client_stream_vla_to_jtc.py  (aligned streaming, execution-fenced, fully async)

Key upgrades:
- IK calls are fully async (no blocking inside timer callbacks)
- Action sends (arm/gripper) are async; no spin_until_future_complete in timers
- Execution-fenced server queries (query_while_executing=False by default)
- Fresh-frame gating (require_fresh_image_after_exec=True)
- Proprio sent to server (optional) from live TF + gripper fraction
- Records last execution end stamp to fence queries
- Reseeds pose cursor right after each executed chunk
"""

import argparse, math, time, json, base64, io
from collections import deque
from typing import List, Optional

import numpy as np
import requests
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState, Image as ImageMsg
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_multiply as qmul, euler_from_quaternion
# from cv_bridge import CvBridge  # Commented out due to NumPy 2.x compatibility issues
from PIL import Image



# -------- cv_bridge replacement --------
def imgmsg_to_cv2(msg, desired_encoding="rgb8"):
    try:
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        channels_map = {"mono8": 1, "mono16": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "yuv422": 2, "yuv422_yuy2": 2}
        channels = channels_map.get(msg.encoding, 3)
        if "16" in msg.encoding:
            img_data = np.frombuffer(msg.data, dtype=np.uint16)
            img = img_data.reshape((msg.height, msg.width)) if channels == 1 else img_data.reshape((msg.height, msg.width, channels))
        else:
            img = img_data.reshape((msg.height, msg.width)) if channels == 1 else img_data.reshape((msg.height, msg.width, channels))
        if msg.encoding == desired_encoding:
            return img
        if desired_encoding == "rgb8":
            if msg.encoding == "bgr8":  return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            if msg.encoding == "bgra8": return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            if msg.encoding == "rgba8": return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            if msg.encoding == "mono8": return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            if msg.encoding == "mono16":
                img_8bit = (img / 256).astype(np.uint8)
                return cv2.cvtColor(img_8bit, cv2.COLOR_GRAY2RGB)
            return img
        if desired_encoding == "bgr8":
            if msg.encoding == "rgb8":  return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if msg.encoding == "rgba8": return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            if msg.encoding == "bgra8": return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            return img
        if desired_encoding == "mono8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if len(img.shape) == 3 else img
        return img
    except Exception as e:
        print(f"Error converting image from {msg.encoding} to {desired_encoding}: {e}")
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ["rgb8", "bgr8"]:
                img = img.reshape((msg.height, msg.width, 3))
            elif msg.encoding in ["rgba8", "bgra8"]:
                img = img.reshape((msg.height, msg.width, 4))
            else:
                img = img.reshape((msg.height, msg.width, -1))
            return img
        except Exception:
            return np.zeros((msg.height, msg.width, 3), dtype=np.uint8)

def cv2_to_imgmsg(cv_img, encoding="rgb8"):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.height = cv_img.shape[0]
    msg.width = cv_img.shape[1]
    msg.encoding = encoding
    msg.step = cv_img.shape[1] * (cv_img.shape[2] if len(cv_img.shape) == 3 else 1)
    if encoding == "bgr8" and len(cv_img.shape) == 3 and cv_img.shape[2] == 3:
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
    msg.data = cv_img.tobytes()
    return msg

from scipy.spatial.transform import Rotation as R

# -------- helpers --------
def axis_angle_to_quat(ax: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12:
        return np.array([0., 0., 0., 1.], dtype=np.float64)
    axis = (ax / angle).astype(np.float64)
    s = math.sin(0.5 * angle)
    return np.array([math.cos(0.5*angle), axis[0]*s, axis[1]*s, axis[2]*s], np.float64)

    # print("axis_angle: ", ax)

    # return R.from_rotvec(ax).as_quat(scalar_first=True) # w x y z


def quat_norm(q):
    q = np.asarray(q)
    n = float(np.linalg.norm(q))
    return q if n < 1e-12 else (q / n)

def ns_from_stamp(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

def quat_to_axis_angle(q_rel):
    q = q_rel / max(1e-12, np.linalg.norm(q_rel))
    w = np.clip(q[3], -1.0, 1.0)
    angle = 2.0*math.acos(w)
    s = math.sqrt(max(1e-12, 1.0 - w*w))
    if s < 1e-8:
        return np.array([1.,0.,0.], np.float64), 0.0
    return np.array([q[0]/s, q[1]/s, q[2]/s], np.float64), angle

# -------- main node --------
class StreamVLAtoJTC(Node):
    def __init__(self, args):
        super().__init__("vla_client_jtc_stream")
        self.args = args

        # self.bridge = CvBridge()  # NumPy 2.x compatibility issues
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self, spin_thread=True)

        # State
        self.joint_map = {}
        self.latest_rgb = None
        self.latest_rgb_stamp = None
        self._latest_rgb_ns = None  # int ns of latest_rgb_stamp
        self.last_fetch_ns = None   # int ns of the frame used in the last server query
        self.latest_wrist_rgb = None
        self.latest_wrist_rgb_stamp = None
        self._latest_wrist_rgb_ns = None  # int ns of latest_wrist_rgb_stamp

        # IK/JTC buffers
        self.ik_points: List[dict] = []
        self.grip_vals: List[float] = []
        self.dt_points: List[float] = []
        self.last_sent_positions: Optional[List[float]] = None
        self.q_prev_list: Optional[List[float]] = None
        self.pose_cursor_pos: Optional[np.ndarray] = None
        self.pose_cursor_quat: Optional[np.ndarray] = None
        self.actions_queue: deque = deque()
        self.chunks_seen = 0

        # Execution & async state
        self.executing = False
        self.last_exec_end_stamp = None  # builtin_interfaces/Time
        self.pending_ik_future = None
        self.pending_ik_meta = None
        self._arm_done = False
        self._gripper_done = False

        # Subs
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(ImageMsg, self.args.image_topic, self._on_image, 10)
        if self.args.wrist_image_topic:
            self.create_subscription(ImageMsg, self.args.wrist_image_topic, self._on_wrist_image, 10)

        # IK service + action clients
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   f"/{self.args.arm_controller}/follow_joint_trajectory")
        self.hand_ac = ActionClient(self, FollowJointTrajectory,
                                    f"/{self.args.hand_controller}/follow_joint_trajectory") if self.args.hand_controller else None

        # Optional enable topic
        self.pub_enable = None
        if self.args.topic_enable:
            self.pub_enable = self.create_publisher(Bool, self.args.topic_enable, 1)

        # Debug image publisher
        self.pub_sent_image = None
        if self.args.pub_debug_image:
            self.pub_sent_image = self.create_publisher(ImageMsg, "~/sent_image", 10)

        # Timers
        self.timer_fetch = self.create_timer(1.0 / self.args.vision_rate_hz, self._tick_fetch)
        self.timer_exec  = self.create_timer(0.01, self._tick_execute)  # 100 Hz

        self.get_logger().info("Streaming VLA→JTC client up. Waiting for servers…")
        self._wait_ready()

        if self.pub_enable:
            self.get_logger().info(f"Publishing enable on {self.args.topic_enable}")
            self.pub_enable.publish(Bool(data=True))

        self.prev_seed_positions = None

    # ----- callbacks -----
    def _on_js(self, msg: JointState):
        self.joint_map = {n: p for n, p in zip(msg.name, msg.position)}

    def _on_image(self, msg: ImageMsg):
        try:
            rgb = imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.latest_rgb = rgb.copy()
            self.latest_rgb_stamp = msg.header.stamp
            # cache ns and prune queue to same-frame actions
            try:
                self._latest_rgb_ns = ns_from_stamp(self.latest_rgb_stamp)
                if self.actions_queue:
                    from collections import deque
                    newq = deque()
                    for it in self.actions_queue:
                        if isinstance(it, tuple) and len(it) == 2 and it[1] == self._latest_rgb_ns:
                            newq.append(it)
                    self.actions_queue = newq
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def _on_wrist_image(self, msg: ImageMsg):
        try:
            rgb = imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.latest_wrist_rgb = rgb.copy()
            self.latest_wrist_rgb_stamp = msg.header.stamp
            try:
                self._latest_wrist_rgb_ns = ns_from_stamp(self.latest_wrist_rgb_stamp)
                if self.actions_queue:
                    newq = deque()
                    for it in self.actions_queue:
                        if isinstance(it, tuple) and len(it) == 2 and it[1] == self._latest_wrist_rgb_ns:
                            newq.append(it)
                    self.actions_queue = newq
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed (wrist): {e}")

    # ----- readiness -----
    def _wait_ready(self):
        self.get_logger().info("Waiting for IK service …")
        self.ik_cli.wait_for_service()
        self.get_logger().info("Waiting for arm JTC action …")
        self.arm_ac.wait_for_server()
        # if self.hand_ac:
        self.get_logger().info("Waiting for hand JTC action …")
        self.hand_ac.wait_for_server()

    # ----- pose & seeds -----
    def _ordered_current_joints(self) -> List[float]:
        return [float(self.joint_map.get(n, 0.0)) for n in self.args.arm_joints]

    def _nearest_constraints(self, q_prev: List[float]) -> Constraints:
        cons = Constraints()
        for name, q in zip(self.args.arm_joints, q_prev):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(q)
            jc.tolerance_above = self.args.constraint_tol_rad   
            jc.tolerance_below = self.args.constraint_tol_rad
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        return cons

    def _ordered_seed_from_map(self, seed_positions):
        if seed_positions is None:
            seed_positions = self.joint_map
        else:
            seed_positions = {n: float(seed_positions.get(n, self.joint_map.get(n, 0.0))) for n in self.args.arm_joints}
        
        if seed_positions == [0.0]*len(self.args.arm_joints) or seed_positions is None or sum(seed_positions.values()) <= 0.001:
            seed_positions = self.prev_seed_positions

        self.prev_seed_positions = seed_positions
        return [seed_positions[n] for n in self.args.arm_joints]

    def _reseed_pose_cursor(self):
        if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                        timeout=Duration(seconds=1.0)):
            raise RuntimeError(f"TF {self.args.base_frame}->{self.args.eef_frame} not available")
        tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                         timeout=Duration(seconds=0.5))
        p = tf.transform.translation; q = tf.transform.rotation
        self.pose_cursor_pos  = np.array([p.x, p.y, p.z], np.float64)
        self.pose_cursor_quat = np.array([q.x, q.y, q.z, q.w], np.float64)

    # ----- gripper fraction & proprio -----
    def _grip_fraction(self) -> float:
        vals = []
        for i, n in enumerate(self.args.finger_joints):
            if n in self.joint_map and i < len(self.args.finger_open) and i < len(self.args.finger_closed):
                num = self.joint_map[n] - self.args.finger_open[i]
                den = max(1e-6, self.args.finger_closed[i] - self.args.finger_open[i])
                vals.append(float(np.clip(num / den, 0.0, 1.0)))
        return float(np.mean(vals)) if vals else 0.0

    def _proprio_state(self) -> List[float]:
        """8D: [x,y,z,r,p,y, 0.0, grip] in base frame."""
        try:
            if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                            timeout=Duration(seconds=0.2)):
                return [0.0]*8
            tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                             timeout=Duration(seconds=0.2))
            t = tf.transform.translation; q = tf.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            g = self._grip_fraction()
            # return [t.x, t.y, t.z, roll, pitch, yaw, 0.0, g]
            # return [t.x, t.y, t.z, q.x, q.y, q.z, q.w, g]
            return [t.x, t.y, t.z, -q.x, -q.y, -q.z, -q.w, g]

        except Exception:
            return [0.0]*8

    # ----- IK (async) -----
    def _ik(self, pos, quat, seed_positions):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.args.move_group
        req.ik_request.ik_link_name = self.args.eef_frame
        req.ik_request.timeout = Duration(seconds=self.args.ik_timeout_s).to_msg()

        # Use nearest-joint constraints when allowed and we have a previous solution
        if (self.q_prev_list is not None) and (not self.args.disable_ik_constraints):
            req.ik_request.constraints = self._nearest_constraints(self.q_prev_list)
        else:
            req.ik_request.constraints = Constraints()
        ps = PoseStamped()
        ps.header.frame_id = self.args.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat.tolist()
        req.ik_request.pose_stamped = ps

        seed_vec = self._ordered_seed_from_map(seed_positions)

        # Clamp the seed vector to the joint limits
        LOW  = np.array([-3.14, 1.57, 0.33, -3.14, 0, 0]) 
        HIGH = np.array([3.14, 5.00, 5.00, 3.14, 3.14, 3.14])
        seed_vec = np.clip(np.array(seed_vec, dtype=np.float64), LOW, HIGH).tolist()

        req.ik_request.robot_state.joint_state.name = list(self.args.arm_joints)
        req.ik_request.robot_state.joint_state.position = seed_vec
        req.ik_request.robot_state.joint_state.name = list(self.args.arm_joints)
        req.ik_request.robot_state.joint_state.position = seed_vec
        print("current joint positions: ", self._ordered_current_joints())
        print("requesting IK target pos: ", pos)
        print("requesting IK target quat: ", quat)
        print("requesting IK seed: ", seed_vec)
        return self.ik_cli.call_async(req)

    # ----- trajectory helpers -----
    def _make_hold_point(self, positions, tfs):
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(seconds=tfs).to_msg()
        return pt

    def _grip_map(self, g: float) -> List[float]:
        g = float(np.clip(g, 0.0, 1.0))
        fo = np.array(self.args.finger_open, dtype=np.float64)
        fc = np.array(self.args.finger_closed, dtype=np.float64)
        return (fo + g*(fc-fo)).tolist()

    # ----- trajectory send (async, arm + gripper in PARALLEL) -----
    def _send_chunk_async(self, q_list, g_list, dt_list):
        if len(q_list) == 0:
            self.get_logger().info("No points to send.")
            return

        # Mark executing; track arm and gripper completion independently
        self.executing = True
        self._arm_done = False
        self._gripper_done = not bool(self.hand_ac)  # already "done" if no hand controller

        # --- ARM trajectory ---
        jt = JointTrajectory()
        jt.joint_names = self.args.arm_joints

        if self.last_sent_positions is None:
            self.last_sent_positions = self._ordered_current_joints()

        tfs0 = self.args.continuity_hold_s * self.args.time_scale
        jt.points.append(self._make_hold_point(self.last_sent_positions, tfs0))

        t_acc = tfs0
        for qsol, dtk in zip(q_list, dt_list):
            t_acc += dtk * self.args.time_scale
            pt = JointTrajectoryPoint()
            pt.positions = [qsol[n] for n in self.args.arm_joints]
            pt.time_from_start = Duration(seconds=t_acc).to_msg()
            jt.points.append(pt)

        self.last_sent_positions = jt.points[-1].positions

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        self.get_logger().info(f"[ARM] chunk: {len(q_list)} pts, dur≈{t_acc - tfs0:.2f}s (+{tfs0:.2f}s hold)")
        arm_send = self.arm_ac.send_goal_async(goal)
        arm_send.add_done_callback(self._on_arm_goal_response)

        # --- GRIPPER trajectory (sent in PARALLEL with arm) ---
        if self.hand_ac and len(g_list) > 0:
            try:
                gtraj = JointTrajectory()
                gtraj.joint_names = self.args.finger_joints

                # Hold point uses CURRENT gripper position (not target)
                current_grip = [float(self.joint_map.get(fn, 0.0))
                                for fn in self.args.finger_joints]

                grip_hold = max(tfs0, self.args.grip_min_duration)
                gtraj.points.append(self._make_hold_point(current_grip, grip_hold))

                gt_acc = grip_hold
                for gv, dtk in zip(g_list, dt_list):
                    gt_acc += max(dtk * self.args.time_scale, self.args.grip_min_duration)
                    gp = JointTrajectoryPoint()
                    gp.positions = self._grip_map(gv)
                    gp.time_from_start = Duration(seconds=gt_acc).to_msg()
                    gtraj.points.append(gp)

                ggoal = FollowJointTrajectory.Goal(trajectory=gtraj)
                ggoal.goal_time_tolerance = Duration(seconds=2.0).to_msg()

                target_pos = self._grip_map(g_list[-1])
                self.get_logger().info(
                    f"[GRIPPER] g_list={g_list}, "
                    f"finger_open={self.args.finger_open}, "
                    f"finger_closed={self.args.finger_closed}, "
                    f"current_grip={current_grip}, "
                    f"target_positions={target_pos}, "
                    f"dur≈{gt_acc:.2f}s")
                gsend = self.hand_ac.send_goal_async(ggoal)
                gsend.add_done_callback(self._on_gripper_goal_response)
            except Exception as e:
                self.get_logger().warn(f"Gripper send failed: {e}")
                self._gripper_done = True
                self._check_chunk_finished()

    # ----- action callbacks (arm + gripper independent) -----
    def _on_arm_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().error(f"Arm goal send failed: {e}")
            self._arm_done = True
            self._check_chunk_finished()
            return

        if not gh or not gh.accepted:
            self.get_logger().error("Arm trajectory rejected")
            self._arm_done = True
            self._check_chunk_finished()
            return

        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_arm_result)

    def _on_arm_result(self, fut):
        # mark when execution ended (for fresh-frame gating)
        self.last_exec_end_stamp = self.get_clock().now().to_msg()
        self._arm_done = True
        self._check_chunk_finished()

    def _on_gripper_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().warn(f"Gripper goal send failed: {e}")
            self._gripper_done = True
            self._check_chunk_finished()
            return

        if not gh or not gh.accepted:
            self.get_logger().warn("Gripper trajectory rejected")
            self._gripper_done = True
            self._check_chunk_finished()
            return

        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, fut):
        self._gripper_done = True
        self._check_chunk_finished()

    def _check_chunk_finished(self):
        """Only mark chunk as finished when BOTH arm and gripper are done."""
        if self._arm_done and self._gripper_done:
            self._on_chunk_finished()
            self.get_logger().loginfo("Both arm and gripper are done. Chunk finished")

    def _on_chunk_finished(self):
        # Reseed pose cursor after executing a chunk
        try:
            self._reseed_pose_cursor()
        except Exception as e:
            self.get_logger().warn(f"Pose reseed after chunk failed: {e}")
        self.executing = False

    # ----- timing from deltas -----
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

    # ----- server call -----
    def _encode_latest_image_b64(self) -> Optional[str]:
        if self.latest_rgb is None:
            return None
        img = Image.fromarray(self.latest_rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def _encode_latest_wrist_image_b64(self) -> Optional[str]:
        if self.latest_wrist_rgb is None:
            return None
        img = Image.fromarray(self.latest_wrist_rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def _query_server(self) -> int:
        """Return number of actions appended; 0 if none/failed."""
        img_b64 = self._encode_latest_image_b64()
        wrist_img_b64 = self._encode_latest_wrist_image_b64()
        if img_b64 is None or wrist_img_b64 is None:
            return 0

        frame_ns = None
        try:
            if self.latest_rgb_stamp is not None:
                frame_ns = ns_from_stamp(self.latest_rgb_stamp)
        except Exception:
            frame_ns = None
        payload = {
            "image_b64": img_b64,
            "wrist_image_b64": wrist_img_b64,
            "instruction": self.args.instruction,
            "state": self._proprio_state() if self.args.send_proprio else [],
            "return_chunk": bool(self.args.return_chunk)
        }

        try:
            r = requests.post(self.args.server_url, json=payload, timeout=self.args.server_timeout_s)
            if r.status_code != 200:
                self.get_logger().warn(f"/infer http {r.status_code}: {r.text[:120]}")
                return 0
            data = r.json()
            acts = data.get("actions", [])
            limit = self.args.server_chunk_limit if self.args.server_chunk_limit > 0 else len(acts)
            kept = acts[:limit]
            for a in kept:
                if len(a) == 7:
                    item = (np.asarray(a, dtype=np.float64), frame_ns)
                    self.actions_queue.append(item)
                    while len(self.actions_queue) > self.args.max_queue:
                        self.actions_queue.popleft()
                    # enforce max queue length
                    while len(self.actions_queue) > self.args.max_queue:
                        self.actions_queue.popleft()
            self.last_fetch_ns = frame_ns
            self.get_logger().info(f"Received {len(acts)} actions from server (chunk={bool(self.args.return_chunk)})")
            print("actions received from server: ", acts[0])
            if self.pub_sent_image and self.latest_rgb is not None and self.latest_rgb_stamp is not None:
                msg = cv2_to_imgmsg(self.latest_rgb, encoding="rgb8")
                msg.header = Header(stamp=self.latest_rgb_stamp, frame_id=self.args.camera_frame)
                self.pub_sent_image.publish(msg)
            return len(acts)
        except Exception as e:
            self.get_logger().warn(f"Server call failed: {e}")
            return 0

    # ----- timers -----
    def _tick_fetch(self):
        """
        Fetch another action chunk only when it makes sense:
        - either we allow overlapping queries, or the last chunk finished;
        - and we have a fresh camera frame AFTER last execution, if required;
        - and we have a new image since the last fetch;
        - and our pending action queue is low.
        """
        # Overlap guard: if not allowed and currently executing, skip
        if not self.args.query_while_executing and self.executing:
            return

        # Fresh-since-execution guard
        if self.args.require_fresh_image_after_exec and self.last_exec_end_stamp is not None:
            if self.latest_rgb_stamp is None:
                return
            if ns_from_stamp(self.latest_rgb_stamp) <= ns_from_stamp(self.last_exec_end_stamp):
                return

        if self.args.require_fresh_image_after_exec and self.last_exec_end_stamp is not None:
            if self.latest_wrist_rgb_stamp is None:
                return
            if ns_from_stamp(self.latest_wrist_rgb_stamp) <= ns_from_stamp(self.last_exec_end_stamp):
                return

        # Fresh-since-fetch guard (avoid stacking for the same frame)
        if self.last_fetch_ns is not None and self.latest_rgb_stamp is not None:
            if ns_from_stamp(self.latest_rgb_stamp) <= int(self.last_fetch_ns):
                return
        
        if self.last_fetch_ns is not None and self.latest_wrist_rgb_stamp is not None:
            if ns_from_stamp(self.latest_wrist_rgb_stamp) <= int(self.last_fetch_ns):
                return

        # Re-seed periodically
        if self.pose_cursor_pos is None or (self.chunks_seen % max(1, self.args.reseed_every) == 0 and self.chunks_seen > 0):
            try:
                self._reseed_pose_cursor()
            except Exception as e:
                self.get_logger().warn(f"Pose reseed failed: {e}")

        # Keep a small queue (jitter buffer)
        if len(self.actions_queue) < self.args.min_server_queue:
            got = self._query_server()
            if got > 0:
                self.chunks_seen += 1
    def _tick_execute(self):
        """
        High-rate loop:
          - pulls pending actions from self.actions_queue
          - integrates pose, starts async IK, buffers points on completion
          - when buffer has >= 1 point, send a trajectory (single mode)
        """
        if not self.joint_map or self.pose_cursor_pos is None:
            return

        # First: consume any completed IK future
        if self.pending_ik_future is not None:
            if not self.pending_ik_future.done():
                return  # yield to let executor process the IK response
            res = self.pending_ik_future.result()
            self.pending_ik_future = None
            meta = self.pending_ik_meta or {}
            self.pending_ik_meta = None
            if not res or res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().warn(f"IK failed (async) - code: {res.error_code.val if res else 'No response'}")
            else:
                names = res.solution.joint_state.name
                posns = res.solution.joint_state.position
                q_sol = {n: p for n, p in zip(names, posns)}
                dpos = meta.get('dpos', np.zeros(3))
                drot = meta.get('drot', np.zeros(3))
                g = float(meta.get('g', 0.0))
                q_curr_list = [q_sol[n] for n in self.args.arm_joints]
                dt_cart = self._cartesian_dt(dpos, drot)
                dt_joint = self._joint_dt(self.q_prev_list, q_curr_list)
                dt_i = min(max(dt_cart, dt_joint), self.args.max_dt)
                self.ik_points.append(q_sol)
                self.grip_vals.append(g)
                self.dt_points.append(dt_i)
                self.q_prev_list = q_curr_list
                self.get_logger().info(f"IK successful (async) - code: {res.error_code.val if res else 'No response'}")
                self.get_logger().info(f"IK solution: {q_sol}, dpos: {dpos}, drot: {drot}, g: {g}")

                self.get_logger().info(f"Executing: {self.executing}")
                self.get_logger().info(f"IK points: {len(self.ik_points)}")

        # Send single point if ready and not currently executing
        if len(self.ik_points) >= self.args.chunk_size and not self.executing:
            k = self.args.chunk_size
            q_chunk = self.ik_points[:k]
            g_chunk = self.grip_vals[:k]
            dt_chunk = self.dt_points[:k]
            self.get_logger().info(
                f"Sending chunk: {k} pts, g_chunk={g_chunk}, dt_chunk={dt_chunk}")
            # pop before sending
            self.ik_points = self.ik_points[k:]
            self.grip_vals = self.grip_vals[k:]
            self.dt_points = self.dt_points[k:]
            self._send_chunk_async(q_chunk, g_chunk, dt_chunk)

        # Start a new IK if none is pending and we have actions
        if self.pending_ik_future is None and self.actions_queue:
            self.get_logger().info("Starting a new IK")
            # Drop stale actions: those computed on a frame older than our latest
            while self.actions_queue:
                item = self.actions_queue.popleft()
                if isinstance(item, tuple) and len(item) == 2:
                    a, a_ns = item
                    latest_ns = None
                    try:
                        latest_ns = ns_from_stamp(self.latest_rgb_stamp) if self.latest_rgb_stamp is not None else None
                    except Exception:
                        latest_ns = None
                    if latest_ns is not None and a_ns is not None and a_ns < latest_ns:
                        # stale, skip
                        continue
                    else:
                        break
                else:
                    # Old untagged item; consider it stale if we have a newer frame; else accept
                    a = item
                    a_ns = None
                    latest_ns = None
                    try:
                        latest_ns = ns_from_stamp(self.latest_rgb_stamp) if self.latest_rgb_stamp is not None else None
                    except Exception:
                        latest_ns = None
                    if latest_ns is not None:
                        # drop untagged as stale
                        continue
                    else:
                        break
            else:
                return  # queue emptied by stale drops

            dpos = a[:3]; drot = a[3:6]; g = float(a[6])

            # integrate pose cursor
            self.pose_cursor_pos = self.pose_cursor_pos + dpos
            self.pose_cursor_quat = quat_norm(np.array(qmul(self.pose_cursor_quat, axis_angle_to_quat(drot))))

            print("pose_cursor_quat: ", self.pose_cursor_quat)
            print("pose_cursor_pos: ", self.pose_cursor_pos)
            print("current g", g)

            # Start async IK
            seed = dict(self.joint_map)
            if seed == [0.0]*len(self.args.arm_joints) or seed is None or sum(seed.values()) <= 0.001:
                self.get_logger().warn("No seed available, skipping IK")
            else:
                self.pending_ik_future = self._ik(self.pose_cursor_pos, self.pose_cursor_quat, seed)
                self.pending_ik_meta = {'dpos': dpos, 'drot': drot, 'g': g}
                return  # let the executor process IK

    # ----- spin -----
    def spin_forever(self):
        # wait briefly for first image + joints
        t0 = time.time()
        while ((self.latest_rgb is None) or (self.latest_wrist_rgb is None) or (not self.joint_map)) and rclpy.ok():
            if (time.time() - t0) > 3.0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        try:
            self._reseed_pose_cursor()
        except Exception as e:
            self.get_logger().warn(f"Initial pose reseed failed: {e}")

        # Initialize last_sent & q_prev from current joints
        self.q_prev_list = self._ordered_current_joints()
        self.last_sent_positions = list(self.q_prev_list)

        self.get_logger().info("Entering main loop… (Ctrl-C to stop)")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    # Server
    ap.add_argument("--server_url", required=True, help="e.g., http://SERVER:8010/infer")
    ap.add_argument("--instruction", required=True)
    ap.add_argument("--server_timeout_s", type=float, default=5.0)
    ap.add_argument("--vision_rate_hz", type=float, default=5.0, help="How often to query server (/infer)")
    ap.add_argument("--min_server_queue", type=int, default=1, help="Fetch new chunk if queue < this")
    ap.add_argument("--max_queue", type=int, default=16, help="Max actions to buffer; drop oldest when exceeded")
    ap.add_argument("--reseed_every", type=int, default=5, help="Reseed pose cursor from TF every N chunks")

    ap.add_argument("--return_chunk", action="store_true",
                    help="Ask server for a full chunk each call (default: True)")
    ap.set_defaults(return_chunk=True)
    ap.add_argument("--server_chunk_limit", type=int, default=0,
                    help="Use only the first N actions from each server chunk (0 = keep all)")

    ap.add_argument("--query_while_executing", action="store_true",
                    help="Allow querying the server while a chunk is executing")
    # ap.set_defaults(query_while_executing=True)
    ap.add_argument("--require_fresh_image_after_exec", action="store_true", default=True,
                    help="Only query server once a post-execution image has arrived")
    ap.add_argument("--send_proprio", action="store_true", default=True,
                    help="Include 8-D proprio in the server request")
    ap.add_argument("--disable_ik_constraints", action="store_true",
                    help="Disable nearest-joint constraints in IK")

    # ROS I/O
    ap.add_argument("--image_topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--wrist_image_topic", default="/camera/wrist/color/image_raw")
    ap.add_argument("--camera_frame", default="camera_color_optical_frame")
    ap.add_argument("--pub_debug_image", action="store_true",
                    help="Republish the exact frame sent to server on ~sent_image")
    ap.add_argument("--topic_enable", default="/servo_node/enable",
                    help="Optional Bool enable topic; '' to disable")

    # Frames/controllers
    ap.add_argument("--base_frame", default="j2n6s200_link_base")
    ap.add_argument("--eef_frame",  default="j2n6s200_end_effector")
    ap.add_argument("--move_group", default="jaco_arm")
    ap.add_argument("--arm_controller", default="jaco_arm_position_controller")
    ap.add_argument("--hand_controller", default="hand_controller")
    ap.add_argument("--arm_joints", nargs="+", default=[
        "j2n6s200_joint_1","j2n6s200_joint_2","j2n6s200_joint_3",
        "j2n6s200_joint_4","j2n6s200_joint_5","j2n6s200_joint_6"
    ])
    ap.add_argument("--finger_joints", nargs="+", default=["j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"])
    ap.add_argument("--finger_open",   nargs="+", type=float, default=[0.0, 0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0, 1.0])
    ap.add_argument("--grip_min_duration", type=float, default=0.5,
                    help="Minimum duration (seconds) for each gripper trajectory point")

    # IK timing & chunking
    ap.add_argument("--chunk_size", type=int, default=1, help="Minimum IK points before sending a chunk")
    ap.add_argument("--max_ik_per_tick", type=int, default=5, help="(unused w/ async-one-at-a-time IK)")
    ap.add_argument("--continuity_hold_s", type=float, default=0.01)
    ap.add_argument("--time_scale", type=float, default=1.0)

    # Speed → duration
    ap.add_argument("--v_lin_max", type=float, default=0.05)
    ap.add_argument("--v_ang_max", type=float, default=0.5)
    ap.add_argument("--joint_vel_max", type=float, default=0.8)
    ap.add_argument("--min_dt", type=float, default=0.08)
    ap.add_argument("--max_dt", type=float, default=0.6)

    # IK
    ap.add_argument("--ik_timeout_s", type=float, default=0.5)
    ap.add_argument("--constraint_tol_rad", type=float, default=1.0, help="Joint constraint tolerance in radians (how close IK must stay to previous joint positions)")

    args = ap.parse_args()

    if args.topic_enable == "":
        args.topic_enable = None

    rclpy.init()
    node = None
    try:
        node = StreamVLAtoJTC(args)
        node.spin_forever()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()