#!/usr/bin/env python3
import argparse, math, numpy as np, rclpy, time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_multiply as qmul

def axis_angle_to_quat(ax):
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12: return np.array([0,0,0,1], dtype=np.float64)
    axis = (ax/angle).astype(np.float64); s = math.sin(0.5*angle)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(0.5*angle)], dtype=np.float64)

class VlaToJTC(Node):
    def __init__(self, args):
        super().__init__("vla_to_jtc_streaming")
        self.args = args

        # TF + joint state
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self, spin_thread=True)
        self.joint_map = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)

        # IK service
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

        # Action clients
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   f"/{args.arm_controller}/follow_joint_trajectory")
        self.hand_ac = ActionClient(self, FollowJointTrajectory,
                                    f"/{args.hand_controller}/follow_joint_trajectory") if args.hand_controller else None

        # Buffers
        self.ik_points = []        # list[dict name->pos]
        self.grip_vals = []        # list[float in 0..1]
        self.dt_points = []        # list[float] per accepted IK point
        self.last_sent_positions = None  # list[float] matching arm_joints order
        self.q_prev_list = None    # previous IK solution (ordered list)

    # ---------------- helpers ----------------
    def _on_js(self, msg: JointState):
        self.joint_map = {n:p for n,p in zip(msg.name, msg.position)}

    def wait_ready(self):
        self.get_logger().info("Waiting for IK service …")
        self.ik_cli.wait_for_service()
        self.get_logger().info("Waiting for arm JTC action …")
        self.arm_ac.wait_for_server()
        if self.hand_ac:
            self.get_logger().info("Waiting for hand JTC action …")
            self.hand_ac.wait_for_server()

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
    
    def _ordered_current_joints(self):
        """Return current joints in self.args.arm_joints order (fallback to 0.0)."""
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
        # clean, ordered seed vector for the arm group
        return [float(seed_positions.get(n, self.joint_map.get(n, 0.0))) for n in self.args.arm_joints]

    def ik(self, pos, quat, seed_positions):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.args.move_group
        req.ik_request.ik_link_name = self.args.eef_frame
        req.ik_request.timeout = Duration(seconds=self.args.ik_timeout_s).to_msg()
        if hasattr(self, "q_prev_list") and self.q_prev_list is not None:
            req.ik_request.constraints = self._nearest_constraints(self.q_prev_list)
        else:
            req.ik_request.constraints = Constraints()

        # Target pose
        ps = PoseStamped()
        ps.header.frame_id = self.args.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat.tolist()
        req.ik_request.pose_stamped = ps

        # Seed (ordered vector)
        seed_vec = self._ordered_seed_from_map(seed_positions)
        req.ik_request.robot_state.joint_state.name = list(self.args.arm_joints)
        req.ik_request.robot_state.joint_state.position = seed_vec

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            return None
        names = res.solution.joint_state.name
        posns = res.solution.joint_state.position
        return {n:p for n,p in zip(names, posns)}

    def grip_map(self, g):
        g = float(np.clip(g, 0.0, 1.0))
        fo = np.array(self.args.finger_open, dtype=np.float64)
        fc = np.array(self.args.finger_closed, dtype=np.float64)
        return (fo + g*(fc-fo)).tolist()

    def make_hold_point(self, positions, tfs):
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(seconds=tfs).to_msg()
        return pt

    def send_chunk_and_wait(self, q_list, g_list, dt_list):
        """
        Send a chunk of joint positions with per-point timing.
        Adds a short continuity point at t = continuity_hold_s, then
        schedules points at t = continuity_hold_s + cumsum(dt_list).
        """
        if len(q_list) == 0:
            return

        # Arm trajectory
        jt = JointTrajectory()
        jt.joint_names = self.args.arm_joints

        # Continuity point: last sent (or current joints)
        if self.last_sent_positions is None:
            cur = self._ordered_current_joints()
            self.last_sent_positions = cur

        tfs0 = self.args.continuity_hold_s * self.args.time_scale
        jt.points.append(self.make_hold_point(self.last_sent_positions, tfs0))

        # Append chunk with cumulative times
        t_acc = tfs0
        for k, (qsol, dtk) in enumerate(zip(q_list, dt_list), start=1):
            t_acc += dtk * self.args.time_scale
            pt = JointTrajectoryPoint()
            pt.positions = [qsol[n] for n in self.args.arm_joints]
            pt.time_from_start = Duration(seconds=t_acc).to_msg()
            jt.points.append(pt)

        # Keep last for next chunk continuity
        self.last_sent_positions = jt.points[-1].positions

        # Send arm chunk (block until done)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        self.get_logger().info(f"[ARM] Sending chunk: {len(q_list)} pts, "
                               f"dur ~{t_acc - tfs0:.2f}s (+{tfs0:.2f}s hold)")

        arm_send = self.arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, arm_send)
        arm_goal_handle = arm_send.result()
        if not arm_goal_handle.accepted:
            self.get_logger().error("Arm trajectory goal was rejected")
            return
        arm_result_future = arm_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, arm_result_future)
        _res = arm_result_future.result()

        # Gripper chunk (optional, matching timing)
        if self.hand_ac and len(g_list) > 0:
            gtraj = JointTrajectory()
            gtraj.joint_names = self.args.finger_joints

            # continuity hold
            g0 = g_list[0]
            gtraj.points.append(self.make_hold_point(self.grip_map(g0), tfs0))

            t_acc = tfs0
            for k, (g, dtk) in enumerate(zip(g_list, dt_list), start=1):
                t_acc += dtk * self.args.time_scale
                gp = JointTrajectoryPoint()
                gp.positions = self.grip_map(g)
                gp.time_from_start = Duration(seconds=t_acc).to_msg()
                gtraj.points.append(gp)

            self.get_logger().info(f"[GRIP] Sending chunk: {len(g_list)} pts")
            gsend = self.hand_ac.send_goal_async(FollowJointTrajectory.Goal(trajectory=gtraj))
            rclpy.spin_until_future_complete(self, gsend)
            ggoal_h = gsend.result()
            if ggoal_h.accepted:
                gresult_future = ggoal_h.get_result_async()
                rclpy.spin_until_future_complete(self, gresult_future)

    # ---- duration from delta (cartesian) & joint motion ----
    def _cartesian_dt(self, dpos, drot):
        # drot is axis-angle in rad (norm == angle)
        lin = float(np.linalg.norm(dpos)) / max(self.args.v_lin_max, 1e-9)
        ang = float(np.linalg.norm(drot)) / max(self.args.v_ang_max, 1e-9)
        return max(lin, ang, self.args.min_dt)

    def _joint_dt(self, q_prev, q_curr):
        if q_prev is None:
            return self.args.min_dt
        dq = np.abs(np.array(q_curr) - np.array(q_prev))
        dq_max = float(np.max(dq)) if dq.size > 0 else 0.0
        return max(dq_max / max(self.args.joint_vel_max, 1e-9), self.args.min_dt)

    # ---------------- main episode ----------------
    def run_episode(self, actions_npz):
        D = np.load(actions_npz, allow_pickle=True)
        A = D["pred_actions"].astype(np.float64)  # [T,7] (dx,dy,dz, dRx,dRy,dRz, grip)

        # start pose
        pos, quat = self.current_eef_pose()
        seed = dict(self.joint_map)

        # Reset buffers
        self.ik_points.clear()
        self.grip_vals.clear()
        self.dt_points.clear()
        self.last_sent_positions = None
        self.q_prev_list = None

        t0 = time.time()
        while not self.joint_map and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.q_prev_list = self._ordered_current_joints()
        self.last_sent_positions = list(self.q_prev_list)

        idx_sent = 0
        for i, a in enumerate(A):
            dpos = a[:3]; drot = a[3:6]; g = float(a[6])
            pos = pos + dpos
            quat = np.array(qmul(quat, axis_angle_to_quat(drot)), dtype=np.float64)

            q_sol = self.ik(pos, quat, seed)
            if q_sol is None:
                self.get_logger().warn(f"IK failed at step {i}, skipping point")
                continue

            # build ordered list for joints
            q_curr_list = [q_sol[n] for n in self.args.arm_joints]

            # compute per-point dt from cartesian & joint-space needs
            dt_cart = self._cartesian_dt(dpos, drot)
            dt_joint = self._joint_dt(self.q_prev_list, q_curr_list)
            dt_i = max(dt_cart, dt_joint)
            dt_i = min(dt_i, self.args.max_dt)  # cap

            # update seeds/previous
            seed.update(q_sol)
            self.q_prev_list = q_curr_list

            # buffer
            self.ik_points.append(q_sol)
            self.grip_vals.append(g)
            self.dt_points.append(dt_i)

            # Start publishing once we have at least chunk_size buffered
            if len(self.ik_points) - idx_sent >= self.args.chunk_size:
                q_chunk = self.ik_points[idx_sent: idx_sent + self.args.chunk_size]
                g_chunk = self.grip_vals[idx_sent: idx_sent + self.args.chunk_size]
                dt_chunk = self.dt_points[idx_sent: idx_sent + self.args.chunk_size]
                self.send_chunk_and_wait(q_chunk, g_chunk, dt_chunk)
                idx_sent += self.args.chunk_size
                # If IK is slower than execution, the controller holds last point.

        # Flush remaining < chunk_size
        if len(self.ik_points) > idx_sent:
            q_chunk = self.ik_points[idx_sent:]
            g_chunk = self.grip_vals[idx_sent:]
            dt_chunk = self.dt_points[idx_sent:]
            self.send_chunk_and_wait(q_chunk, g_chunk, dt_chunk)

        self.get_logger().info("Episode done.")

# ---------------- entry ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--actions_npz", required=True)
    ap.add_argument("--base_frame", default="j2n6s200_link_base")
    ap.add_argument("--eef_frame",  default="j2n6s200_end_effector")  # or tool TCP
    ap.add_argument("--move_group", default="jaco_arm")
    ap.add_argument("--arm_controller", default="jaco_arm_servo_controller")  # JTC name
    ap.add_argument("--arm_joints", nargs="+", default=[
        "j2n6s200_joint_1","j2n6s200_joint_2","j2n6s200_joint_3",
        "j2n6s200_joint_4","j2n6s200_joint_5","j2n6s200_joint_6"
    ])

    # Timing & chunking
    ap.add_argument("--chunk_size", type=int, default=10, help="Min buffered IK points before first publish")
    ap.add_argument("--continuity_hold_s", type=float, default=0.05,
                    help="Initial hold per chunk at last position to ensure continuity")

    # Speed → duration mapping (cartesian & joint-based)
    ap.add_argument("--v_lin_max", type=float, default=0.05, help="m/s cap used to compute dt from ‖dpos‖")
    ap.add_argument("--v_ang_max", type=float, default=0.5,  help="rad/s cap used to compute dt from ‖drot‖")
    ap.add_argument("--joint_vel_max", type=float, default=0.8, help="rad/s cap per joint (uniform bound)")
    ap.add_argument("--min_dt", type=float, default=0.08, help="floor per-step duration (s)")
    ap.add_argument("--max_dt", type=float, default=0.6,  help="ceiling per-step duration (s)")
    ap.add_argument("--time_scale", type=float, default=1.0, help="multiply all times globally (>1 = slower)")

    # IK settings
    ap.add_argument("--ik_timeout_s", type=float, default=0.25)

    # Optional gripper control
    ap.add_argument("--hand_controller", default="hand_controller")
    ap.add_argument("--finger_joints", nargs="+", default=["j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"])
    ap.add_argument("--finger_open",   nargs="+", type=float, default=[0.0, 0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0, 1.0])

    args = ap.parse_args()

    rclpy.init()
    try:
        node = VlaToJTC(args)
        node.wait_ready()
        node.run_episode(args.actions_npz)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
