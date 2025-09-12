#!/usr/bin/env python3
"""
Closed-loop MoveIt Servo execution from a saved NPZ of OpenVLA actions.

Input NPZ must contain:
  - pred_actions: [T, 7]  with [dx, dy, dz, dRx, dRy, dRz, grip]
  - (optional) timestamps_ns: [T] int64 (unused here; we compute our own hold)
  - (optional) meta: json string (ignored)

Behavior:
  - For each step i, compute a hold time T from your max linear/angular speeds.
  - Stream a constant Twist (velocities = delta / T) at control_rate_hz.
  - Stop earlier if pose error (EEF in base frame) is below tolerances.
  - Send gripper as a JointTrajectory (absolute openness g ∈ [0,1]).

Requirements:
  - MoveIt Servo running, configured with:
      command_in_type: "speed_units"
      robot_link_command_frame: (e.g.) j2n6s200_link_base
      cartesian_command_in_topic: ~/delta_twist_cmds  -> /servo_node/delta_twist_cmds
  - Active JointTrajectoryController for the gripper (hand_controller).
"""

import argparse, math, time, json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header, Bool
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_multiply as qmul, quaternion_inverse as qinv

# ---------------- math helpers ----------------
def axis_angle_to_quat(ax):
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12: return np.array([0.,0.,0.,1.], np.float64)
    axis = (ax/angle).astype(np.float64)
    s = math.sin(0.5*angle)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(0.5*angle)], np.float64)

def quat_to_axis_angle(q_rel):
    q = q_rel / max(1e-12, np.linalg.norm(q_rel))
    w = np.clip(q[3], -1.0, 1.0)
    angle = 2.0*math.acos(w)
    s = math.sqrt(max(1e-12, 1.0 - w*w))
    if s < 1e-8:
        return np.array([1.,0.,0.], np.float64), 0.0
    return np.array([q[0]/s, q[1]/s, q[2]/s], np.float64), angle

# ---------------- node ----------------
class ActionsServoNode(Node):
    def __init__(self, args):
        super().__init__("vla_actions_closed_loop_servo")
        self.args = args

        # TF feedback
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self, spin_thread=True)

        # Optional joint state tap (health check)
        self.latest_js_names = []
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Publishers
        self.pub_twist = self.create_publisher(TwistStamped, self.args.topic_twist, 10)
        self.pub_grip  = self.create_publisher(JointTrajectory, self.args.topic_grip, 10)
        self.pub_enable = None
        if self.args.topic_enable:
            self.pub_enable = self.create_publisher(Bool, self.args.topic_enable, 10)

        # Load actions NPZ
        self.actions, self.timestamps, self.meta = self._load_actions(self.args.actions_npz)
        self.get_logger().info(f"Loaded actions: {self.actions.shape} steps")

        # Gripper mapping
        self.open_pos   = np.array(self.args.finger_open,   np.float64)
        self.closed_pos = np.array(self.args.finger_closed, np.float64)

        # Optionally scale deltas (if your saved actions are in normalized units)
        if self.args.scale_lin != 1.0 or self.args.scale_ang != 1.0:
            self.get_logger().info(f"Scaling deltas: lin x{self.args.scale_lin}, ang x{self.args.scale_ang}")
            self.actions[:, :3]  *= float(self.args.scale_lin)
            self.actions[:, 3:6] *= float(self.args.scale_ang)

    # ---------- I/O ----------
    def _load_actions(self, path):
        D = np.load(path, allow_pickle=True)
        if "pred_actions" not in D:
            raise ValueError("NPZ must contain 'pred_actions' [T,7].")
        A = D["pred_actions"].astype(np.float64)
        ts = D["timestamps_ns"] if "timestamps_ns" in D else None
        meta = {}
        if "meta" in D:
            try: meta = json.loads(str(D["meta"].item()))
            except Exception: pass
        return A, ts, meta

    def _on_js(self, msg: JointState):
        self.latest_js_names = msg.name

    # ---------- TF feedback ----------
    def current_pose(self):
        # Latest transform (Time() == 0)
        if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(), timeout=Duration(seconds=2.0)):
            raise RuntimeError(f"TF {self.args.base_frame}->{self.args.eef_frame} not available")
        tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(), timeout=Duration(seconds=0.5))
        t = tf.transform.translation; q = tf.transform.rotation
        pos  = np.array([t.x, t.y, t.z], dtype=np.float64)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        return pos, quat

    # ---------- Gripper ----------
    def grip_to_joints(self, g):
        g = float(np.clip(g, 0.0, 1.0))
        return (self.open_pos + g*(self.closed_pos - self.open_pos)).tolist()

    def send_gripper_now(self, g, hold_s=0.2):
        jt = JointTrajectory()
        jt.header = Header()
        jt.header.stamp = self.get_clock().now().to_msg()  # Add proper timestamp
        jt.joint_names = list(self.args.finger_joints)
        pt = JointTrajectoryPoint()
        pt.positions = self.grip_to_joints(g)
        pt.time_from_start.sec = int(hold_s)
        pt.time_from_start.nanosec = int((hold_s - int(hold_s))*1e9)
        jt.points = [pt]
        self.pub_grip.publish(jt)

    # ---------- Twist ----------
    def publish_twist(self, v_lin, v_ang):
        tw = TwistStamped()
        tw.header = Header()
        tw.header.stamp = self.get_clock().now().to_msg()  # Add proper timestamp
        tw.header.frame_id = self.args.command_frame  # must match Servo robot_link_command_frame
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = map(float, v_lin.tolist())
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = map(float, v_ang.tolist())
        self.pub_twist.publish(tw)

    # ---------- Execute one delta (time + feedback gating) ----------
    def execute_delta(self, dpos, drot, grip):
        # Compute base hold T from speed limits
        T_lin = np.linalg.norm(dpos) / max(self.args.v_lin_max, 1e-6)
        T_ang = np.linalg.norm(drot) / max(self.args.v_ang_max, 1e-6)
        T = float(max(self.args.min_hold_s, T_lin, T_ang, 1e-3))

        v_lin = dpos / T
        v_ang = drot / T

        # Clamp velocities as a safety
        lin_norm = np.linalg.norm(v_lin)
        if lin_norm > self.args.v_lin_max + 1e-9:
            v_lin *= (self.args.v_lin_max / lin_norm)
        ang_norm = np.linalg.norm(v_ang)
        if ang_norm > self.args.v_ang_max + 1e-9:
            v_ang *= (self.args.v_ang_max / ang_norm)

        # Build target from *current* pose (receding-horizon)
        try:
            pos0, quat0 = self.current_pose()
        except Exception as e:
            self.get_logger().warn(f"TF not ready, running time-only gating this step: {e}")
            pos0 = None; quat0 = None

        if pos0 is not None:
            pos_goal = pos0 + dpos
            quat_goal = qmul(quat0, axis_angle_to_quat(drot))

        # Send gripper (absolute openness)
        if self.args.send_gripper_each_step:
            self.send_gripper_now(float(grip), hold_s=self.args.gripper_hold_s)

        # Stream twist
        t_start = time.time()
        dt = 1.0 / self.args.control_rate_hz
        while True:
            self.publish_twist(v_lin, v_ang)

            # Feedback stop
            if pos0 is not None and self.args.use_feedback:
                try:
                    pos_cur, quat_cur = self.current_pose()
                    e_pos = pos_goal - pos_cur
                    q_rel = qmul(quat_goal, qinv(quat_cur))
                    axis, ang = quat_to_axis_angle(q_rel)
                    pos_err = float(np.linalg.norm(e_pos))
                    ang_err = float(abs(ang))
                    if pos_err <= self.args.pos_tol_m and ang_err <= self.args.ang_tol_rad:
                        break
                except Exception as e:
                    # TF hiccup: ignore for this tick
                    pass

            # Time stop
            if (time.time() - t_start) >= T * self.args.max_hold_scale:
                break

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    # ---------- Run all steps ----------
    def run(self):
        # Best-effort enable
        if self.pub_enable:
            self.get_logger().info(f"Publishing enable on {self.args.topic_enable}")
            self.pub_enable.publish(Bool(data=True))

        # Warm-up TF
        self.get_logger().info("Waiting for TF (base->eef) ...")
        t0 = time.time()
        while rclpy.ok():
            ok = self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(), timeout=Duration(seconds=0.1))
            if ok or (time.time() - t0) > 2.0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        # Optionally pre-set gripper to first value
        if self.actions.shape[0] > 0 and not self.args.send_gripper_each_step:
            self.send_gripper_now(float(self.actions[0, 6]), hold_s=self.args.gripper_hold_s)

        self.get_logger().info(f"Executing {self.actions.shape[0]} action steps ...")
        for i in range(self.actions.shape[0]):
            if not rclpy.ok():
                break
            d = self.actions[i]
            dpos = d[:3]
            drot = d[3:6]
            grip = d[6]
            self.execute_delta(dpos, drot, grip)

            # Pace like a vision loop (optional)
            time.sleep(max(0.0, 1.0 / self.args.vision_rate_hz))

        self.get_logger().info("Done.")

# ---------------- entry ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--actions_npz", required=True, help="NPZ with 'pred_actions' [T,7] and optional 'timestamps_ns'")
    # Frames / topics
    ap.add_argument("--command_frame", default="j2n6s200_link_base", help="Must match Servo robot_link_command_frame")
    ap.add_argument("--base_frame",    default="j2n6s200_link_base")
    ap.add_argument("--eef_frame",     default="j2n6s200_end_effector")
    ap.add_argument("--topic_twist",   default="/servo_node/delta_twist_cmds")
    ap.add_argument("--topic_grip",    default="/hand_controller/joint_trajectory")
    ap.add_argument("--topic_enable",  default="/servo_node/enable", help="'' to disable publishing enable")
    # Control rates & limits
    ap.add_argument("--control_rate_hz", type=float, default=30.0)
    ap.add_argument("--vision_rate_hz",  type=float, default=3.0, help="Spacing between steps; set >0")
    ap.add_argument("--v_lin_max", type=float, default=0.08, help="m/s limit for linear twist")
    ap.add_argument("--v_ang_max", type=float, default=0.8,  help="rad/s limit for angular twist")
    ap.add_argument("--min_hold_s", type=float, default=0.15, help="minimum hold per step")
    ap.add_argument("--max_hold_scale", type=float, default=1.5, help="allow up to T*scale before forcing next step")
    # Feedback gating
    ap.add_argument("--use_feedback", action="store_true", default=True)
    ap.add_argument("--pos_tol_m",    type=float, default=0.005)
    ap.add_argument("--ang_tol_rad",  type=float, default=math.radians(3.0))
    # Gripper
    ap.add_argument("--send_gripper_each_step", action="store_true", help="else sent once at first step")
    ap.add_argument("--gripper_hold_s", type=float, default=0.2)
    ap.add_argument("--finger_joints", nargs="+",
                    default=["j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"])
    ap.add_argument("--finger_open",   nargs="+", type=float, default=[0.0, 0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0, 1.0])
    # Optional scaling if your saved actions are normalized
    ap.add_argument("--scale_lin", type=float, default=1.0, help="multiply dpos by this")
    ap.add_argument("--scale_ang", type=float, default=1.0, help="multiply drot by this")
    args = ap.parse_args()

    if args.topic_enable == "": args.topic_enable = None

    rclpy.init()
    node = ActionsServoNode(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
