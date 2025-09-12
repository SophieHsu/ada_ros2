#!/usr/bin/env python3
# actions_to_rosbag.py  —  write Image + Servo + Gripper (fixed timing) (+ optional Path)
# Usage:
#   python3 actions_to_rosbag.py \
#     --npz_obs /data/ep.npz \
#     --npz_actions /data/ep_actions.npz \
#     --out_bag ./bags/ep_servo_w_image \
#     --base_frame j2n6s200_link_base \
#     --camera_frame camera_color_optical_frame \
#     --dt 0.1 \
#     --write_camera_info \
#     --write_path
import argparse, os, json, math, numpy as np
from pathlib import Path
from typing import Optional

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message

from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TwistStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import Image as ImageMsg, CameraInfo

# ----------------- helpers -----------------
def load_npz(path: Path):
    D = np.load(path, allow_pickle=True)
    imgs = D["images"] if "images" in D else D["image_primary"]      # [T,H,W,3] uint8 (RGB)
    lang = str(D["language"]) if "language" in D else str(D["instruction"])
    states = None
    if "states" in D: states = D["states"].astype(np.float32)        # [T,8] (x,y,z,qx,qy,qz,qw,g)
    elif "proprio" in D: states = D["proprio"].astype(np.float32)
    ts = D["timestamps_ns"] if "timestamps_ns" in D else None        # [T] int64 ns (optional)
    return imgs, lang, states, ts

def load_actions(path: Path):
    D = np.load(path, allow_pickle=True)
    if "pred_actions" not in D:
        raise ValueError("pred_actions not found in action NPZ. Run your inference saver first.")
    A = D["pred_actions"].astype(np.float32)                          # [T,7] (dx,dy,dz, dRx,dRy,dRz, g_abs)
    ts = D["timestamps_ns"] if "timestamps_ns" in D else None
    meta = {}
    if "meta" in D:
        try: meta = json.loads(str(D["meta"].item()))
        except Exception: pass
    return A, ts, meta

def to_time_msg_from_ns(t_ns: int) -> TimeMsg:
    return TimeMsg(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000))

def make_image_msg(rgb: np.ndarray, frame_id: str, stamp: TimeMsg) -> ImageMsg:
    H, W = rgb.shape[:2]
    msg = ImageMsg()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = H; msg.width = W
    msg.encoding = "rgb8"; msg.is_bigendian = 0
    msg.step = W * 3
    msg.data = rgb.tobytes()
    return msg

def make_camera_info(W, H, frame_id, stamp, fx=None, fy=None, cx=None, cy=None):
    fx = fx if fx is not None else float(W)
    fy = fy if fy is not None else float(W)
    cx = cx if cx is not None else float(W)/2.0
    cy = cy if cy is not None else float(H)/2.0
    K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam = CameraInfo()
    cam.header = Header(stamp=stamp, frame_id=frame_id)
    cam.width = W; cam.height = H
    cam.distortion_model = "plumb_bob"
    cam.d = []; cam.k = K; cam.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]; cam.p = P
    return cam

def quat_from_axis_angle(ax):
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12:
        return np.array([0.0,0.0,0.0,1.0], dtype=np.float64)
    axis = (ax/angle).astype(np.float64)
    s = math.sin(0.5*angle)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(0.5*angle)], dtype=np.float64)
def quat_mul(q1,q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2], dtype=np.float64)
def quat_norm(q):
    n=float(np.linalg.norm(q));  return q/n if n>1e-12 else np.array([0,0,0,1], np.float64)

# ----------------- main -----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz_obs", required=True, help="original episode (images, optional states/timestamps)")
    ap.add_argument("--npz_actions", required=True, help="from your inference step (pred_actions [T,7])")
    ap.add_argument("--out_bag", required=True)
    # timing
    ap.add_argument("--dt", type=float, default=0.1, help="sec per step if timestamps missing")
    # frames/topics
    ap.add_argument("--base_frame", default="j2n6s200_link_base")
    ap.add_argument("--camera_frame", default="camera_color_optical_frame")
    ap.add_argument("--topic_image", default="/camera/camera/color/image_raw")
    ap.add_argument("--topic_camerainfo", default="/camera/camera/color/camera_info")
    ap.add_argument("--topic_twist", default="/servo_node/delta_twist_cmds")
    ap.add_argument("--topic_grip", default="/hand_controller/joint_trajectory")
    # Fix for trajectory timing
    ap.add_argument("--trajectory_start_mode", choices=["now","future","bag_stamp"], default="now",
                    help="'now' sets JointTrajectory.header.stamp = 0 (recommended). "
                         "'future' shifts start slightly into the future relative to bag stamp. "
                         "'bag_stamp' uses the bag timestamp (may cause 'ends in the past').")
    ap.add_argument("--future_offset_ns", type=int, default=100_000_000,
                    help="Only used when --trajectory_start_mode=future (default 0.1s)")
    ap.add_argument("--grip_hold_ns", type=int, default=200_000_000,
                    help="time_from_start for the gripper point (>=50 ms).")
    # optional predicted path
    ap.add_argument("--write_path", action="store_true")
    ap.add_argument("--topic_pose", default="/predicted_pose")
    ap.add_argument("--topic_path", default="/predicted_path")
    # camera intrinsics (optional)
    ap.add_argument("--fx", type=float); ap.add_argument("--fy", type=float)
    ap.add_argument("--cx", type=float); ap.add_argument("--cy", type=float)
    ap.add_argument("--write_camera_info", action="store_true")
    # gripper mapping
    ap.add_argument("--finger_joints", nargs="+", default=["j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"])
    ap.add_argument("--finger_open", nargs="+", type=float, default=[0.0,0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0,1.0])
    ap.add_argument("--write_servo_enable", action="store_true")

    args = ap.parse_args()

    imgs, _lang, states, ts_obs = load_npz(Path(args.npz_obs))
    A, ts_act, meta = load_actions(Path(args.npz_actions))

    T = min(len(imgs), len(A))
    if T == 0:
        raise ValueError("No frames/actions to write.")

    # choose timeline: prefer actions ts, else obs ts, else synth
    dt_ns_cfg = int(args.dt * 1e9)
    if ts_act is not None and len(ts_act) >= T:
        ts = ts_act
    elif ts_obs is not None and len(ts_obs) >= T:
        ts = ts_obs
    else:
        t0 = 0
        ts = np.array([t0 + i*dt_ns_cfg for i in range(T)], dtype=np.int64)

    # Writer
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(args.out_bag), storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    if args.write_servo_enable:
        writer.create_topic(TopicMetadata(name="/servo_node/enable",
                                        type="std_msgs/msg/Bool",
                                        serialization_format="cdr"))
        t0_ns = int(ts[0])
        from std_msgs.msg import Bool
        enable_msg = Bool()
        enable_msg.data = True
        writer.write("/servo_node/enable", serialize_message(enable_msg), t0_ns)
    # Topics
    writer.create_topic(TopicMetadata(name=args.topic_image, type="sensor_msgs/msg/Image", serialization_format="cdr"))
    if args.write_camera_info:
        writer.create_topic(TopicMetadata(name=args.topic_camerainfo, type="sensor_msgs/msg/CameraInfo", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name=args.topic_twist, type="geometry_msgs/msg/TwistStamped", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name=args.topic_grip, type="trajectory_msgs/msg/JointTrajectory", serialization_format="cdr"))
    if args.write_path:
        writer.create_topic(TopicMetadata(name=args.topic_pose, type="geometry_msgs/msg/PoseStamped", serialization_format="cdr"))
        writer.create_topic(TopicMetadata(name=args.topic_path, type="nav_msgs/msg/Path", serialization_format="cdr"))
        path_msg = PathMsg(); path_msg.header.frame_id = args.base_frame
        # init pose from first state (if available)
        if states is not None and len(states)>0:
            pos = states[0][:3].astype(np.float64)
            quat = states[0][3:7].astype(np.float64)
            if len(states[0])>=8: grip_state = float(states[0][7])
            else: grip_state = 0.0
        else:
            pos = np.zeros(3, np.float64); quat = np.array([0,0,0,1], np.float64); grip_state = 0.0

    # gripper mapping
    open_pos = np.array(args.finger_open, dtype=np.float64)
    closed_pos = np.array(args.finger_closed, dtype=np.float64)
    def grip_to_joints(g):
        g = float(np.clip(g, 0.0, 1.0))
        return (open_pos + g*(closed_pos - open_pos)).tolist()

    H, W = imgs[0].shape[:2]

    # Loop
    for i in range(T):
        t_ns = int(ts[i])
        stamp = to_time_msg_from_ns(t_ns)

        # (1) image
        img_msg = make_image_msg(imgs[i], frame_id=args.camera_frame, stamp=stamp)
        writer.write(args.topic_image, serialize_message(img_msg), t_ns)
        if args.write_camera_info:
            cam = make_camera_info(W,H, frame_id=args.camera_frame, stamp=stamp,
                                   fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy)
            writer.write(args.topic_camerainfo, serialize_message(cam), t_ns)

        # (2) servo twist (vel from delta/Δt)
        a = A[i].astype(np.float64)  # [dx,dy,dz, dRx,dRy,dRz, g_abs]
        if i > 0:
            dt_i = (t_ns - int(ts[i-1])) * 1e-9
            if dt_i <= 1e-6: dt_i = dt_ns_cfg * 1e-9
        else:
            dt_i = dt_ns_cfg * 1e-9
        v_lin = a[:3] / dt_i
        v_ang = a[3:6] / dt_i
        g_abs = float(a[6])

        tw = TwistStamped()
        # Twist header timestamp can be bag-stamp; Servo generally ignores it.
        tw.header = Header(stamp=stamp, frame_id=args.base_frame)
        tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = map(float, v_lin.tolist())
        tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = map(float, v_ang.tolist())
        writer.write(args.topic_twist, serialize_message(tw), t_ns)

        # (3) gripper JointTrajectory — FIXED: start "now" by default
        jt = JointTrajectory()
        if args.trajectory_start_mode == "now":
            # Start immediately when controller receives it
            jt.header = Header(stamp=TimeMsg(sec=0, nanosec=0), frame_id=args.base_frame)
        elif args.trajectory_start_mode == "future":
            future_ns = t_ns + int(args.future_offset_ns)
            jt.header = Header(stamp=to_time_msg_from_ns(future_ns), frame_id=args.base_frame)
        else:  # 'bag_stamp' (may cause "past" errors if playback or processing lags)
            jt.header = Header(stamp=stamp, frame_id=args.base_frame)

        jt.joint_names = list(args.finger_joints)
        pt = JointTrajectoryPoint()
        pt.positions = grip_to_joints(g_abs)
        # ensure strictly >0 duration
        hold_ns = max(50_000_000, int(args.grip_hold_ns))  # >=50 ms
        pt.time_from_start.sec = int(hold_ns // 1_000_000_000)
        pt.time_from_start.nanosec = int(hold_ns % 1_000_000_000)
        jt.points = [pt]
        writer.write(args.topic_grip, serialize_message(jt), t_ns)

        # (4) optional visual path (integrate deltas)
        if args.write_path:
            pos = pos + a[:3]
            dq = quat_from_axis_angle(a[3:6])
            quat = quat_norm(quat_mul(quat, dq))
            ps = PoseStamped()
            ps.header.frame_id = args.base_frame
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos.tolist())
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, quat.tolist())
            path_msg.header.stamp = stamp
            path_msg.poses.append(ps)
            writer.write(args.topic_pose, serialize_message(ps), t_ns)
            writer.write(args.topic_path, serialize_message(path_msg), t_ns)

    print(f"[ok] wrote bag with {T} steps → {args.out_bag}")
    print(f"  Image:        {args.topic_image} (frame={args.camera_frame})")
    if args.write_camera_info:
        print(f"  CameraInfo:   {args.topic_camerainfo}")
    print(f"  Servo Twist:  {args.topic_twist}")
    print(f"  Gripper Traj: {args.topic_grip}  (start_mode={args.trajectory_start_mode})")
    if args.write_path:
        print(f"  Path/Pose:    {args.topic_path}, {args.topic_pose}")
    print("If needed, run consumers with use_sim_time:=true so they follow /clock from the bag.")

if __name__ == "__main__":
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL","3")
    main()
