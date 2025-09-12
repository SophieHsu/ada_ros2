import argparse, os, math, io, struct
import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional

# rosbag2 + ROS2 types
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeMsg

import cv2
from tf2_ros import Buffer
from tf2_ros.transform_broadcaster import toMsg as _unused  # just to ensure tf2 is available
from tf2_py import BufferCoreInterface  # installed with tf2_py

import tensorflow as tf
from tf_transformations import euler_from_quaternion

def ns(stamp) -> int: return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)

def add_tf_to_buffer(buf: Buffer, t: TransformStamped, is_static=False):
    """
    Feed a single TransformStamped into Buffer.
    In Python we use buf.set_transform (dynamic) / set_transform_static (static).
    """
    if is_static:
        buf.set_transform_static(t, "bag_reader")
    else:
        buf.set_transform(t, "bag_reader")

def encode_image_bytes(msg: Image, prefer_jpeg=True) -> bytes:
    """Return compressed bytes (jpeg or png) from a sensor_msgs/Image RGB8."""
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if arr.shape[2] == 3:
        bgr = arr[:, :, ::-1]  # OpenCV expects BGR
    else:
        # Fallback: take first three channels
        bgr = arr[:, :, :3][:, :, ::-1]
    if prefer_jpeg:
        ok, enc = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if ok: return enc.tobytes()
    ok, enc = cv2.imencode('.png', bgr)
    return enc.tobytes() if ok else bytes()

def make_example(image_bytes: bytes, state: np.ndarray, image_ns: int, joints_ns: int,
                 cam_info: Optional[CameraInfo]):
    def _bytes(v): return tf.train.Feature(bytes_list=tf.train.BytesList(value=[v]))
    def _floats(v): return tf.train.Feature(float_list=tf.train.FloatList(value=list(v)))
    def _int64(v): return tf.train.Feature(int64_list=tf.train.Int64List(value=[int(v)]))
    feats = {
        "image_primary": _bytes(image_bytes),
        "state": _floats(state.astype(np.float32)),
        "timestamps/image_ns": _int64(image_ns),
        "timestamps/joints_ns": _int64(joints_ns),
    }
    if cam_info:
        feats.update({
            "camera_info/width":  _int64(cam_info.width),
            "camera_info/height": _int64(cam_info.height),
            "camera_info/fx": _floats([cam_info.k[0]]),
            "camera_info/fy": _floats([cam_info.k[4]]),
            "camera_info/cx": _floats([cam_info.k[2]]),
            "camera_info/cy": _floats([cam_info.k[5]]),
        })
    return tf.train.Example(features=tf.train.Features(feature=feats))

def gripper_from_joints(pos_map: Dict[str, float], finger_names, open_pos, closed_pos) -> float:
    vals = []
    for i, name in enumerate(finger_names):
        if name in pos_map and i < len(open_pos) and i < len(closed_pos):
            num = pos_map[name] - open_pos[i]
            den = max(1e-6, closed_pos[i] - open_pos[i])
            vals.append(np.clip(num/den, 0.0, 1.0))
    return float(np.mean(vals)) if vals else 0.0

def nearest_joint(ordered_js: deque, t_ns: int, slop_ns: int) -> Optional[Tuple[int, Dict[str,float]]]:
    """
    ordered_js: deque of (ns, {name:pos}), chronological.
    Returns nearest within slop_ns, else None.
    """
    # simple linear search; for large sets, use bisect on a separate list of ns.
    best = None; best_dt = None
    for ts, mp in ordered_js:
        dt = abs(ts - t_ns)
        if best is None or dt < best_dt:
            best = (ts, mp); best_dt = dt
    if best is None or best_dt > slop_ns: return None
    return best

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="rosbag2 directory (MCAP or sqlite3)")
    ap.add_argument("--out_dir", required=True, help="output dir for TFRecord shards")
    ap.add_argument("--base_frame", default="base_link")
    ap.add_argument("--eef_frame", default="tool_link")
    ap.add_argument("--finger_joints", nargs="+", default=["finger_1_joint","finger_2_joint","finger_3_joint"])
    ap.add_argument("--finger_open", nargs="+", type=float, default=[0.0,0.0,0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0,1.0,1.0])
    ap.add_argument("--slop_ms", type=float, default=50.0, help="max time diff for nearest JointState")
    ap.add_argument("--shard_size", type=int, default=1000, help="examples per shard")
    ap.add_argument("--prefer_jpeg", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    slop_ns = int(args.slop_ms * 1e6)

    # Reader
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Prepare TF buffer (we will feed /tf_static and /tf)
    tfbuf = Buffer(cache_time=None)  # infinite cache by default

    # Buffers
    joint_buffer = deque(maxlen=10000)  # (ns, {name:pos})
    last_cam_info: Optional[CameraInfo] = None

    # First pass: ingest everything and emit examples on the fly (images drive it)
    def read_msg(topic_type, data, t):
        return deserialize_message(data, eval(topic_type.split('/')[-1]))  # sensor_msgs/msg/Image -> Image

    # TFRecord writers
    shard_idx = 0; in_shard = 0
    writer = tf.io.TFRecordWriter(os.path.join(args.out_dir, f"data-{shard_idx:05d}.tfrecord"))

    try:
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            rtype = type_map.get(topic, "")
            # --- Camera info ---
            if rtype.endswith("sensor_msgs/msg/CameraInfo"):
                msg = deserialize_message(data, CameraInfo)
                if topic.endswith("/camera/color/camera_info"):
                    last_cam_info = msg
            # --- Joint states ---
            elif rtype.endswith("sensor_msgs/msg/JointState"):
                js = deserialize_message(data, JointState)
                pos_map = {n:p for n,p in zip(js.name, js.position)}
                joint_buffer.append((ns(js.header.stamp), pos_map))
            # --- TF static/dynamic ---
            elif rtype.endswith("tf2_msgs/msg/TFMessage"):
                tfm = deserialize_message(data, TFMessage)
                is_static = (topic == "/tf_static")
                for ts in tfm.transforms:
                    add_tf_to_buffer(tfbuf, ts, is_static=is_static)
            # --- Image: drive example creation ---
            elif rtype.endswith("sensor_msgs/msg/Image"):
                if not topic.endswith("/camera/color/image_raw"):
                    continue
                img = deserialize_message(data, Image)
                img_ns = ns(img.header.stamp)

                # find nearest JointState within slop
                nearest = nearest_joint(joint_buffer, img_ns, slop_ns)
                if nearest is None:  # skip frame
                    continue
                js_ns, pos_map = nearest

                # TF lookup at image time
                try:
                    # tf2_py Buffer expects builtin_interfaces/Time
                    tmsg = TimeMsg(sec=int(img.header.stamp.sec), nanosec=int(img.header.stamp.nanosec))
                    tf = tfbuf.lookup_transform(args.base_frame, args.eef_frame, tmsg)
                except Exception:
                    continue

                txyz = tf.transform.translation; q = tf.transform.rotation
                r,p,y = euler_from_quaternion([q.x,q.y,q.z,q.w])
                g = gripper_from_joints(pos_map, args.finger_joints, args.finger_open, args.finger_closed)
                state = np.array([txyz.x, txyz.y, txyz.z, r, p, y, 0.0, g], dtype=np.float32)

                # Encode image
                img_bytes = encode_image_bytes(img, prefer_jpeg=args.prefer_jpeg)

                # Write example
                ex = make_example(img_bytes, state, img_ns, js_ns, last_cam_info)
                writer.write(ex.SerializeToString())
                in_shard += 1

                if in_shard >= args.shard_size:
                    writer.close()
                    shard_idx += 1; in_shard = 0
                    writer = tf.io.TFRecordWriter(os.path.join(args.out_dir, f"data-{shard_idx:05d}.tfrecord"))
            # ignore other topics
    finally:
        writer.close()

    print(f"Wrote {shard_idx+1} shard(s) to {args.out_dir}")

if __name__ == "__main__":
    main()