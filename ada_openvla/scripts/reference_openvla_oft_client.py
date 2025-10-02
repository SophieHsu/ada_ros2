#!/usr/bin/env python3
"""
client_stream_vla_to_jtc_refined.py  — dual-cam with "use older wrist or error" policy

- Primary + optional wrist camera subscribers
- Time pairing:
    1) Try nearest wrist within stereo_sync_tol_ms (either side).
    2) Else use the most recent OLDER wrist frame within max_wrist_age_ms.
    3) Else raise RuntimeError (skip request this cycle).
- Sends images_b64: [primary, wrist] to the server (or [primary] for single-cam).
"""

import argparse, math, base64, io
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import requests
from PIL import Image

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

# --- lightweight cv_bridge replacement ---
import cv2
def imgmsg_to_rgb_np(msg: ImageMsg) -> np.ndarray:
    enc = (msg.encoding or "").lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if enc in ("rgb8","bgr8","rgba8","bgra8","mono8","mono16"):
        if enc == "mono16":
            dtype=np.uint16; ch=1
        elif enc in ("rgba8","bgra8"):
            dtype=np.uint8; ch=4
        elif enc in ("rgb8","bgr8","mono8"):
            dtype=np.uint8; ch=3 if enc!="mono8" else 1
    else:
        dtype=np.uint8; ch=3
    bpp = 2 if dtype==np.uint16 else 1
    arr = np.frombuffer(msg.data, dtype=dtype)
    if step == w*ch*bpp:
        arr = arr.reshape((h,w,ch))
    else:
        row_bytes = step//bpp
        arr = arr.reshape((h,row_bytes))[:, :w*ch].reshape((h,w,ch))
    if enc == "bgr8": arr = arr[..., ::-1]
    elif enc == "bgra8": arr = arr[..., :3][:, :, ::-1]
    elif enc == "rgba8": arr = arr[..., :3]
    elif enc == "mono8": arr = np.repeat(arr[:, :, None], 3, axis=2)
    elif enc == "mono16":
        arr8 = (arr >> 8).astype(np.uint8); arr = np.repeat(arr8[:, :, None], 3, axis=2)
    return np.ascontiguousarray(arr, dtype=np.uint8)

def quat_norm(q):
    q = np.asarray(q, dtype=np.float64)
    n = float(np.linalg.norm(q))
    return q if n < 1e-12 else (q / n)

def axis_angle_to_quat(ax: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(ax))
    if angle < 1e-12:
        return np.array([0., 0., 0., 1.], dtype=np.float64)
    axis = (ax / angle).astype(np.float64)
    s = math.sin(0.5 * angle)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(0.5*angle)], np.float64)

def ns_from_stamp(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

class StreamVLAtoJTC(Node):
    def __init__(self, args):
        super().__init__("vla_client_jtc_stream")
        self.args = args

        # TF + state
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self, spin_thread=True)
        self.joint_map = {}

        # Perception buffers (primary)
        self.latest_rgb = None
        self.latest_rgb_stamp = None
        self._latest_rgb_ns = None

        # Wrist buffers (keep a short history for pairing)
        self._wrist_buf: deque = deque(maxlen=self.args.wrist_buffer_size)
        self._wrist_ever_seen = False

        # Server gating
        self.last_fetch_ns: Optional[int] = None
        self.last_exec_end_stamp: Optional[Time] = None

        # Pose cursor (EEF pose in base_frame)
        self.pose_cursor_pos: Optional[np.ndarray] = None
        self.pose_cursor_quat: Optional[np.ndarray] = None
        self._cursor_reseeded = False

        # IK / exec buffers
        self.ik_points: List[dict] = []
        self.grip_vals: List[float] = []
        self.dt_points: List[float] = []
        self.q_prev_list: Optional[List[float]] = None
        self.last_sent_positions: Optional[List[float]] = None
        # Action queue (model outputs)
        self.actions_queue: deque = deque()
        self.chunks_seen = 0
        self.executing = False
        self.pending_ik_future = None
        self.pending_ik_meta = None
        self._pending_gripper_data = None

        # Subs
        self.create_subscription(JointState, self.args.joint_states_topic, self._on_js, 50)
        self.create_subscription(ImageMsg, self.args.image_topic, self._on_image, 10)
        if self.args.wrist_image_topic:
            self.create_subscription(ImageMsg, self.args.wrist_image_topic, self._on_wrist_image, 10)

        # ROS services / actions
        self.ik_cli = self.create_client(GetPositionIK, self.args.compute_ik_srv)
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   f"/{self.args.arm_controller}/follow_joint_trajectory")
        self.hand_ac = ActionClient(self, FollowJointTrajectory,
                                    f"/{self.args.hand_controller}/follow_joint_trajectory") if self.args.hand_controller else None

        # Optional enable topic
        self.pub_enable = None
        if self.args.topic_enable:
            self.pub_enable = self.create_publisher(Bool, self.args.topic_enable, 1)

        # Optional debug publishers (what we sent to server)
        self.pub_sent_image = None
        self.pub_sent_wrist_image = None
        if self.args.pub_debug_image:
            self.pub_sent_image = self.create_publisher(ImageMsg, "~/sent_image", 10)
            if self.args.wrist_image_topic:
                self.pub_sent_wrist_image = self.create_publisher(ImageMsg, "~/sent_wrist_image", 10)

        # Timers
        self.timer_fetch = self.create_timer(1.0 / self.args.vision_rate_hz, self._tick_fetch)
        self.timer_exec  = self.create_timer(0.01, self._tick_execute)  # 100 Hz control loop

        # Bring-up waits + initial reseed
        self.get_logger().info("Streaming VLA→JTC client starting… (waiting for ROS servers)")
        self._wait_ready()
        self._try_initial_reseed()

        if self.pub_enable:
            self.get_logger().info(f"Publishing enable on {self.args.topic_enable}")
            self.pub_enable.publish(Bool(data=True))

        # Initialize last_sent & q_prev from current joints
        self.q_prev_list = self._ordered_current_joints()
        self.last_sent_positions = None if self.q_prev_list is None else list(self.q_prev_list)

        self.get_logger().info("Ready. Entering main loop… (Ctrl-C to stop)")

    # -------------------- callbacks --------------------
    def _on_js(self, msg: JointState):
        self.joint_map = {n: p for n, p in zip(msg.name, msg.position)}

    def _on_image(self, msg: ImageMsg):
        try:
            rgb = imgmsg_to_rgb_np(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed (primary): {e}")
            return
        self.latest_rgb = rgb
        self.latest_rgb_stamp = msg.header.stamp
        self._latest_rgb_ns = ns_from_stamp(self.latest_rgb_stamp)
        if not self._cursor_reseeded:
            self._try_initial_reseed()
        # Cutover: keep only actions from this new primary frame
        if self.actions_queue:
            newq = deque((a,ns) for (a,ns) in self.actions_queue if ns == self._latest_rgb_ns)
            dropped = len(self.actions_queue) - len(newq)
            self.actions_queue = newq
            if dropped:
                self.get_logger().info(f"[queue] Dropped {dropped} old-frame actions due to new primary image.")

    def _on_wrist_image(self, msg: ImageMsg):
        try:
            rgb = imgmsg_to_rgb_np(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed (wrist): {e}")
            return
        ts_ns = ns_from_stamp(msg.header.stamp)
        self._wrist_buf.append((ts_ns, rgb, msg.header.stamp))
        self._wrist_ever_seen = True

    # -------------------- readiness & reseed --------------------
    def _wait_ready(self):
        self.get_logger().info(f"Waiting for IK service at {self.args.compute_ik_srv} …")
        self.ik_cli.wait_for_service()
        self.get_logger().info(f"Waiting for ARM controller /{self.args.arm_controller}/follow_joint_trajectory …")
        self.arm_ac.wait_for_server()
        if self.hand_ac:
            self.get_logger().info(f"Waiting for HAND controller /{self.args.hand_controller}/follow_joint_trajectory …")
            self.hand_ac.wait_for_server()

    def _try_initial_reseed(self):
        try:
            if not self.tfbuf.can_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                            timeout=Duration(seconds=0.5)):
                raise RuntimeError("TF not yet available")
            tf = self.tfbuf.lookup_transform(self.args.base_frame, self.args.eef_frame, Time(),
                                             timeout=Duration(seconds=0.5))
            t = tf.transform.translation; q = tf.transform.rotation
            self.pose_cursor_pos  = np.array([t.x, t.y, t.z], np.float64)
            self.pose_cursor_quat = np.array([q.x, q.y, q.z, q.w], np.float64)
            self._cursor_reseeded = True
            self.get_logger().info(f"[reseed] cursor from TF {self.args.base_frame}->{self.args.eef_frame}: "
                                   f"pos={self.pose_cursor_pos}, quat={self.pose_cursor_quat}")
        except Exception as e:
            self.get_logger().warn(f"[reseed] initial TF lookup failed (will retry on first image): {e}")

    # -------------------- helpers --------------------
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
            return [t.x, t.y, t.z, roll, pitch, yaw, 0.0, g]
        except Exception:
            return [0.0]*8

    def _encode_image_np_to_b64(self, rgb: np.ndarray) -> str:
        img = Image.fromarray(rgb)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    # -------------------- pairing & fetch --------------------
    def _pair_images(self) -> Tuple[List[np.ndarray], List[Time]]:
        """
        Returns ([primary] or [primary,wrist], [primary_stamp,(wrist_stamp)])
        Policy (dual-cam):
          1) Find wrist within stereo_sync_tol_ms of primary (nearest by |Δt|).
          2) Else pick the most recent OLDER wrist with age <= max_wrist_age_ms.
          3) Else raise RuntimeError.
        """
        primary_ok = self.latest_rgb is not None and self.latest_rgb_stamp is not None
        if not primary_ok:
            return [], []
        ns_p = ns_from_stamp(self.latest_rgb_stamp)

        # single-camera
        if self.args.num_images_in_input <= 1 or not self.args.wrist_image_topic:
            return [self.latest_rgb], [self.latest_rgb_stamp]

        if not self._wrist_ever_seen or len(self._wrist_buf) == 0:
            raise RuntimeError("No wrist frames received yet; cannot send dual-cam payload.")

        # 1) nearest within tolerance
        tol_ns = int(self.args.stereo_sync_tol_ms * 1e6)
        best_idx, best_abs_dt = None, None
        for i, (ns_w, _, _) in enumerate(self._wrist_buf):
            dt = abs(ns_p - ns_w)
            if dt <= tol_ns and (best_abs_dt is None or dt < best_abs_dt):
                best_idx, best_abs_dt = i, dt
        if best_idx is not None:
            _, wrist_img, wrist_stamp = self._wrist_buf[best_idx]
            return [self.latest_rgb, wrist_img], [self.latest_rgb_stamp, wrist_stamp]

        # 2) latest older within max age
        max_age_ns = int(self.args.max_wrist_age_ms * 1e6)
        candidate = None  # (ns_w, img, stamp)
        for ns_w, img, stamp in reversed(self._wrist_buf):
            if ns_w <= ns_p and (ns_p - ns_w) <= max_age_ns:
                candidate = (ns_w, img, stamp)
                break
        if candidate is not None:
            age_ms = (ns_p - candidate[0]) / 1e6
            self.get_logger().warn(f"Using older wrist frame: age={age_ms:.1f} ms "
                                   f"(<= {self.args.max_wrist_age_ms:.1f} ms).")
            return [self.latest_rgb, candidate[1]], [self.latest_rgb_stamp, candidate[2]]

        # 3) fail hard
        raise RuntimeError("Cannot pair wrist image: no synced frame and no older wrist within max_wrist_age_ms.")

    def _tick_fetch(self):
        if not self.args.query_while_executing and self.executing:
            return
        if self.latest_rgb_stamp is None:
            return
        if self.args.require_fresh_image_after_exec and self.last_exec_end_stamp is not None:
            if ns_from_stamp(self.latest_rgb_stamp) <= ns_from_stamp(self.last_exec_end_stamp):
                return
        if self.last_fetch_ns is not None:
            if ns_from_stamp(self.latest_rgb_stamp) <= int(self.last_fetch_ns):
                return
        if len(self.actions_queue) >= self.args.min_server_queue:
            return

        if (self.pose_cursor_pos is None) or (self.chunks_seen % max(1, self.args.reseed_every) == 0 and self.chunks_seen > 0):
            try:
                self._try_initial_reseed()
            except Exception as e:
                self.get_logger().warn(f"Pose reseed failed: {e}")

        try:
            pushed = self._query_server()
        except RuntimeError as e:
            self.get_logger().error(f"[pair] {e}")
            pushed = 0

        if pushed > 0:
            self.chunks_seen += 1

    def _query_server(self) -> int:
        imgs, stamps = self._pair_images()
        if len(imgs) == 0:
            return 0

        frame_ns = ns_from_stamp(stamps[0])  # primary timestamp
        payload = {
            "instruction": self.args.instruction,
            "state": self._proprio_state() if self.args.send_proprio else [],
            "return_chunk": bool(self.args.return_chunk)
        }
        payload["images_b64"] = [self._encode_image_np_to_b64(imgs[0])]
        if self.args.num_images_in_input > 1:
            if len(imgs) < 2:
                raise RuntimeError("Dual-cam mode but wrist image missing after pairing.")
            payload["images_b64"].append(self._encode_image_np_to_b64(imgs[1]))

        try:
            r = requests.post(self.args.server_url, json=payload, timeout=self.args.server_timeout_s)
            r.raise_for_status()
        except Exception as e:
            self.get_logger().warn(f"/infer failed: {e}")
            return 0

        data = r.json()
        acts = data.get("actions", [])
        limit = self.args.server_chunk_limit if self.args.server_chunk_limit > 0 else len(acts)
        kept = acts[:limit]
        pushed = 0
        for a in kept:
            if len(a) == 7:
                self.actions_queue.append((np.asarray(a, dtype=np.float64), frame_ns))
                pushed += 1
                while len(self.actions_queue) > self.args.max_queue:
                    self.actions_queue.popleft()
        self.last_fetch_ns = frame_ns

        # Debug republish: what we actually sent (primary + wrist, if any)
        if self.pub_sent_image and len(imgs) >= 1:
            rgb = imgs[0]
            msg = ImageMsg()
            msg.height, msg.width = rgb.shape[:2]
            msg.encoding = "rgb8"
            msg.step = msg.width * 3
            msg.data = rgb.tobytes()
            msg.header = Header(stamp=stamps[0], frame_id=self.args.camera_frame)
            self.pub_sent_image.publish(msg)
        if self.pub_sent_wrist_image and len(imgs) >= 2:
            rgb = imgs[1]
            msg = ImageMsg()
            msg.height, msg.width = rgb.shape[:2]
            msg.encoding = "rgb8"
            msg.step = msg.width * 3
            msg.data = rgb.tobytes()
            msg.header = Header(stamp=stamps[1], frame_id=self.args.wrist_camera_frame or self.args.camera_frame)
            self.pub_sent_wrist_image.publish(msg)

        self.get_logger().info(f"[fetch] received {len(acts)} actions; kept={pushed}; queue={len(self.actions_queue)}")
        return pushed

    # -------------------- IK helpers & execute loop --------------------
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

    def _ordered_current_joints(self):
        vals, missing = [], []
        for n in self.args.arm_joints:
            if n in self.joint_map:
                vals.append(float(self.joint_map[n]))
            else:
                missing.append(n)
        if missing:
            self.get_logger().warn(f"Missing joints in /joint_states: {missing} (skipping IK)")
            return None
        return vals

    def _ordered_seed_from_map(self):
        # prefer last good IK solution
        if self.q_prev_list and len(self.q_prev_list) == len(self.args.arm_joints):
            return list(self.q_prev_list)

        # else try full current joints
        curr = self._ordered_current_joints()
        if curr is not None:
            return curr

        # else, no safe seed → tell caller to skip
        raise RuntimeError("No complete seed available")

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

    def _tick_execute(self):
        if not self.joint_map or self.pose_cursor_pos is None:
            return

        # Consume any completed IK
        if self.pending_ik_future is not None:
            if self.pending_ik_future.done():
                res = self.pending_ik_future.result()
                self.pending_ik_future = None
                meta = self.pending_ik_meta or {}
                self.pending_ik_meta = None

                if not res or res.error_code.val != res.error_code.SUCCESS:
                    code = res.error_code.val if res else None
                    self.get_logger().warn(f"IK failed (async) - code: {code}")
                else:
                    names = res.solution.joint_state.name
                    posns = res.solution.joint_state.position
                    q_sol_map = {n: p for n, p in zip(names, posns)}
                    dpos = meta.get('dpos', np.zeros(3))
                    drot = meta.get('drot', np.zeros(3))
                    g = float(meta.get('g', 0.0))

                    q_curr_list = [q_sol_map[n] for n in self.args.arm_joints]
                    dt_cart = self._cartesian_dt(dpos, drot)
                    dt_joint = self._joint_dt(self.q_prev_list, q_curr_list)
                    dt_i = min(max(dt_cart, dt_joint), self.args.max_dt)

                    self.ik_points.append(q_sol_map)
                    self.grip_vals.append(g)
                    self.dt_points.append(dt_i)
                    self.q_prev_list = q_curr_list
                
                    # integrate pose cursor only after successful IK
                    if meta:
                        self.pose_cursor_pos  = meta.get('target_pos',  self.pose_cursor_pos)
                        self.pose_cursor_quat = meta.get('target_quat', self.pose_cursor_quat)


        # Start a new IK if none pending and have actions
        if self.pending_ik_future is None and self.actions_queue:
            latest_ns = ns_from_stamp(self.latest_rgb_stamp) if self.latest_rgb_stamp else None
            while self.actions_queue and latest_ns is not None:
                a, a_ns = self.actions_queue[0]
                if a_ns is None or a_ns < latest_ns:
                    self.actions_queue.popleft()
                else:
                    break
            if not self.actions_queue:
                return

            a, a_ns = self.actions_queue[0]
            dpos = a[:3]; drot = a[3:6]; g = float(a[6])

            # compute target without mutating cursor
            target_pos = self.pose_cursor_pos + dpos
            target_quat = quat_norm(np.array(qmul(self.pose_cursor_quat, axis_angle_to_quat(drot))))

            # launch async IK with a safe seed
            future = self._ik_async(target_pos, target_quat)
            if future is None:
                self.get_logger().warn("Skipped IK this tick (no valid seed).")
                return  # try again next spin

            # Start IK then consume the action; stash target pose for later
            self.actions_queue.popleft()
            self.pending_ik_future = future
            self.pending_ik_meta = {
                'dpos': dpos, 'drot': drot, 'g': g,
                'target_pos': target_pos, 'target_quat': target_quat
            }
            return

        # Send when enough points and not executing
        if len(self.ik_points) >= self.args.chunk_size and not self.executing:
            k = self.args.chunk_size
            q_chunk = self.ik_points[:k]; self.ik_points = self.ik_points[k:]
            g_chunk = self.grip_vals[:k]; self.grip_vals = self.grip_vals[k:]
            dt_chunk = self.dt_points[:k]; self.dt_points = self.dt_points[k:]
            self._send_chunk_async(q_chunk, g_chunk, dt_chunk)

    # -------------------- IK, trajectory, gripper --------------------
    def _ik_async(self, pos: np.ndarray, quat: np.ndarray):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.args.move_group
        req.ik_request.ik_link_name = self.args.eef_frame
        req.ik_request.avoid_collisions = bool(self.args.avoid_collisions)
        req.ik_request.timeout = Duration(seconds=self.args.ik_timeout_s).to_msg()

        if (self.q_prev_list is not None) and (not self.args.disable_ik_constraints):
            req.ik_request.constraints = self._nearest_constraints(self.q_prev_list)
        else:
            req.ik_request.constraints = Constraints()

        ps = PoseStamped()
        ps.header.frame_id = self.args.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = quat.tolist()
        req.ik_request.pose_stamped = ps

        try:
            seed_vec = self._ordered_seed_from_map()
        except RuntimeError as e:
            self.get_logger().warn(str(e))
            return None  # no-op; caller will see pending_ik_future is None

        req.ik_request.robot_state.joint_state.name = list(self.args.arm_joints)
        req.ik_request.robot_state.joint_state.position = seed_vec

        self.get_logger().debug(f"IK target pos={pos}, quat={quat}, seed(first)={seed_vec[:3]}…")
        return self.ik_cli.call_async(req)

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

    def _send_chunk_async(self, q_list: List[dict], g_list: List[float], dt_list: List[float]):
        if len(q_list) == 0:
            return

        jt = JointTrajectory()
        jt.joint_names = self.args.arm_joints

        if self.last_sent_positions is None:
            cur = self._ordered_current_joints()
            if cur is not None:
                self.last_sent_positions = cur
            elif self.q_prev_list:
                self.last_sent_positions = list(self.q_prev_list)
            else:
                self.get_logger().warn("No valid current/previous joints; postponing chunk send")
                return

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

        self.executing = True
        self._pending_gripper_data = {'g_list': g_list, 'dt_list': dt_list, 'tfs0': tfs0}
        self.get_logger().info(f"[ARM] chunk: {len(q_list)} pts, dur≈{t_acc - tfs0:.2f}s (+{tfs0:.2f}s hold)")

        arm_send = self.arm_ac.send_goal_async(goal)
        arm_send.add_done_callback(self._on_arm_goal_response)

    def _on_arm_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().error(f"Arm goal send failed: {e}")
            self.executing = False
            self._pending_gripper_data = None
            return

        if not gh or not gh.accepted:
            self.get_logger().error("Arm trajectory rejected")
            self.executing = False
            self._pending_gripper_data = None
            return

        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_arm_result)

    def _on_arm_result(self, fut):
        self.last_exec_end_stamp = self.get_clock().now().to_msg()
        if not self.hand_ac or not self._pending_gripper_data or len(self._pending_gripper_data.get('g_list', [])) == 0:
            return self._on_chunk_finished()
        try:
            g_list = self._pending_gripper_data['g_list']
            dt_list = self._pending_gripper_data['dt_list']
            tfs0   = self._pending_gripper_data['tfs0']

            gtraj = JointTrajectory()
            gtraj.joint_names = self.args.finger_joints
            gtraj.points.append(self._make_hold_point(self._grip_map(g_list[0]), tfs0))
            t_acc = tfs0
            for g, dtk in zip(g_list, dt_list):
                t_acc += dtk * self.args.time_scale
                gp = JointTrajectoryPoint()
                gp.positions = self._grip_map(g)
                gp.time_from_start = Duration(seconds=t_acc).to_msg()
                gtraj.points.append(gp)

            ggoal = FollowJointTrajectory.Goal(trajectory=gtraj)
            gsend = self.hand_ac.send_goal_async(ggoal)
            gsend.add_done_callback(self._on_gripper_goal_response)
        except Exception as e:
            self.get_logger().warn(f"Gripper send failed: {e}")
            self._on_chunk_finished()

    def _on_gripper_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().warn(f"Gripper goal send failed: {e}")
            return self._on_chunk_finished()
        if not gh or not gh.accepted:
            self.get_logger().warn("Gripper trajectory rejected")
            return self._on_chunk_finished()
        gh.get_result_async().add_done_callback(lambda _: self._on_chunk_finished())

    def _on_chunk_finished(self):
        # Reseed pose cursor after executing a chunk
        try:
            self._try_initial_reseed()
        except Exception as e:
            self.get_logger().warn(f"Pose reseed after chunk failed: {e}")
        self.executing = False
        self._pending_gripper_data = None

# -------------------- main --------------------
def main():
    ap = argparse.ArgumentParser()

    # Server
    ap.add_argument("--server_url", required=True, help="http://SERVER:PORT/infer")
    ap.add_argument("--instruction", required=True)
    ap.add_argument("--server_timeout_s", type=float, default=5.0)
    ap.add_argument("--vision_rate_hz", type=float, default=6.0)
    ap.add_argument("--min_server_queue", type=int, default=1)
    ap.add_argument("--max_queue", type=int, default=16)
    ap.add_argument("--reseed_every", type=int, default=5)
    ap.add_argument("--return_chunk", action="store_true")
    ap.set_defaults(return_chunk=True)
    ap.add_argument("--server_chunk_limit", type=int, default=0)
    ap.add_argument("--query_while_executing", action="store_true")
    ap.set_defaults(query_while_executing=True)
    ap.add_argument("--require_fresh_image_after_exec", action="store_true", default=True)
    ap.add_argument("--send_proprio", action="store_true", default=True)

    # Cameras / topics
    ap.add_argument("--image_topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--wrist_image_topic", default="", help="Optional wrist camera topic; empty to disable")
    ap.add_argument("--num_images_in_input", type=int, default=1, help="1=single, 2=dual-cam")
    ap.add_argument("--stereo_sync_tol_ms", type=float, default=100.0, help="max |Δt| to treat frames as synced")
    ap.add_argument("--max_wrist_age_ms", type=float, default=1000.0, help="fallback: allow older wrist up to this age")
    ap.add_argument("--wrist_buffer_size", type=int, default=60, help="how many wrist frames to buffer for pairing")
    ap.add_argument("--camera_frame", default="camera_color_optical_frame")
    ap.add_argument("--wrist_camera_frame", default="wrist_color_optical_frame")
    ap.add_argument("--pub_debug_image", action="store_true", help="Republish the sent frames on ~sent_image/~sent_wrist_image")
    ap.add_argument("--topic_enable", default="", help="Optional Bool enable topic; empty disables")


    # ROS / frames / controllers
    ap.add_argument("--joint_states_topic", default="/joint_states")
    ap.add_argument("--base_frame", default="j2n6s200_link_base")
    ap.add_argument("--eef_frame",  default="j2n6s200_end_effector")
    ap.add_argument("--move_group", default="jaco_arm")
    ap.add_argument("--arm_controller", default="jaco_arm_servo_controller")
    ap.add_argument("--hand_controller", default="hand_controller")
    ap.add_argument("--compute_ik_srv", default="/compute_ik")

    ap.add_argument("--arm_joints", nargs="+", default=[
        "j2n6s200_joint_1","j2n6s200_joint_2","j2n6s200_joint_3",
        "j2n6s200_joint_4","j2n6s200_joint_5","j2n6s200_joint_6"
    ])
    ap.add_argument("--finger_joints", nargs="+", default=[
        "j2n6s200_joint_finger_1","j2n6s200_joint_finger_2"
    ])
    ap.add_argument("--finger_open",   nargs="+", type=float, default=[0.0, 0.0])
    ap.add_argument("--finger_closed", nargs="+", type=float, default=[1.0, 1.0])

    # IK & timing
    ap.add_argument("--chunk_size", type=int, default=4)
    ap.add_argument("--continuity_hold_s", type=float, default=0.01)
    ap.add_argument("--time_scale", type=float, default=1.0)
    ap.add_argument("--v_lin_max", type=float, default=0.05)
    ap.add_argument("--v_ang_max", type=float, default=0.5)
    ap.add_argument("--joint_vel_max", type=float, default=0.8)
    ap.add_argument("--min_dt", type=float, default=0.08)
    ap.add_argument("--max_dt", type=float, default=0.6)

    ap.add_argument("--ik_timeout_s", type=float, default=0.5)
    ap.add_argument("--constraint_tol_rad", type=float, default=10.0)
    ap.add_argument("--disable_ik_constraints", action="store_true")
    ap.set_defaults(disable_ik_constraints=True)
    ap.add_argument("--avoid_collisions", action="store_true")
    ap.set_defaults(avoid_collisions=True)

    args = ap.parse_args()
    if args.wrist_image_topic == "": args.wrist_image_topic = None
    if args.num_images_in_input < 1: args.num_images_in_input = 1
    if not getattr(args, "topic_enable", ""): args.topic_enable = None

    rclpy.init()
    node = None
    try:
        node = StreamVLAtoJTC(args)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
