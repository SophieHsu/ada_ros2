import rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion

def sensor_qos(depth=5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

def run_openvla_oft(obs):
    """
    Hook for your openvla-oft/test_start.py pipeline.
    Expect:
      obs["image_primary"]: HxWx3 uint8 RGB
      obs["state"]: (8,) float32  [x,y,z, r,p,y, 0.0, gripper]
    """
    img = obs["image_primary"]; state = obs["state"]
    assert img.dtype == np.uint8 and img.ndim == 3 and img.shape[2] == 3
    assert state.dtype == np.float32 and state.shape == (8,)

class OpenVLAOFTCapture(Node):
    def __init__(self):
        super().__init__('openvla_oft_capture')
        # Frames & topics
        self.base = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.eef  = self.declare_parameter('eef_frame',  'tool_link').get_parameter_value().string_value
        self.image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').get_parameter_value().string_value
        self.max_joint_age_s = float(self.declare_parameter('max_joint_age_s', 0.1).get_parameter_value().double_value)
        # Jaco fingers (+ calibrations)
        self.finger_names = self.declare_parameter('finger_joints',
            ['j2n6s200_joint_finger_1','j2n6s200_joint_finger_2']).get_parameter_value().string_array_value
        self.open_pos   = list(self.declare_parameter('finger_open',  [0.0,0.0]).get_parameter_value().double_array_value)
        self.closed_pos = list(self.declare_parameter('finger_closed',[1.46977,1.46977]).get_parameter_value().double_array_value)

        # TF & subs
        self.tfbuf = Buffer(cache_time=rclpy.time.Duration(seconds=10.0))
        self.tfl   = TransformListener(self.tfbuf, self, spin_thread=True)
        self.bridge = CvBridge()
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile=sensor_qos())
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, qos_profile=sensor_qos())
        self.latest_joint_state = None  # (ns, {name: pos})

        self.get_logger().info(f"[OFT] image={self.image_topic}, joints=/joint_states, TF {self.base}->{self.eef}")

    @staticmethod
    def _ns(stamp): return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)

    def on_joint_state(self, msg: JointState):
        self.latest_joint_state = (self._ns(msg.header.stamp), {n:p for n,p in zip(msg.name, msg.position)})

    def _gripper_fraction(self, pos_map):
        import numpy as np
        vals=[]
        for i,n in enumerate(self.finger_names):
            if n in pos_map and i < len(self.open_pos) and i < len(self.closed_pos):
                num = pos_map[n]-self.open_pos[i]
                den = max(1e-6, self.closed_pos[i]-self.open_pos[i])
                vals.append(float(np.clip(num/den,0.0,1.0)))
        return float(np.mean(vals)) if vals else 0.0

    def on_image(self, img_msg: Image):
        if self.latest_joint_state is None: return
        try:
            rgb = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}"); return

        img_ns = self._ns(img_msg.header.stamp)
        js_ns, pos_map = self.latest_joint_state
        age_s = abs(img_ns - js_ns)/1e9
        if age_s > self.max_joint_age_s:
            self.get_logger().debug(f"stale joints ({age_s:.3f}s)"); return

        try:
            tf = self.tfbuf.lookup_transform(
                self.base, self.eef,
                rclpy.time.Time.from_msg(img_msg.header.stamp),
                timeout=rclpy.time.Duration(seconds=0.05))
        except TransformException: return

        t = tf.transform.translation; q = tf.transform.rotation
        r,p,y = euler_from_quaternion([q.x,q.y,q.z,q.w])
        g = self._gripper_fraction(pos_map)
        state = np.array([t.x, t.y, t.z, r, p, y, 0.0, g], dtype=np.float32)
        obs = {"image_primary": rgb, "state": state, "timestamps":{"image_ns":img_ns,"joints_ns":js_ns}}
        try: run_openvla_oft(obs)
        except Exception as e: self.get_logger().error(f"OFT failed: {e}")

def main():
    rclpy.init(); node = OpenVLAOFTCapture()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
