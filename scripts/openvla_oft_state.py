# openvla_state_node.py
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState, CompressedImage, Image
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class OpenVLAState(Node):
    def __init__(self):
        super().__init__('openvla_state')
        self.base = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.eef  = self.declare_parameter('eef_frame',  'tool_link').get_parameter_value().string_value
        self.finger_names = self.declare_parameter(
            'finger_joints', ['finger_1_joint', 'finger_2_joint', 'finger_3_joint']
        ).get_parameter_value().string_array_value
        # calibration for normalization
        self.open_pos  = self.declare_parameter('finger_open',  [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.closed_pos= self.declare_parameter('finger_closed',[1.0, 1.0, 1.0]).get_parameter_value().double_array_value

        self.tfbuf = Buffer()
        self.tfl   = TransformListener(self.tfbuf, self)
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/openvla/state', 10)
        self.timer = self.create_timer(1/30.0, self.tick)  # 30 Hz

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_positions[n] = p

    def gripper_fraction(self):
        vals = []
        for i, name in enumerate(self.finger_names):
            if name in self.joint_positions:
                p = self.joint_positions[name]
                # normalize
                num = p - self.open_pos[i]
                den = max(1e-6, self.closed_pos[i] - self.open_pos[i])
                vals.append(np.clip(num / den, 0.0, 1.0))
        return float(np.mean(vals)) if vals else 0.0

    def tick(self):
        try:
            tf = self.tfbuf.lookup_transform(self.base, self.eef, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            r,p,y = euler_from_quaternion([q.x, q.y, q.z, q.w])  # radians
            g = self.gripper_fraction()
            state = np.array([t.x, t.y, t.z, r, p, y, 0.0, g], dtype=np.float32)
            msg = Float32MultiArray(data=state.tolist())
            self.pub.publish(msg)
        except Exception:
            pass

class OpenVLAImageRaw(Node):
    def __init__(self):
        super().__init__('openvla_image_raw')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/openvla/image_rgb8')
        self.declare_parameter('also_publish_jpeg', False)  # optional

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.also_jpeg = self.get_parameter('also_publish_jpeg').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_raw = self.create_publisher(Image, self.output_topic, 10)
        self.pub_jpeg = self.create_publisher(CompressedImage, self.output_topic + '_jpeg', 10) if self.also_jpeg else None

        self.get_logger().info(f"Subscribing: {self.image_topic} -> Publishing raw RGB8: {self.output_topic}")

    def on_image(self, msg: Image):
        # Ensure encoding is RGB8; re-encode if needed
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')  # RealSense publishes RGB8
        # Re-wrap as sensor_msgs/Image (no resize/crop/normalize)
        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='rgb8')
        out.header = msg.header  # preserve timestamp/frame
        self.pub_raw.publish(out)

        # Optional: also publish JPEG bytes (useful if you want compact logs)
        if self.pub_jpeg:
            ok, enc = cv2.imencode('.jpg', cv_img[:, :, ::-1])  # cv2 expects BGR for encoding
            if ok:
                c = CompressedImage()
                c.header = msg.header
                c.format = 'jpeg'
                c.data = enc.tobytes()
                self.pub_jpeg.publish(c)

def main():
    rclpy.init()
    n = OpenVLAState()
    i = OpenVLAImageRaw()
    rclpy.spin(n)
    rclpy.spin(i)
    i.destroy_node()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()