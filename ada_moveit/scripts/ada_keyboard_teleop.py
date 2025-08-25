#!/usr/bin/env python3
# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

# Adapted from:
# https://github.com/turtlebot/turtlebot/blob/melodic/turtlebot_teleop/scripts/turtlebot_teleop_key
"""
This module contains a ROS2 node to allow the user to teleoperate the ADA arm
using the keyboard. Specifically, this node allows users to send linear cartesian
velocities in the base frame, angular cartesian velocities in the end-effector
frame, or joint velocities to the robot via MoveIt Servo.

Usage:
    ros2 run ada_moveit ada_keyboard_teleop.py --mock    # For mock simulation
    ros2 run ada_moveit ada_keyboard_teleop.py --real    # For real robot
"""

# Standard imports
import termios
import tty
import select
import sys
import argparse

# Third-party imports
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.time import Time
from tf2_geometry_msgs import Vector3Stamped  # pylint: disable=unused-import
import tf2_py as tf2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

INSTRUCTION_MSG = """
Control the ADA arm!
---------------------------
Cartesian control (linear):
  w/s: forward/backwards
  a/d: left/right
  q/e: up/down
  f: toggle between base frame (default) and end effector frame

Cartesian control (angular):
  i/k: +pitch/-pitch
  j/l: +yaw/-yaw
  u/o: +roll/-roll

Joint control:
  1-6: joint 1-6
  r: reverse the direction of joint movement

CTRL-C to quit
"""
BASE_FRAME = "j2n6s200_link_base"
EE_FRAME = "forkTip"
LINEAR_VEL_CMD = 0.1  # m/s
ANGULAR_VEL_CMD = 0.3  # rad/s
JOINT_VEL_CMD = 0.5  # rad/s


def get_key(settings):
    """
    Read a key from stdin without writing it to terminal.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


cartesian_control_linear_bindings = {
    "w": (0.0, -1.0, 0.0),  # forward
    "s": (0.0, 1.0, 0.0),  # backwards
    "a": (1.0, 0.0, 0.0),  # left
    "d": (-1.0, 0.0, 0.0),  # right
    "q": (0.0, 0.0, 1.0),  # up
    "e": (0.0, 0.0, -1.0),  # down
}
toggle_linear_frame_key = "f"  # pylint: disable=invalid-name
cartesian_control_angular_bindings = {
    "i": (1.0, 0.0, 0.0),  # +pitch
    "k": (-1.0, 0.0, 0.0),  # -pitch
    "j": (0.0, 1.0, 0.0),  # +yaw
    "l": (0.0, -1.0, 0.0),  # -yaw
    "u": (0.0, 0.0, 1.0),  # +roll
    "o": (0.0, 0.0, -1.0),  # -roll
}
joint_control_bindings = {
    "1": "j2n6s200_joint_1",
    "2": "j2n6s200_joint_2",
    "3": "j2n6s200_joint_3",
    "4": "j2n6s200_joint_4",
    "5": "j2n6s200_joint_5",
    "6": "j2n6s200_joint_6",
}
reverse_joint_direction_key = "r"  # pylint: disable=invalid-name


def parse_arguments():
    """Parse command line arguments to determine robot mode."""
    parser = argparse.ArgumentParser(description='ADA Keyboard Teleop')
    parser.add_argument('--mock', action='store_true', 
                       help='Use mock simulation mode (servo control)')
    parser.add_argument('--real', action='store_true', 
                       help='Use real robot mode (cartesian controller)')
    parser.add_argument('--topic', type=str,
                       help='Custom cartesian command topic (overrides mode selection)')
    
    args = parser.parse_args()
    
    # Determine the cartesian command topic
    if args.topic:
        # Custom topic specified
        cartesian_topic = args.topic
        mode = "custom"
    elif args.real:
        # Real robot mode
        cartesian_topic = "/jaco_arm_cartesian_controller/twist_cmd"
        mode = "real"
    elif args.mock:
        # Mock simulation mode
        cartesian_topic = "/servo_node/delta_twist_cmds"
        mode = "mock"
    else:
        # Default to mock mode
        cartesian_topic = "/servo_node/delta_twist_cmds"
        mode = "mock"
    
    return cartesian_topic, mode


def main(args=None):
    """Main function for the keyboard teleop node."""
    # Parse command line arguments
    cartesian_topic, mode = parse_arguments()
    
    # Initialize the ROS context
    rclpy.init(args=args)
    node = rclpy.create_node("ada_keyboard_teleop")
    
    # Print mode information
    print(f"🎮 ADA Keyboard Teleop - Mode: {mode.upper()}")
    print(f"📡 Cartesian topic: {cartesian_topic}")
    print("⌨️  Use WASD keys for linear motion, QE for angular motion, 1-6 for joint control")
    print("🔄 Press 'r' to reverse joint direction, 'c' to clear commands, 'q' to quit")
    print()
    
    twist_pub = node.create_publisher(
        # TwistStamped, "/servo_node/delta_twist_cmds", 1
        TwistStamped, cartesian_topic, 1
    )
    joint_pub = node.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 1)

    settings = termios.tcgetattr(sys.stdin)

    # Initialize the tf2 buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # pylint: disable=unused-variable

    # Create the cartesian control messages
    # The linear velocity is always in the base frame
    linear_msg = Vector3Stamped()
    linear_msg.header.stamp = Time().to_msg()  # use latest time
    linear_msg.header.frame_id = BASE_FRAME
    # The angular velocity is always in the end effector frame
    angular_msg = Vector3Stamped()
    angular_msg.header.stamp = Time().to_msg()  # use latest time
    angular_msg.header.frame_id = EE_FRAME
    # The final message should be either in the base or end effector frame.
    # It should match the `robot_link_command_frame`` servo param.
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = BASE_FRAME

    # Create the joint control message
    joint_msg = JointJog()
    joint_msg.header.frame_id = BASE_FRAME

    prev_key = ""
    joint_direction = 1.0

    try:
        node.get_logger().info(INSTRUCTION_MSG)
        while 1:
            rclpy.spin_once(node, timeout_sec=0)

            publish_joint_msg = False

            key = get_key(settings)
            if key in cartesian_control_linear_bindings:
                # Due to keyboard delay before repeat, when the user holds down a
                # key we will read it as the key, followed by some number of empty
                # readings, followed by the key consecutively. To account for this,
                # we require two consecutive readings of the same key before
                # publishing the velcoity commands.
                if prev_key == key:
                    x, y, z = cartesian_control_linear_bindings[key]
                    linear_msg.vector.x = x * LINEAR_VEL_CMD
                    linear_msg.vector.y = y * LINEAR_VEL_CMD
                    linear_msg.vector.z = z * LINEAR_VEL_CMD

                    # Transform the linear message to the overall twist message frame
                    twist_msg.twist.linear = linear_msg.vector
                    if linear_msg.header.frame_id != twist_msg.header.frame_id:
                        try:
                            linear_transformed = tf_buffer.transform(
                                linear_msg, twist_msg.header.frame_id
                            )
                            twist_msg.twist.linear = linear_transformed.vector
                        except tf2.ExtrapolationException as exc:
                            node.get_logger().warning(
                                f"Transform from {linear_msg.header.frame_id} to "
                                f"{twist_msg.header.frame_id} failed: {type(exc)}: {exc}\n"
                                f"Interpreting the linear velocity in {twist_msg.header.frame_id} "
                                "without transforming."
                            )
            elif key in cartesian_control_angular_bindings:
                if prev_key == key:
                    x, y, z = cartesian_control_angular_bindings[key]
                    angular_msg.vector.x = x * ANGULAR_VEL_CMD
                    angular_msg.vector.y = y * ANGULAR_VEL_CMD
                    angular_msg.vector.z = z * ANGULAR_VEL_CMD

                    # Transform the angular message to the overall twist message frame
                    twist_msg.twist.angular = angular_msg.vector
                    if angular_msg.header.frame_id != twist_msg.header.frame_id:
                        try:
                            angular_transformed = tf_buffer.transform(
                                angular_msg, twist_msg.header.frame_id
                            )
                            twist_msg.twist.angular = angular_transformed.vector
                        except tf2.ExtrapolationException as exc:
                            node.get_logger().warning(
                                f"Transform from {angular_msg.header.frame_id} to "
                                f"{twist_msg.header.frame_id} failed: {type(exc)}: {exc}\n"
                                f"Interpreting the angular velocity in {twist_msg.header.frame_id}"
                                " without transforming."
                            )
            elif key in joint_control_bindings:
                if prev_key == key:
                    joint_msg.joint_names = [joint_control_bindings[key]]
                    joint_msg.velocities = [JOINT_VEL_CMD * joint_direction]
                    publish_joint_msg = True
            elif key == reverse_joint_direction_key:
                joint_direction *= -1.0
            elif key == toggle_linear_frame_key:
                if linear_msg.header.frame_id == BASE_FRAME:
                    linear_msg.header.frame_id = EE_FRAME
                else:
                    linear_msg.header.frame_id = BASE_FRAME
            else:
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.linear.z = 0.0
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 0.0
                twist_msg.twist.angular.z = 0.0

                # Ctrl+C Interrupt
                if key == "\x03":
                    break

            # Publish the message
            if publish_joint_msg:
                joint_msg.header.stamp = node.get_clock().now().to_msg()
                joint_pub.publish(joint_msg)
            else:
                twist_msg.header.stamp = node.get_clock().now().to_msg()
                twist_pub.publish(twist_msg)

            prev_key = key
    except Exception as exc:  # pylint: disable=broad-except
        print(repr(exc))

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # Terminate this node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
