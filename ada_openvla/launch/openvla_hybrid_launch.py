from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
import datetime

def generate_launch_description():
    bag_prefix = datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S")
    storage = LaunchConfiguration('storage')
    image_topic = LaunchConfiguration('image_topic')
    base_frame  = LaunchConfiguration('base_frame')
    eef_frame   = LaunchConfiguration('eef_frame')

    return LaunchDescription([
        DeclareLaunchArgument('storage', default_value='mcap'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('base_frame',  default_value='j2n6s200_link_base'),
        DeclareLaunchArgument('eef_frame',   default_value='forkTip'),

        # OpenVLA OFT capture node (online obs)
        Node(
            package='ada_openvla',
            executable='openvla_oft_online',
            name='openvla_oft_capture',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'base_frame': base_frame,
                'eef_frame': eef_frame,
                'max_joint_age_s': 0.1,
                'finger_joints': ['j2n6s200_joint_finger_1','j2n6s200_joint_finger_2'],
                'finger_open':   [0.0,0.0],
                'finger_closed': [1.46977,1.46977],
            }],
        ),

        # rosbag2 record (raw)
        ExecuteProcess(
            cmd=[
                'ros2','bag','record',
                '--storage', storage,
                '-o', bag_prefix,
                '/camera/color/image_raw',
                '/camera/color/camera_info',
                '/joint_states',
                '/tf','/tf_static',
            ],
            output='screen'
        )
    ])
