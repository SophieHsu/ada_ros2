# ada_ros2

ROS2 Hardware Interface and Description for the ADA Robot

## Setup

See the [`ada_feeding` top-level README for setup instructions](https://github.com/personalrobotics/ada_feeding/blob/ros2-devel/README.md).

## Running ADA MoveIt

### RVIZ

1. Run `ros2 launch ada_moveit demo.launch.py sim:=mock` command from your ROS2 workspace.
2. See here for a [brief guide to using RVIZ to interact with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

### Real

1. Run `ros2 launch ada_moveit demo.launch.py` command from your ROS2 workspace. If running for feeding specifically (i.e., where the watchdog dying kills the controllers) run `ros2 launch ada_moveit demo_feeding.launch.py`. Make sure the watchdog is running before you launch this node.

### MoveIt Servo

MoveIt Servo allows [real-time arm servoing](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html) in cartesian space by sending twist commands to the end effector.

To use Servo with keyboard teleop:

1. Launch the force-torque sensor:
   1. Sim: `ros2 run ada_feeding dummy_ft_sensor.py`
   2. Real: `ros2 run forque_sensor_hardware forque_sensor_hardware`
2. Launch MoveIt:
   1. Sim:`ros2 launch ada_moveit demo.launch.py sim:=mock use_servo:=true`
   2. Real: `ros2 launch ada_moveit demo.launch.py sim:=real use_servo:=true`
3. Re-tare the F/T sensor: `ros2 service call /wireless_ft/set_bias std_srvs/srv/SetBool "{data: true}"`
4. Enable MoveIt Servo:
   1. Switch Controllers: `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [\"jaco_arm_servo_controller\"], deactivate_controllers: [\"jaco_arm_controller\"], start_controllers: [], stop_controllers: [], strictness: 0, start_asap: false, activate_asap: false, timeout: {sec: 0, nanosec: 0}}"`
   2. Toggle Servo On: `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"`
5. Run the keyboard teleop script:
   - For mock simulation: `ros2 run ada_moveit ada_keyboard_teleop.py --mock`
   - For real robot: `ros2 run ada_moveit ada_keyboard_teleop.py --real`
   - Default (mock): `ros2 run ada_moveit ada_keyboard_teleop.py`
6. Follow the on-screen instructions to teleoperate the robot. Note that although cartesian control avoids obstacles in the planning scene, joint control does not.
7. Toggle Servo Off: `ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger "{}"`

To create your own Servo client:

1. Follow steps 1-4 above.
2. Have your client publish Twist commands to `/servo_node/delta_twist_cmds`. Note the following:
   1. For reliable cartesian control when sending angular velocities on the real robot and `lovelace`, ensure the angular velocity is \<= 0.3 rad/s in magnitude. Greater angular velocities might change the end effector's position in addition to its orientation. We believe this is because of latencies with MoveIt Servo getting the robot's joint states via the joint state publisher.
   2. Be sure to send 0-velocity Twist messages at the end to stop the robot.

## Keyboard Control Modes

The keyboard teleop script supports two different modes depending on your setup:

### Mock Simulation Mode (`--mock`)
- **Use case**: RViz simulation with MoveIt Servo
- **Topic**: `/servo_node/delta_twist_cmds`
- **Features**: 
  - Real-time cartesian control via MoveIt Servo
  - Collision detection and singularity avoidance
  - Joint trajectory output for smooth motion
- **Command**: `ros2 run ada_moveit ada_keyboard_teleop.py --mock`

### Real Robot Mode (`--real`)
- **Use case**: Physical robot with cartesian controller
- **Topic**: `/jaco_arm_cartesian_controller/twist_cmd`
- **Features**:
  - Direct twist control to cartesian controller
  - Hardware-level safety limits
  - Real-time robot response
- **Command**: `ros2 run ada_moveit ada_keyboard_teleop.py --real`

### Custom Topic Mode
- **Use case**: Custom controller setup
- **Command**: `ros2 run ada_moveit ada_keyboard_teleop.py --topic /your/custom/topic`

## Usage

## Camera Calibration

See the [README for the `default` calibration](./ada_moveit/calib/default/README.md) for details about our extrinsics calibration methdology.

## **Complete Setup Guide for OpenVLA Data Collection**

### **Step 1: Build and Source the Workspace**
```bash
<code_block_to_apply_changes_from>
# Build the workspace
cd /home/sophie/colcon_ws
colcon build --packages-select ada_openvla ada_moveit

# Source the workspace
source install/setup.bash
```

### **Step 2: Launch MoveIt (Choose One Option)**

#### **Option A: Simulation Mode (Recommended for Testing)**
```bash
ros2 launch ada_moveit demo.launch.py sim:=mock
```
- **Use case**: RViz simulation, testing OpenVLA setup
- **Benefits**: Safe, no real robot needed, full TF transforms
- **What you'll see**: RViz opens with the ADA robot model

#### **Option B: Real Robot Mode**
```bash
ros2 launch ada_moveit demo.launch.py
```
- **Use case**: Physical robot data collection
- **Requirements**: Real robot connected and powered on
- **Benefits**: Real sensor data, actual robot movements

#### **Option C: Feeding-Specific Mode**
```bash
ros2 launch ada_moveit demo_feeding.launch.py
```
- **Use case**: Feeding-related data collection
- **Requirements**: Watchdog must be running first
- **Benefits**: Optimized for feeding tasks

### **Step 3: Verify MoveIt is Running Properly**

Check that these topics are publishing:
```bash
# Check joint states
ros2 topic list | grep joint
ros2 topic echo /joint_states --once

# Check TF transforms
ros2 topic list | grep tf
ros2 topic echo /tf --once

# Check robot description
ros2 topic list | grep robot_description
```

### **Step 4: Launch OpenVLA for Data Collection**

#### **Option A: Launch OpenVLA with MoveIt (Recommended)**
```bash
# In a new terminal, source the workspace
source /home/sophie/colcon_ws/install/setup.bash

# Launch OpenVLA
ros2 launch ada_openvla openvla_hybrid_launch.py
```

#### **Option B: Launch with Custom Parameters**
```bash
ros2 launch ada_openvla openvla_hybrid_launch.py \
    storage:=mcap \
    image_topic:=/camera/color/image_raw \
    base_frame:=j2n6s200_link_base \
    eef_frame:=forkTip
```

### **Step 5: Verify OpenVLA is Collecting Data**

Check that OpenVLA is receiving the required data:
```bash
# Check if OpenVLA node is running
ros2 node list | grep openvla

# Check OpenVLA topics
ros2 topic list | grep openvla

# Monitor the data collection
ros2 topic echo /joint_states --once
```

### **Step 6: Test Data Collection**

#### **For Simulation Mode:**
1. In RViz, use the interactive markers to move the robot arm
2. Watch the terminal where OpenVLA is running for data collection logs
3. Check that rosbag2 is recording data

#### **For Real Robot Mode:**
1. Use MoveIt's planning interface to send the robot to different positions
2. Monitor the data collection in real-time
3. Ensure the robot is moving safely

### **Step 7: Monitor Data Collection**

OpenVLA will automatically:
- Record camera images from `/camera/color/image_raw`
- Capture joint states from `/joint_states`
- Record TF transforms
- Save everything to a timestamped rosbag2 file

### **Step 8: Stop Data Collection**

```bash
# Stop OpenVLA (Ctrl+C in its terminal)
# Stop MoveIt (Ctrl+C in its terminal)

# Check the recorded data
ls -la *.mcap  # or whatever storage format you used
```

## **Troubleshooting Common Issues**

### **Issue: "No TF transforms found"**
**Solution**: Ensure MoveIt is fully loaded and the robot description is published
```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

### **Issue: "No joint states received"**
**Solution**: Check that MoveIt controllers are running
```bash
ros2 service list | grep controller
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

### **Issue: "Camera topic not found"**
**Solution**: Either connect a real camera or use a mock camera node
```bash
# For testing without camera, you can use a mock image publisher
ros2 run image_transport_tutorials mock_image_publisher
```

## **Complete Launch Sequence Example**

Here's the complete sequence in separate terminals:

**Terminal 1 (MoveIt):**
```bash
cd /home/sophie/colcon_ws
source install/setup.bash
ros2 launch ada_moveit demo.launch.py sim:=mock
```

**Terminal 2 (OpenVLA):**
```bash
cd /home/sophie/colcon_ws
source install/setup.bash
ros2 launch ada_openvla openvla_hybrid_launch.py
```

**Terminal 3 (Monitoring):**
```bash
cd /home/sophie/colcon_ws
source install/setup.bash
# Monitor topics
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
```

## **Expected Output**

When everything is working correctly, you should see:
- MoveIt RViz interface with the ADA robot
- OpenVLA logging data collection messages
- rosbag2 recording data to a timestamped file
- No error messages about missing topics or transforms

This setup gives you a complete pipeline for collecting OpenVLA training data with proper joint states, TF transforms, and camera images!
