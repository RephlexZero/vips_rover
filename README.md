# Rover ROS2 Workspace

This workspace contains the ROS2 packages for controlling an Ackermann steering rover, both in simulation and in real life.

## Packages

- **rover_hardware_interface**: Hardware interface and controllers for the rover
- **vips_driver**: VIPS sensor driver

## Quick Start

### Prerequisites

Make sure you have ROS2 installed and this workspace built:

```bash
# Quick build using the provided script
./scripts/build.sh

# OR manually:
cd /home/jakestewart/ros2_ws
colcon build
source install/setup.bash
```

### Running the Simulation

To launch the rover in Gazebo simulation:

```bash
./scripts/run_simulation.sh
```

This will:
- Start Gazebo with an empty world
- Spawn the rover model
- Load all controllers (joint state broadcaster and Ackermann steering controller)
- Set up all necessary ROS topics for control

### Running the Real Robot

To launch the real rover hardware:

```bash
./scripts/run_real_robot.sh
```

This will:
- Start the hardware interface for real CAN bus communication
- Load all controllers
- Set up ROS topics for real robot control

### Controlling the Rover

Once either simulation or real robot is running, you can control it using:

#### Manual Control
```bash
# Drive forward at 1 m/s
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.0}}"

# Turn right while moving
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.5}}"

# Stop
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

#### Automated Movement Scripts

Interactive control menu:
```bash
./scripts/interactive_control.sh
```

Drive in a circle:
```bash
./scripts/drive_circle.sh
```

Drive in a figure-8 pattern:
```bash
./scripts/drive_figure8.sh
```

Drive forward and back:
```bash
./scripts/drive_forward_back.sh
```

Stop the rover immediately:
```bash
./scripts/stop_rover.sh
```

## Monitoring

### View joint states
```bash
ros2 topic echo /joint_states
```

### View controller status
```bash
ros2 control list_controllers
```

### View odometry (if enabled)
```bash
ros2 topic echo /ackermann_steering_controller/odometry
```

## Troubleshooting

### Controllers not loading
```bash
# Check controller manager status
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Restart controllers if needed
ros2 control unload_controller ackermann_steering_controller
ros2 control load_controller ackermann_steering_controller
ros2 control set_controller_state ackermann_steering_controller active
```

### Simulation issues
- Make sure Gazebo is properly installed
- Check that the URDF file is valid: `xacro rover_hardware_interface/urdf/rover.urdf.xacro`

### Real robot issues
- Check CAN bus connection
- Verify hardware interface parameters in the URDF
- Check controller configuration in `rover_controllers.yaml`
