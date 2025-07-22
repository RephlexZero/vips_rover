# Rover ROS2 Workspace

This workspace contains the ROS2 packages for controlling an Ackermann steering rover, both in simulation and in real life.

## Requirements

- **Ubuntu 24.04 LTS** (Noble Numbat) - Required for ROS2 Jazzy
- **ROS2 Jazzy** - [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)

## Packages

- **rover_hardware_interface**: Hardware interface and controllers for the rover
- **vips_driver**: VIPS sensor driver

## Setup

```bash
# Clone the workspace
git clone --recursive git@gitlab.com:vips-ros2-rover/rover.git ros2_ws
cd ros2_ws

# Install all dependencies using rosdep (recommended)
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
./scripts/build.sh
```

**Note**: `rosdep` automatically installs all required packages including:
- Gazebo Harmonic (simulation)
- ros2_control packages  
- All ROS2 message packages
- System dependencies

## Quick Start

### Prerequisites

- ROS2 Jazzy installed ([Installation Guide](https://docs.ros.org/en/jazzy/Installation.html))
- For initial setup, see the [Setup](#setup) section above

### Daily Usage

```bash
# Source ROS2 (add to ~/.bashrc for persistence)
source /opt/ros/jazzy/setup.bash

# Update dependencies (run occasionally)
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build and source workspace
./scripts/build.sh
# OR manually: colcon build && source install/setup.bash
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

### Missing Dependencies
```bash
# Update and install missing dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

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
- Ensure all dependencies are installed: `rosdep install --from-paths . --ignore-src -r -y`
- Check that the URDF file is valid: `xacro rover_hardware_interface/urdf/rover.urdf.xacro`
- Verify Gazebo installation: `gz sim --version`

### Real robot issues
- Check CAN bus connection
- Verify hardware interface parameters in the URDF
- Check controller configuration in `rover_controllers.yaml`
