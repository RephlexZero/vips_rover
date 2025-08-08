# Rover ROS2 Workspace

A complete ROS2 system for controlling an Ackermann steering rover with seamless switching between simulation and real hardware. Features industry-standard separation of concerns, robust CAN bus communication, and high-precision indoor positioning.

## ✨ Key Features

- **🎯 Seamless Mode Switching**: One command to switch between pure simulation and real hardware
- **🏗️ Clean Architecture**: Industry-standard package separation with no conflicts
- **🚗 Ackermann Control**: Proper differential steering with front wheel steering geometry
- **📡 CAN Bus Integration**: Reliable real-time communication with motor controllers
- **📍 VIPS Positioning**: High-precision indoor localization with sensor fusion
- **🔧 Easy Development**: Pure simulation mode for algorithm development and testing

## System Requirements

- **Ubuntu 24.04 LTS** (Noble Numbat) or **Ubuntu 22.04 LTS** (Jammy)
- **ROS2 Jazzy** (or Humble) - [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Garden/Harmonic** (for simulation)

## 📦 Package Architecture

### Core Packages
- **`rover_description`**: Robot URDF, configurations, and launch files (simulation + hardware)
- **`rover_hardware_interface`**: Real hardware CAN bus interface and ros2_control plugin
- **`vips_driver`**: High-precision indoor positioning system driver

### Clean Separation
- **Simulation**: Pure Gazebo simulation with no hardware dependencies
- **Hardware**: Real CAN bus and VIPS drivers with no simulation dependencies  
- **Shared**: Common robot description works for both modes

## 🚀 Quick Start

### 1. Clone and Build

```bash
# Clone the workspace
git clone --recursive <your-repo-url> ros2_ws
cd ros2_ws

# Build everything
./scripts/build.sh

# Verify installation  
./scripts/validate.sh
```

### 2. Choose Your Mode

#### 🎮 Pure Simulation (Development & Testing)
```bash
./scripts/sim.sh
# OR: ros2 launch rover_description simulation.launch.py
```
**Perfect for**: Algorithm development, controller tuning, path planning testing

#### 🤖 Real Hardware (Field Operations)
```bash
# One-time system setup (first run only)
sudo ./scripts/setup.sh

# Launch hardware system
./scripts/real.sh  
# OR: ros2 launch rover_description hardware.launch.py
```
**Includes**: CAN bus interface, VIPS positioning, sensor fusion with robot_localization EKF

#### 🧭 Autonomous Navigation

**Simulation with SLAM and Nav2:**
```bash
./scripts/nav_sim.sh
# OR: ros2 launch rover_navigation rover_nav_sim.launch.py
```

**Hardware with Sensor Fusion:**
```bash
./scripts/nav_hw.sh
# OR: ros2 launch rover_navigation rover_nav_hw.launch.py
```

**Features**: Click-to-navigate in RViz, automatic path planning, obstacle avoidance, SLAM mapping, recovery behaviors

### 3. Control the Rover

#### Interactive Control
```bash
./scripts/control.sh
```

#### Manual Commands
```bash
# Drive forward at 1 m/s
ros2 topic pub /ackermann_steering_controller/reference_unstamped \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}"

# Turn right while moving (0.5 rad steering angle)
ros2 topic pub /ackermann_steering_controller/reference_unstamped \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.5}}"
```

## 🛠️ Available Scripts

| Command | Description | Mode |
|---------|-------------|------|
| `./scripts/build.sh` | Build all packages | Development |
| `./scripts/validate.sh` | Run validation tests | Development |
| `./scripts/sim.sh` | Launch pure simulation | Simulation |
| `./scripts/real.sh` | Launch real hardware | Hardware |
| `./scripts/nav_sim.sh` | Autonomous navigation (sim) | Navigation |
| `./scripts/nav_hw.sh` | Autonomous navigation (hw) | Navigation |
| `./scripts/control.sh` | Interactive rover control | Both |
| `./scripts/setup.sh` | One-time system setup | Setup |

## 🔧 Hardware Configuration

### Rover Specifications
- **Wheelbase**: 567mm (high-performance RC racing rover scale)
- **Track Width**: 435mm  
- **Wheel Diameter**: 160mm (80mm radius)
- **Front Wheel Camber**: 5° for improved cornering
- **Motor**: D43L86-400 (400KV, 6-pole, 3000W max)
- **Gearing**: 11:1 total (4:1 intermediate × 2.75:1 final)

### Communication Interfaces
- **CAN Bus**: 500kbps bitrate, SocketCAN interface
- **VIPS Serial**: `/dev/ttyUSB0` (configurable)
- **Control Loop**: 20Hz real-time operation

### CAN Protocol Implementation
- **Steering**: 0x100 (angle mode with degrees → radians conversion)
- **Throttle Left**: 0x101 (pulse width mode with m/s → PWM conversion)  
- **Throttle Right**: 0x102 (independent differential control)
- **Wheel Speeds**: 0x210-0x213 (km/h → m/s conversion with proper endianness)
- **Servo Position**: 0x206 (degrees → radians conversion)

## 📊 Monitoring and Diagnostics

### Key Topics

#### Simulation Mode
```bash
ros2 topic echo /joint_states                              # All joint states
ros2 topic echo /ackermann_steering_controller/odometry    # Wheel odometry
ros2 topic list | grep ackermann                          # Controller topics
```

#### Hardware Mode  
```bash
ros2 topic echo /joint_states                              # Joint states from hardware
ros2 topic echo /ackermann_steering_controller/odometry    # Wheel odometry
ros2 topic echo /vips/odometry                             # VIPS positioning
ros2 topic echo /vips/imu                                  # VIPS IMU data
ros2 topic echo /odometry/filtered                         # Fused odometry (EKF)
candump can0                                               # Raw CAN traffic
```

### System Health Checks
```bash
# Overall system status
./scripts/help.sh

# Controller status
ros2 control list_controllers

# Hardware interface status (hardware mode)
ros2 topic hz /joint_states                    # Should be ~20Hz
ip link show can0                              # CAN interface status
```

## 🔧 Technical Implementation Details

### Critical Issues Resolved
1. **✅ Hardware Plugin Mismatch**: Dynamic plugin selection via URDF parameterization
2. **✅ Simulation Time Configuration**: Separate configs for sim vs hardware timing
3. **✅ Differential Drive Control**: Independent left/right wheel command interfaces  
4. **✅ CAN Protocol Implementation**: Proper unit conversions, endianness, and real-time safety
5. **✅ Package Separation**: Clean architecture with no simulation/hardware conflicts

### URDF Parameterization
```xml
<!-- Dynamic configuration based on launch mode -->
<xacro:arg name="hardware_plugin" default="rover_hardware_interface/RoverSystemHardware"/>
<xacro:arg name="controllers_file" default="rover_controllers_sim.yaml"/>
<xacro:arg name="use_sim_time" default="true"/>
```

### Sensor Fusion (Hardware Mode)
- **EKF Filter**: robot_localization package fuses wheel odometry + VIPS positioning
- **Input Sources**: Wheel speeds (trusted for velocity), VIPS position (trusted for absolute position)
- **Output**: `/odometry/filtered` with reduced drift and improved accuracy

## 🧪 Troubleshooting

### Build Issues
```bash
# Install missing dependencies
rosdep update  
rosdep install --from-paths . --ignore-src -r -y

# Clean rebuild
rm -rf build install log
./scripts/build.sh
```

### Simulation Issues
```bash
# Check Gazebo installation
gz sim --version

# Test launch arguments
ros2 launch --show-args rover_description simulation.launch.py

# Alternative for older ROS distributions
ros2 launch rover_description simulation.launch.py world:=empty.world
```

### Hardware Issues
```bash  
# Check CAN interface
ip link show can0                              # Interface exists
ip link show can0 | grep UP                    # Interface is up

# Manual CAN setup if needed
sudo ip link set can0 up type can bitrate 500000

# Check VIPS connection
ls -la /dev/ttyUSB*                            # VIPS device present
sudo dmesg | grep tty                          # USB serial messages

# Check user permissions
groups $USER | grep dialout                    # Should include 'dialout'
```

### Controller Issues
```bash
# List controllers
ros2 control list_controllers

# Restart specific controller
ros2 control set_controller_state ackermann_steering_controller inactive
ros2 control set_controller_state ackermann_steering_controller active

# Check controller manager
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
```

## 🚀 Development Workflow

### Algorithm Development
1. **Start simulation**: `./scripts/sim.sh`
2. **Develop/test algorithms** in pure simulation environment
3. **No hardware dependencies** to worry about
4. **Fast iteration** with immediate feedback

### Hardware Deployment  
1. **Test in simulation first**: Validate algorithms work
2. **Deploy to hardware**: `./scripts/real.sh`  
3. **Same robot model**: Consistent behavior between sim and hardware
4. **Sensor fusion active**: Better odometry than simulation

### Adding New Features
1. **Modify robot description**: Edit `rover_description/urdf/rover.urdf.xacro`
2. **Update controllers**: Modify `rover_description/config/rover_controllers_*.yaml` 
3. **Test simulation**: `./scripts/sim.sh`
4. **Test hardware**: `./scripts/real.sh`
5. **No conflicts**: Clean separation ensures both modes work

---

## 📚 Documentation

- **📋 Implementation Summary**: `IMPLEMENTATION_SUMMARY.md` - Technical details of the architecture
- **🗺️ Original Analysis**: `ROADMAP.md` - Problem analysis and solution roadmap
- **🏗️ System Architecture**: `system_architecture.md` - Overall system design

For immediate help and status: `./scripts/help.sh`

### Real Rover Specifications
- **Wheelbase**: 567mm
- **Track Width**: 435mm  
- **Wheel Diameter**: 160mm
- **Front Wheel Camber**: 5°
- **CAN Bus**: 125kbps bitrate
- **VIPS Serial**: `/dev/ttyUSB0`

### System Integration
- **CAN Interface**: Automatic setup via systemd service
- **VIPS Positioning**: Real-time odometry and IMU data
- **Controllers**: Ackermann steering with proper kinematics
- **No Sudo Required**: After initial setup script

## Manual Control Commands

### Basic Movement
```bash
# Drive forward at 1 m/s
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.0}}"

# Turn right while moving
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.5}}"

# Stop
ros2 topic pub /ackermann_steering_controller/reference_unstamped ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 0.0}}"
```

## Monitoring

### System Topics
```bash
# Joint states
ros2 topic echo /joint_states

# Controller odometry
ros2 topic echo /ackermann_steering_controller/odometry

# VIPS positioning (real hardware only)
ros2 topic echo /vips/odometry
ros2 topic echo /vips/imu

# Controller status
ros2 control list_controllers
```

## Troubleshooting

### Build Issues
```bash
# Install missing dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Clean rebuild
rm -rf build install log
./scripts/build.sh
```

### Hardware Issues
```bash
# Check CAN interface
ip link show can0

# Check VIPS device
ls -la /dev/ttyUSB0

# Verify permissions
groups $USER  # Should include 'dialout'
```

### Controller Issues
```bash
# List available controllers
ros2 control list_controllers

# Check controller manager
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

### System Reset
If you encounter permission issues after setup:
```bash
# Reboot to ensure all services are active
sudo reboot

# Or manually restart services
sudo systemctl restart can-setup.service
```

## Development

### File Structure
```
rover_hardware_interface/
├── config/          # Controller configurations
├── launch/          # Launch files
├── urdf/           # Robot description (URDF/Xacro)
└── src/            # Hardware interface code

vips_driver/
├── launch/         # VIPS driver launch files
└── src/            # VIPS communication code

scripts/            # Simplified automation scripts
setup/              # System configuration files
```

### Adding New Features
1. Modify URDF model in `rover_hardware_interface/urdf/`
2. Update controllers in `rover_hardware_interface/config/`
3. Test in simulation first: `./scripts/sim.sh`
4. Test on real hardware: `./scripts/real.sh`

---

For help and status information, run: `./scripts/help.sh`
