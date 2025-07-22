# Rover ROS2 Workspace

This workspace contains the ROS2 packages for controlling an Ackermann steering rover, both in simulation and with real hardware.

## System Requirements

- **Ubuntu 24.04 LTS** (Noble Numbat)
- **ROS2 Jazzy** - [Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Harmonic** (for simulation)

## Packages

- **rover_hardware_interface**: Hardware interface, controllers, and URDF model
- **vips_driver**: VIPS positioning system driver

## Quick Start

### 1. Initial Setup

```bash
# Clone the workspace
git clone --recursive <your-repo-url> ros2_ws
cd ros2_ws

# Build the workspace
./scripts/build.sh

# One-time system setup (for real hardware)
./scripts/setup.sh
```

The setup script configures:
- CAN interface permissions (no sudo required)
- Automatic CAN setup on boot
- Required system utilities

**Important**: Log out and back in (or reboot) after running setup for the first time.

### 2. Choose Your Mode

#### Simulation Mode
```bash
./scripts/sim.sh
```
Launches Gazebo simulation with the rover model and all controllers.

#### Real Hardware Mode
```bash
./scripts/real.sh
```
Connects to real rover hardware via CAN bus and VIPS positioning system.

### 3. Control the Rover

```bash
./scripts/control.sh
```
Interactive control menu for movement commands.

## Available Scripts

| Script | Purpose |
|--------|---------|
| `./scripts/help.sh` | Show status and available commands |
| `./scripts/build.sh` | Build the ROS2 workspace |
| `./scripts/setup.sh` | One-time system configuration |
| `./scripts/sim.sh` | Start Gazebo simulation |
| `./scripts/real.sh` | Start real hardware system |
| `./scripts/control.sh` | Interactive rover control |

## System Status

Check your system status anytime:
```bash
./scripts/help.sh
```

This shows:
- ✅ Workspace build status
- ✅ User permissions
- ✅ CAN interface status
- ✅ VIPS device detection

## Hardware Configuration

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
