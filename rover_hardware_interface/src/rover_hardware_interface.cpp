#include "rover_hardware_interface/rover_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <linux/can.h>
#include <cstring>

namespace rover_hardware_interface
{

  hardware_interface::CallbackReturn RoverSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get parameters from URDF
    can_interface_name_ = "can0"; // default
    wheel_radius_ = 0.15;         // default from URDF

    if (info.hardware_parameters.find("can_interface_name") != info.hardware_parameters.end())
    {
      can_interface_name_ = info.hardware_parameters.at("can_interface_name");
    }

    if (info.hardware_parameters.find("wheel_radius") != info.hardware_parameters.end())
    {
      wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));
    }

    // Initialize CAN socket
    can_socket_ = std::make_unique<SocketCAN>();

    // Initialize all state and command variables to a default value
    hw_front_left_wheel_pos_ = 0.0;
    hw_front_left_wheel_vel_ = 0.0;
    hw_front_right_wheel_pos_ = 0.0;
    hw_front_right_wheel_vel_ = 0.0;
    hw_rear_left_wheel_pos_ = 0.0;
    hw_rear_left_wheel_vel_ = 0.0;
    hw_rear_right_wheel_pos_ = 0.0;
    hw_rear_right_wheel_vel_ = 0.0;
    hw_front_left_steer_pos_ = 0.0;
    hw_front_right_steer_pos_ = 0.0;

    // FIXED: Separate command variables for left and right rear wheels
    hw_rear_left_wheel_velocity_cmd_ = 0.0;
    hw_rear_right_wheel_velocity_cmd_ = 0.0;
    hw_front_left_steer_cmd_ = 0.0;
    hw_front_right_steer_cmd_ = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("RoverSystemHardware"), "Hardware Interface Initialized Successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RoverSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // For each joint, we export a state interface for each available state (e.g., position, velocity).
    // The name of the interface must match the joint name and interface name in the URDF.
    // The third argument is a pointer to the member variable that stores the state.
    state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_front_left_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_front_left_wheel_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_front_right_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_front_right_wheel_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("rear_left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rear_left_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rear_left_wheel_vel_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("rear_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_rear_right_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rear_right_wheel_vel_));

    state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_steer_joint", hardware_interface::HW_IF_POSITION, &hw_front_left_steer_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_steer_joint", hardware_interface::HW_IF_POSITION, &hw_front_right_steer_pos_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RoverSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // FIXED: Separate command interfaces for left and right rear wheels
    // This allows the ackermann controller to command different speeds for differential steering
    command_interfaces.emplace_back(hardware_interface::CommandInterface("rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rear_left_wheel_velocity_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_rear_right_wheel_velocity_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface("front_left_steer_joint", hardware_interface::HW_IF_POSITION, &hw_front_left_steer_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface("front_right_steer_joint", hardware_interface::HW_IF_POSITION, &hw_front_right_steer_cmd_));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RoverSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoverSystemHardware"), "Activating hardware interface...");

    // Open CAN socket
    if (!can_socket_->open(can_interface_name_))
    {
      RCLCPP_ERROR(rclcpp::get_logger("RoverSystemHardware"),
                   "Failed to open CAN interface: %s", can_interface_name_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RoverSystemHardware"),
                "Hardware interface activated successfully on %s.", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoverSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoverSystemHardware"), "Deactivating hardware interface...");

    // Close CAN socket
    if (can_socket_ && can_socket_->is_open())
    {
      can_socket_->close();
    }

    RCLCPP_INFO(rclcpp::get_logger("RoverSystemHardware"), "Hardware interface deactivated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RoverSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // Read all available CAN frames with bounded loop for real-time safety
    can_frame frame;
    int max_frames_per_cycle = 10; // Limit to prevent control loop delays
    int frame_count = 0;
    
    while (can_socket_->read(frame) && frame_count < max_frames_per_cycle)
    {
      frame_count++;
      
      switch (frame.can_id)
      {
      case 0x210: // WHEEL_FRONT_LEFT_SPEED (528)
      {
        double rpm, velocity_kmh;
        if (read_wheel_speed(frame, rpm, velocity_kmh))
        {
          // FIXED: Convert km/h to m/s for ROS standard units
          hw_front_left_wheel_vel_ = velocity_kmh / 3.6;
          hw_front_left_wheel_pos_ += hw_front_left_wheel_vel_ * period.seconds();
        }
        break;
      }
      case 0x211: // WHEEL_FRONT_RIGHT_SPEED (529)
      {
        double rpm, velocity_kmh;
        if (read_wheel_speed(frame, rpm, velocity_kmh))
        {
          // FIXED: Convert km/h to m/s for ROS standard units
          hw_front_right_wheel_vel_ = velocity_kmh / 3.6;
          hw_front_right_wheel_pos_ += hw_front_right_wheel_vel_ * period.seconds();
        }
        break;
      }
      case 0x212: // WHEEL_REAR_LEFT_SPEED (530)
      {
        double rpm, velocity_kmh;
        if (read_wheel_speed(frame, rpm, velocity_kmh))
        {
          // FIXED: Convert km/h to m/s for ROS standard units
          hw_rear_left_wheel_vel_ = velocity_kmh / 3.6;
          hw_rear_left_wheel_pos_ += hw_rear_left_wheel_vel_ * period.seconds();
        }
        break;
      }
      case 0x213: // WHEEL_REAR_RIGHT_SPEED (531)
      {
        double rpm, velocity_kmh;
        if (read_wheel_speed(frame, rpm, velocity_kmh))
        {
          // FIXED: Convert km/h to m/s for ROS standard units
          hw_rear_right_wheel_vel_ = velocity_kmh / 3.6;
          hw_rear_right_wheel_pos_ += hw_rear_right_wheel_vel_ * period.seconds();
        }
        break;
      }
      case 0x206: // SERVO_POSITION (518)
      {
        double position_deg;
        if (read_servo_position(frame, position_deg))
        {
          // FIXED: Convert degrees to radians for ROS standard units
          double position_rad = position_deg * M_PI / 180.0;
          
          // Assuming both front wheels steer together for simplicity
          hw_front_left_steer_pos_ = position_rad;
          hw_front_right_steer_pos_ = position_rad;
        }
        break;
      }
      default:
        // Ignore unknown CAN IDs
        break;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RoverSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Send steering command based on the average of left and right front wheel commands
    double average_steer_cmd = (hw_front_left_steer_cmd_ + hw_front_right_steer_cmd_) / 2.0;
    send_steering_command(average_steer_cmd);

    // FIXED: Send separate throttle commands for left and right rear wheels
    // This enables proper differential steering for Ackermann vehicles
    send_throttle_command_left(hw_rear_left_wheel_velocity_cmd_);
    send_throttle_command_right(hw_rear_right_wheel_velocity_cmd_);

    return hardware_interface::return_type::OK;
  }

  // Helper methods for CAN communication
  void RoverSystemHardware::send_steering_command(double angle_rad)
  {
    can_frame frame;
    frame.can_id = 0x100; // STEERING (256)
    frame.can_dlc = 5;    // Mode (1 byte) + angle (4 bytes) = 5 bytes

    // Set steering mode to ANGLE_MODE (1)
    frame.data[0] = 1;

    // Convert angle from radians to degrees and send as int32
    int32_t angle_degrees = static_cast<int32_t>(angle_rad * 180.0 / M_PI);
    
    // FIXED: Proper little-endian packing
    frame.data[1] = angle_degrees & 0xFF;
    frame.data[2] = (angle_degrees >> 8) & 0xFF;
    frame.data[3] = (angle_degrees >> 16) & 0xFF;
    frame.data[4] = (angle_degrees >> 24) & 0xFF;

    can_socket_->write(frame);
  }

  void RoverSystemHardware::send_throttle_command_left(double velocity_ms)
  {
    can_frame frame;
    frame.can_id = 0x101; // THROTTLE_LEFT (257) - You may need different CAN IDs for left/right
    frame.can_dlc = 5;    // FIXED: Mode (1 byte) + pulse width (2 bytes) + padding (2 bytes) = 5 bytes

    // Set throttle mode to PULSE_WIDTH_MODE (0)
    frame.data[0] = 0;

    // Convert velocity to pulse width (1000-2000 ms range)
    double pulse_width = velocity_to_throttle_pulse_width(velocity_ms);
    uint16_t pulse_width_raw = static_cast<uint16_t>(pulse_width / 0.001); // DBC scale factor is 0.001

    // FIXED: Proper little-endian packing
    frame.data[1] = pulse_width_raw & 0xFF;
    frame.data[2] = (pulse_width_raw >> 8) & 0xFF;
    
    // Zero-pad unused bytes
    frame.data[3] = 0;
    frame.data[4] = 0;

    can_socket_->write(frame);
  }

  void RoverSystemHardware::send_throttle_command_right(double velocity_ms)
  {
    can_frame frame;
    frame.can_id = 0x102; // THROTTLE_RIGHT (258) - Different CAN ID for right wheel
    frame.can_dlc = 5;    // FIXED: Mode (1 byte) + pulse width (2 bytes) + padding (2 bytes) = 5 bytes

    // Set throttle mode to PULSE_WIDTH_MODE (0)
    frame.data[0] = 0;

    // Convert velocity to pulse width (1000-2000 ms range)
    double pulse_width = velocity_to_throttle_pulse_width(velocity_ms);
    uint16_t pulse_width_raw = static_cast<uint16_t>(pulse_width / 0.001); // DBC scale factor is 0.001

    // FIXED: Proper little-endian packing
    frame.data[1] = pulse_width_raw & 0xFF;
    frame.data[2] = (pulse_width_raw >> 8) & 0xFF;
    
    // Zero-pad unused bytes
    frame.data[3] = 0;
    frame.data[4] = 0;

    can_socket_->write(frame);
  }

  bool RoverSystemHardware::read_wheel_speed(const can_frame &frame, double &rpm, double &velocity_kmh)
  {
    if (frame.can_dlc != 8)
    {
      return false;
    }

    // FIXED: Proper little-endian unpacking
    // Extract RPM (first 4 bytes) and SPEED (next 4 bytes)
    uint32_t rpm_raw = 
      frame.data[0] | 
      (frame.data[1] << 8) | 
      (frame.data[2] << 16) | 
      (frame.data[3] << 24);
      
    uint32_t speed_raw = 
      frame.data[4] | 
      (frame.data[5] << 8) | 
      (frame.data[6] << 16) | 
      (frame.data[7] << 24);

    // Interpret as floats (assuming IEEE 754 format)
    float rpm_float, speed_float;
    std::memcpy(&rpm_float, &rpm_raw, sizeof(float));
    std::memcpy(&speed_float, &speed_raw, sizeof(float));

    rpm = static_cast<double>(rpm_float);
    velocity_kmh = static_cast<double>(speed_float); // DBC specifies km/h

    return true;
  }

  bool RoverSystemHardware::read_servo_position(const can_frame &frame, double &position_deg)
  {
    if (frame.can_dlc != 4)
    {
      return false;
    }

    // FIXED: Proper little-endian unpacking
    uint32_t position_raw = 
      frame.data[0] | 
      (frame.data[1] << 8) | 
      (frame.data[2] << 16) | 
      (frame.data[3] << 24);

    // Interpret as float (assuming IEEE 754 format)
    float position_float;
    std::memcpy(&position_float, &position_raw, sizeof(float));

    position_deg = static_cast<double>(position_float); // DBC specifies degrees

    return true;
  }

  double RoverSystemHardware::pulse_width_to_angle(uint16_t pulse_width_ms)
  {
    // Convert pulse width (500-2500 ms) to angle (-45 to +45 degrees)
    // Assuming 1500 ms is center (0 degrees)
    double angle_degrees = (static_cast<double>(pulse_width_ms) - 1500.0) * (45.0 / 1000.0);
    return angle_degrees * M_PI / 180.0; // Convert to radians
  }

  uint16_t RoverSystemHardware::angle_to_pulse_width(double angle_rad)
  {
    // Convert angle (radians) to pulse width (500-2500 ms)
    double angle_degrees = angle_rad * 180.0 / M_PI;
    double pulse_width = 1500.0 + (angle_degrees * 1000.0 / 45.0);

    // Clamp to valid range
    pulse_width = std::max(500.0, std::min(2500.0, pulse_width));

    return static_cast<uint16_t>(pulse_width);
  }

  double RoverSystemHardware::rpm_to_velocity(double rpm)
  {
    // Convert RPM to linear velocity (m/s)
    // velocity = (RPM * 2 * pi * wheel_radius) / 60
    return (rpm * 2.0 * M_PI * wheel_radius_) / 60.0;
  }

  double RoverSystemHardware::velocity_to_throttle_pulse_width(double velocity_ms)
  {
    // Convert linear velocity to throttle pulse width
    // This is a simplified mapping - you may need to adjust based on your specific setup
    // Neutral: 1500 ms, Forward: 1500-2000 ms, Reverse: 1000-1500 ms

    if (std::abs(velocity_ms) < 0.01) // Dead zone
    {
      return 1500.0; // Neutral
    }

    // Scale velocity to pulse width
    // Assuming max velocity of ±2 m/s maps to full throttle range
    double max_velocity = 2.0;
    double normalized_velocity = std::max(-1.0, std::min(1.0, velocity_ms / max_velocity));

    if (normalized_velocity > 0)
    {
      // Forward: 1500-2000 ms
      return 1500.0 + (normalized_velocity * 500.0);
    }
    else
    {
      // Reverse: 1000-1500 ms
      return 1500.0 + (normalized_velocity * 500.0);
    }
  }

} // namespace rover_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverSystemHardware,
    hardware_interface::SystemInterface)
