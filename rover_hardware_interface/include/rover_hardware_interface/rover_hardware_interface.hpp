#ifndef ROVER_HARDWARE_INTERFACE_HPP_
#define ROVER_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rover_hardware_interface/socketcan_wrapper.hpp"
#include <linux/can.h>
#include <memory>
#include <cmath>

namespace rover_hardware_interface
{
    class RoverSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RoverSystemHardware)

        // The on_init method is called when the hardware interface is loaded by the controller manager.
        // It's used to parse the URDF and initialize member variables.
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        // This method is used to export the state interfaces that the hardware provides.
        // These are read-only interfaces that report the current state of the robot (e.g., wheel position, velocity).
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // This method is used to export the command interfaces that the hardware accepts.
        // These are writeable interfaces that controllers use to send commands to the hardware.
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // on_activate is called when the hardware interface is activated.
        // This is the place to start your hardware communication (e.g., open the CAN socket).
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        // on_deactivate is called when the hardware interface is deactivated.
        // This is the place to stop your hardware communication.
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // The read method is called in every control loop cycle.
        // It's responsible for reading the latest data from the hardware (e.g., from the CAN bus)
        // and updating the state variables.
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // The write method is called in every control loop cycle.
        // It's responsible for taking the latest commands from the controllers and sending them to the hardware.
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // --- Member variables to store the state of the robot ---
        // These are updated in the read() method.
        // We store position and velocity for all four wheels.
        double hw_front_left_wheel_pos_;
        double hw_front_left_wheel_vel_;
        double hw_front_right_wheel_pos_;
        double hw_front_right_wheel_vel_;
        double hw_rear_left_wheel_pos_;
        double hw_rear_left_wheel_vel_;
        double hw_rear_right_wheel_pos_;
        double hw_rear_right_wheel_vel_;

        // We only need position for the steering joints.
        double hw_front_left_steer_pos_;
        double hw_front_right_steer_pos_;

        // --- Member variables to store the commands for the robot ---
        // These are updated by the controller manager and sent to the hardware in the write() method.
        // FIXED: Separate command variables for left and right rear wheels
        double hw_rear_left_wheel_velocity_cmd_;
        double hw_rear_right_wheel_velocity_cmd_;
        double hw_front_left_steer_cmd_;
        double hw_front_right_steer_cmd_;

        // CAN communication
        std::unique_ptr<SocketCAN> can_socket_;
        std::string can_interface_name_;
        double wheel_radius_;

        // Helper methods for CAN message parsing and construction
        void send_steering_command(double angle_rad);
        void send_throttle_command_left(double velocity_ms);
        void send_throttle_command_right(double velocity_ms);
        bool read_wheel_speed(const can_frame &frame, double &rpm, double &velocity_kmh);
        bool read_servo_position(const can_frame &frame, double &position_deg);
        double pulse_width_to_angle(uint16_t pulse_width_ms);
        uint16_t angle_to_pulse_width(double angle_rad);
        double rpm_to_velocity(double rpm);
        double velocity_to_throttle_pulse_width(double velocity_ms);
    };
} // namespace rover_hardware_interface

#endif // ROVER_HARDWARE_INTERFACE_HPP_
