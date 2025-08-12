#ifndef VIPS_DRIVER_HPP_
#define VIPS_DRIVER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "asio.hpp"

namespace vips_driver
{

    // VIPS Protocol Constants
    static constexpr uint8_t VIPS_HEADER_1 = 0x24;
    static constexpr uint8_t VIPS_HEADER_2 = 0xD9;
    static constexpr uint16_t VIPS_CRC_POLYNOMIAL = 0x1021;

    // Options mask bit definitions
    static constexpr uint32_t OUTPUT_GLOBAL_POSITION = 0x01 << 0;
    static constexpr uint32_t OUTPUT_STATUS = 0x01 << 1;
    static constexpr uint32_t OUTPUT_ORIENTATION = 0x01 << 2;
    static constexpr uint32_t OUTPUT_VELOCITY = 0x01 << 3;
    static constexpr uint32_t OUTPUT_VERT_VELOCITY = 0x01 << 4;
    static constexpr uint32_t OUTPUT_UNCERTAINTY = 0x01 << 5;
    static constexpr uint32_t OUTPUT_ACCURACY = 0x01 << 6;
    static constexpr uint32_t OUTPUT_RAW_VIPS = 0x01 << 7;
    static constexpr uint32_t OUTPUT_RAW_IMU = 0x01 << 8;
    static constexpr uint32_t OUTPUT_UNIT_ID = 0x01 << 9;
    static constexpr uint32_t OUTPUT_FIZ_DATA = 0x01 << 10;
    static constexpr uint32_t OUTPUT_ORIGIN = 0x01 << 11;
    static constexpr uint32_t OUTPUT_BEACON_USED = 0x01 << 12;
    static constexpr uint32_t OUTPUT_VCU_STATUS = 0x01 << 13;
    static constexpr uint32_t OUTPUT_QUATERNIONS = 0x01 << 14;
    static constexpr uint32_t OUTPUT_FIZ_EXTENDED = 0x01 << 15;

    // Define a structure to hold the parsed VIPS data
    struct VipsData
    {
        // Basic packet info
        uint32_t time_ms;
        uint32_t mask_flags;
        bool valid;

        // Position (either global lat/lon or local x/y/z meters)
        bool is_global_position;
        union
        {
            struct
            {
                double latitude;  // degrees
                double longitude; // degrees
                float altitude;   // meters
            } global;
            struct
            {
                double x; // meters
                double y; // meters
                float z;  // meters
            } local;
        } position;

        // Status data
        bool has_status;
        uint8_t beacons;
        uint8_t solution_type;
        uint16_t kf_status;

        // Orientation
        bool has_orientation;
        float roll;  // degrees
        float pitch; // degrees
        float yaw;   // degrees

        // Velocity
        bool has_velocity;
        union
        {
            struct
            {
                float speed_kmh; // km/h
                float heading;   // degrees
            } global;
            struct
            {
                float vx; // m/s
                float vy; // m/s
            } local;
        } velocity;

        // Vertical velocity
        bool has_vertical_velocity;
        float vertical_velocity; // m/s

        // Uncertainty data
        bool has_uncertainty;
        float pos_uncertainty[3];    // x, y, z std dev in meters
        float orient_uncertainty[3]; // roll, pitch, yaw std dev in degrees
        float vel_uncertainty[3];    // vx, vy, vz std dev in m/s

        // Accuracy data
        bool has_accuracy;
        float pos_residual; // meters
        uint8_t reliability_flags;
        float vel_residual; // m/s
        uint8_t rover_id;

        // Quaternion orientation (alternative to Euler angles)
        bool has_quaternions;
        float quat_x, quat_i, quat_j, quat_k;
    };

    class VipsDriverNode : public rclcpp::Node
    {
    public:
        explicit VipsDriverNode(const rclcpp::NodeOptions &options);
        ~VipsDriverNode();

    private:
        // Methods
        void declare_parameters();
        void setup_serial_port();
        void read_data_timer_callback();
        bool find_and_parse_vips_packet(VipsData &data);
        bool parse_vips_packet(const std::vector<uint8_t> &buffer, size_t offset, VipsData &data);
        uint16_t calculate_crc(const std::vector<uint8_t> &data, size_t start, size_t length);
        void publish_odometry(const VipsData &data);
        void publish_imu(const VipsData &data);

        // Serial port communication
        asio::io_context io_context_;
        asio::serial_port serial_port_;
        std::string port_name_;
        std::vector<uint8_t> read_buffer_;
        std::vector<uint8_t> packet_buffer_; // Accumulates data for packet parsing

        // ROS 2 Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        // ROS 2 Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Frame IDs
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string imu_frame_id_;
    };

} // namespace vips_driver

#endif // VIPS_DRIVER_HPP_
