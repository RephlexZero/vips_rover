#include "vips_driver/vips_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace vips_driver
{

    VipsDriverNode::VipsDriverNode(const rclcpp::NodeOptions &options)
        : Node("vips_node", options), serial_port_(io_context_)
    {
        declare_parameters();
        setup_serial_port();

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vips/odometry", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/vips/imu", 10);

        // Create a timer to periodically read from the serial port
        timer_ = this->create_wall_timer(
            10ms, std::bind(&VipsDriverNode::read_data_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "VIPS Driver Node has been started.");
    }

    VipsDriverNode::~VipsDriverNode()
    {
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }
    }

    void VipsDriverNode::declare_parameters()
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("imu_frame_id", "vips_imu_link");
    }

    void VipsDriverNode::setup_serial_port()
    {
        this->get_parameter("serial_port", port_name_);
        this->get_parameter("odom_frame_id", odom_frame_id_);
        this->get_parameter("base_frame_id", base_frame_id_);
        this->get_parameter("imu_frame_id", imu_frame_id_);

        try
        {
            serial_port_.open(port_name_);
            serial_port_.set_option(asio::serial_port_base::baud_rate(115200));
            serial_port_.set_option(asio::serial_port_base::character_size(8));
            serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port: %s", port_name_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s: %s", port_name_.c_str(), e.what());
            rclcpp::shutdown();
        }
    }

    void VipsDriverNode::read_data_timer_callback()
    {
        try
        {
            // Try to read up to 1024 bytes from the serial port
            std::vector<uint8_t> buffer(1024);
            std::error_code ec;
            size_t bytes_read = serial_port_.read_some(asio::buffer(buffer), ec);

            if (!ec && bytes_read > 0)
            {
                // Append new data to packet buffer
                packet_buffer_.insert(packet_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);

                // Try to find and parse complete packets
                VipsData parsed_data;
                while (find_and_parse_vips_packet(parsed_data))
                {
                    if (parsed_data.valid)
                    {
                        publish_odometry(parsed_data);
                        publish_imu(parsed_data);
                    }
                }

                // Prevent buffer from growing too large
                if (packet_buffer_.size() > 4096)
                {
                    packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + 2048);
                }
            }
            else if (ec && ec != asio::error::would_block)
            {
                // Log error if it's not just "no data available"
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Serial port read error: %s", ec.message().c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Exception in read_data_timer_callback: %s", e.what());
        }
    }

    bool VipsDriverNode::find_and_parse_vips_packet(VipsData &data)
    {
        // Look for VIPS header in the buffer
        for (size_t i = 0; i < packet_buffer_.size() - 1; ++i)
        {
            if (packet_buffer_[i] == VIPS_HEADER_1 && packet_buffer_[i + 1] == VIPS_HEADER_2)
            {
                // Found potential header, check if we have enough data for length field
                if (i + 4 > packet_buffer_.size())
                {
                    // Not enough data yet, keep the header for next time
                    if (i > 0)
                    {
                        packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + i);
                    }
                    return false;
                }

                // Read message length (little endian)
                uint16_t msg_length = static_cast<uint16_t>(packet_buffer_[i + 2]) |
                                      (static_cast<uint16_t>(packet_buffer_[i + 3]) << 8);

                // Check if we have the complete packet
                if (i + msg_length > packet_buffer_.size())
                {
                    // Incomplete packet, keep the header for next time
                    if (i > 0)
                    {
                        packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + i);
                    }
                    return false;
                }

                // Try to parse the complete packet
                if (parse_vips_packet(packet_buffer_, i, data))
                {
                    // Remove the parsed packet from buffer
                    packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + i + msg_length);
                    return true;
                }
                else
                {
                    // Parsing failed, remove just the header and continue searching
                    packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + i + 2);
                    return false;
                }
            }
        }

        // No header found, remove all but the last byte (in case it's the start of a header)
        if (packet_buffer_.size() > 1)
        {
            packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.end() - 1);
        }
        return false;
    }

    bool VipsDriverNode::parse_vips_packet(const std::vector<uint8_t> &buffer, size_t offset, VipsData &data)
    {
        // Initialize data structure
        data = VipsData{};
        data.valid = false;

        // Check minimum packet size (header + length + mask + time + position + checksum)
        if (offset + 34 > buffer.size())
        {
            return false;
        }

        size_t read_pos = offset;

        // Verify header
        if (buffer[read_pos] != VIPS_HEADER_1 || buffer[read_pos + 1] != VIPS_HEADER_2)
        {
            return false;
        }
        read_pos += 2;

        // Read message length
        uint16_t msg_length = static_cast<uint16_t>(buffer[read_pos]) |
                              (static_cast<uint16_t>(buffer[read_pos + 1]) << 8);
        read_pos += 2;

        // Verify we have the complete message
        if (offset + msg_length > buffer.size())
        {
            return false;
        }

        // Verify checksum
        uint16_t calculated_crc = calculate_crc(buffer, offset, msg_length - 2);
        uint16_t packet_crc = static_cast<uint16_t>(buffer[offset + msg_length - 2]) << 8 |
                              static_cast<uint16_t>(buffer[offset + msg_length - 1]);

        if (calculated_crc != packet_crc)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "VIPS packet checksum mismatch: calculated=0x%04X, packet=0x%04X",
                                 calculated_crc, packet_crc);
            return false;
        }

        // Read mask flags
        data.mask_flags = static_cast<uint32_t>(buffer[read_pos]) |
                          (static_cast<uint32_t>(buffer[read_pos + 1]) << 8) |
                          (static_cast<uint32_t>(buffer[read_pos + 2]) << 16) |
                          (static_cast<uint32_t>(buffer[read_pos + 3]) << 24);
        read_pos += 4;

        // Read time
        data.time_ms = static_cast<uint32_t>(buffer[read_pos]) |
                       (static_cast<uint32_t>(buffer[read_pos + 1]) << 8) |
                       (static_cast<uint32_t>(buffer[read_pos + 2]) << 16) |
                       (static_cast<uint32_t>(buffer[read_pos + 3]) << 24);
        read_pos += 4;

        // Read position (always present - 20 bytes)
        data.is_global_position = (data.mask_flags & OUTPUT_GLOBAL_POSITION) != 0;

        if (data.is_global_position)
        {
            // Read as latitude/longitude/altitude
            std::memcpy(&data.position.global.latitude, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.global.longitude, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.global.altitude, &buffer[read_pos], 4);
            read_pos += 4;
        }
        else
        {
            // Read as local x/y/z
            std::memcpy(&data.position.local.x, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.local.y, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.local.z, &buffer[read_pos], 4);
            read_pos += 4;
        }

        // Read status block
        if (data.mask_flags & OUTPUT_STATUS)
        {
            data.has_status = true;
            data.beacons = buffer[read_pos++];
            data.solution_type = buffer[read_pos++];
            data.kf_status = static_cast<uint16_t>(buffer[read_pos]) |
                             (static_cast<uint16_t>(buffer[read_pos + 1]) << 8);
            read_pos += 2;
        }

        // Read orientation block
        if (data.mask_flags & OUTPUT_ORIENTATION)
        {
            data.has_orientation = true;
            std::memcpy(&data.roll, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.pitch, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.yaw, &buffer[read_pos], 4);
            read_pos += 4;
        }

        // Read velocity block
        if (data.mask_flags & OUTPUT_VELOCITY)
        {
            data.has_velocity = true;
            if (data.is_global_position)
            {
                std::memcpy(&data.velocity.global.speed_kmh, &buffer[read_pos], 4);
                read_pos += 4;
                std::memcpy(&data.velocity.global.heading, &buffer[read_pos], 4);
                read_pos += 4;
            }
            else
            {
                std::memcpy(&data.velocity.local.vx, &buffer[read_pos], 4);
                read_pos += 4;
                std::memcpy(&data.velocity.local.vy, &buffer[read_pos], 4);
                read_pos += 4;
            }
        }

        // Read vertical velocity
        if (data.mask_flags & OUTPUT_VERT_VELOCITY)
        {
            data.has_vertical_velocity = true;
            std::memcpy(&data.vertical_velocity, &buffer[read_pos], 4);
            read_pos += 4;
        }

        // Read uncertainty data
        if (data.mask_flags & OUTPUT_UNCERTAINTY)
        {
            data.has_uncertainty = true;
            // Position uncertainty (always present if uncertainty bit is set)
            for (int i = 0; i < 3; ++i)
            {
                std::memcpy(&data.pos_uncertainty[i], &buffer[read_pos], 4);
                read_pos += 4;
            }
            // Orientation uncertainty (if orientation is present)
            if (data.mask_flags & OUTPUT_ORIENTATION)
            {
                for (int i = 0; i < 3; ++i)
                {
                    std::memcpy(&data.orient_uncertainty[i], &buffer[read_pos], 4);
                    read_pos += 4;
                }
            }
            // Velocity uncertainty (if velocity is present)
            if (data.mask_flags & OUTPUT_VELOCITY)
            {
                for (int i = 0; i < 3; ++i)
                {
                    std::memcpy(&data.vel_uncertainty[i], &buffer[read_pos], 4);
                    read_pos += 4;
                }
            }
        }

        // Read accuracy data
        if (data.mask_flags & OUTPUT_ACCURACY)
        {
            data.has_accuracy = true;
            data.pos_residual = static_cast<float>(buffer[read_pos]) / 20.0f;
            read_pos++;
            data.reliability_flags = buffer[read_pos++];
            data.vel_residual = static_cast<float>(buffer[read_pos]) / 10.0f;
            read_pos++;
            data.rover_id = buffer[read_pos++];
        }

        // Skip other fields for now (RAW_VIPS, RAW_IMU, UNIT_ID, FIZ_DATA, etc.)
        // These can be added later if needed

        // Read quaternions if present
        if (data.mask_flags & OUTPUT_QUATERNIONS)
        {
            data.has_quaternions = true;
            std::memcpy(&data.quat_x, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.quat_i, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.quat_j, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.quat_k, &buffer[read_pos], 4);
            read_pos += 4;
        }

        data.valid = true;
        return true;
    }

    uint16_t VipsDriverNode::calculate_crc(const std::vector<uint8_t> &data, size_t start, size_t length)
    {
        uint16_t crc = 0;
        for (size_t i = 0; i < length; ++i)
        {
            uint16_t temp = data[start + i];
            crc = crc ^ (temp << 8);
            crc = crc % 0x10000;
            for (int j = 0; j < 8; ++j)
            {
                if ((crc & 0x8000) == 0x8000)
                {
                    crc = crc << 1;
                    crc = crc ^ VIPS_CRC_POLYNOMIAL;
                }
                else
                {
                    crc = crc << 1;
                }
                crc = crc % 0x10000;
            }
        }
        return crc;
    }

    void VipsDriverNode::publish_odometry(const VipsData &data)
    {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = odom_frame_id_;
        odom_msg->child_frame_id = base_frame_id_;

        // Position
        if (data.is_global_position)
        {
            // For global coordinates, we would need to convert lat/lon to local coordinates
            // For now, we'll set position to zero and log a warning
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Global position data received but conversion to local coordinates not implemented");
            odom_msg->pose.pose.position.x = 0.0;
            odom_msg->pose.pose.position.y = 0.0;
            odom_msg->pose.pose.position.z = 0.0;
        }
        else
        {
            odom_msg->pose.pose.position.x = data.position.local.x;
            odom_msg->pose.pose.position.y = data.position.local.y;
            odom_msg->pose.pose.position.z = data.position.local.z;
        }

        // Orientation
        if (data.has_quaternions)
        {
            // Use quaternion data if available
            odom_msg->pose.pose.orientation.x = data.quat_x;
            odom_msg->pose.pose.orientation.y = data.quat_i;
            odom_msg->pose.pose.orientation.z = data.quat_j;
            odom_msg->pose.pose.orientation.w = data.quat_k;
        }
        else if (data.has_orientation)
        {
            // Convert Euler angles to quaternion
            tf2::Quaternion q;
            q.setRPY(data.roll * M_PI / 180.0, data.pitch * M_PI / 180.0, data.yaw * M_PI / 180.0);
            odom_msg->pose.pose.orientation.x = q.x();
            odom_msg->pose.pose.orientation.y = q.y();
            odom_msg->pose.pose.orientation.z = q.z();
            odom_msg->pose.pose.orientation.w = q.w();
        }

        // Velocity
        if (data.has_velocity)
        {
            if (data.is_global_position)
            {
                // Convert speed/heading to velocity components
                double speed_ms = data.velocity.global.speed_kmh / 3.6; // km/h to m/s
                double heading_rad = data.velocity.global.heading * M_PI / 180.0;
                odom_msg->twist.twist.linear.x = speed_ms * cos(heading_rad);
                odom_msg->twist.twist.linear.y = speed_ms * sin(heading_rad);
            }
            else
            {
                odom_msg->twist.twist.linear.x = data.velocity.local.vx;
                odom_msg->twist.twist.linear.y = data.velocity.local.vy;
            }
        }

        if (data.has_vertical_velocity)
        {
            odom_msg->twist.twist.linear.z = data.vertical_velocity;
        }

        // Set covariance matrices based on uncertainty data if available
        if (data.has_uncertainty)
        {
            // Position covariance (6x6 matrix stored as 36-element array)
            odom_msg->pose.covariance[0] = data.pos_uncertainty[0] * data.pos_uncertainty[0];  // x
            odom_msg->pose.covariance[7] = data.pos_uncertainty[1] * data.pos_uncertainty[1];  // y
            odom_msg->pose.covariance[14] = data.pos_uncertainty[2] * data.pos_uncertainty[2]; // z

            if (data.has_orientation && data.mask_flags & OUTPUT_ORIENTATION)
            {
                // Orientation covariance (roll, pitch, yaw)
                double roll_var = (data.orient_uncertainty[0] * M_PI / 180.0);
                double pitch_var = (data.orient_uncertainty[1] * M_PI / 180.0);
                double yaw_var = (data.orient_uncertainty[2] * M_PI / 180.0);
                odom_msg->pose.covariance[21] = roll_var * roll_var;
                odom_msg->pose.covariance[28] = pitch_var * pitch_var;
                odom_msg->pose.covariance[35] = yaw_var * yaw_var;
            }

            if (data.has_velocity && data.mask_flags & OUTPUT_VELOCITY)
            {
                // Velocity covariance
                odom_msg->twist.covariance[0] = data.vel_uncertainty[0] * data.vel_uncertainty[0];  // vx
                odom_msg->twist.covariance[7] = data.vel_uncertainty[1] * data.vel_uncertainty[1];  // vy
                odom_msg->twist.covariance[14] = data.vel_uncertainty[2] * data.vel_uncertainty[2]; // vz
            }
        }

        odom_pub_->publish(std::move(odom_msg));
    }

    void VipsDriverNode::publish_imu(const VipsData &data)
    {
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        imu_msg->header.stamp = this->get_clock()->now();
        imu_msg->header.frame_id = imu_frame_id_;

        // Orientation
        if (data.has_quaternions)
        {
            // Use quaternion data if available
            imu_msg->orientation.x = data.quat_x;
            imu_msg->orientation.y = data.quat_i;
            imu_msg->orientation.z = data.quat_j;
            imu_msg->orientation.w = data.quat_k;
        }
        else if (data.has_orientation)
        {
            // Convert Euler angles to quaternion
            tf2::Quaternion q;
            q.setRPY(data.roll * M_PI / 180.0, data.pitch * M_PI / 180.0, data.yaw * M_PI / 180.0);
            imu_msg->orientation.x = q.x();
            imu_msg->orientation.y = q.y();
            imu_msg->orientation.z = q.z();
            imu_msg->orientation.w = q.w();
        }

        // Angular Velocity - VIPS doesn't provide IMU angular velocity directly
        // We could potentially calculate it from orientation changes, but for now set to zero
        imu_msg->angular_velocity.x = 0.0;
        imu_msg->angular_velocity.y = 0.0;
        imu_msg->angular_velocity.z = 0.0;

        // Linear Acceleration - VIPS doesn't provide IMU acceleration directly
        // We could potentially calculate it from velocity changes, but for now set to zero
        imu_msg->linear_acceleration.x = 0.0;
        imu_msg->linear_acceleration.y = 0.0;
        imu_msg->linear_acceleration.z = 0.0;

        // Set covariance matrices
        if (data.has_uncertainty && data.has_orientation && data.mask_flags & OUTPUT_ORIENTATION)
        {
            // Orientation covariance
            double roll_var = (data.orient_uncertainty[0] * M_PI / 180.0);
            double pitch_var = (data.orient_uncertainty[1] * M_PI / 180.0);
            double yaw_var = (data.orient_uncertainty[2] * M_PI / 180.0);
            imu_msg->orientation_covariance[0] = roll_var * roll_var;
            imu_msg->orientation_covariance[4] = pitch_var * pitch_var;
            imu_msg->orientation_covariance[8] = yaw_var * yaw_var;
        }
        else
        {
            // Set to unknown if no uncertainty data
            imu_msg->orientation_covariance[0] = -1;
        }

        // Set angular velocity and linear acceleration covariances to unknown
        imu_msg->angular_velocity_covariance[0] = -1;
        imu_msg->linear_acceleration_covariance[0] = -1;

        imu_pub_->publish(std::move(imu_msg));
    }

} // namespace vips_driver

RCLCPP_COMPONENTS_REGISTER_NODE(vips_driver::VipsDriverNode)

// Main function for standalone executable
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vips_driver::VipsDriverNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
