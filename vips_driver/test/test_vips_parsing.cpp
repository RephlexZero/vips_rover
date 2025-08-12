#include <gtest/gtest.h>
#include <vector>
#include <cstring>
#include "vips_driver/vips_driver.hpp"

class VipsParsingTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Example packet from VIPS documentation
        // 24 D9 36 00 46 00 00 00 50 5F 00 00 D9 CE F7 53 E3 A5 0B 40 58 39 B4 C8 76 BE F3 BF 66 66 E6 3F 0C 20 35 02 CD CC 8C BF 00 00 C0 3F CD 4C 0C 43 00 00 00 FE 2F 25
        example_packet = {
            0x24, 0xD9,                                     // Header
            0x36, 0x00,                                     // Length = 54 bytes
            0x46, 0x00, 0x00, 0x00,                         // Mask = 0x46 (STATUS | ORIENTATION | ACCURACY)
            0x50, 0x5F, 0x00, 0x00,                         // Time = 24400 ms
            0xD9, 0xCE, 0xF7, 0x53, 0xE3, 0xA5, 0x0B, 0x40, // X = 3.456
            0x58, 0x39, 0xB4, 0xC8, 0x76, 0xBE, 0xF3, 0xBF, // Y = -1.234
            0x66, 0x66, 0xE6, 0x3F,                         // Z = 1.8
            0x0C,                                           // Beacons = 12
            0x20,                                           // Solution type = 32 (VIPS)
            0x35, 0x02,                                     // KF Status = 0x0235
            0xCD, 0xCC, 0x8C, 0xBF,                         // Roll = -1.1
            0x00, 0x00, 0xC0, 0x3F,                         // Pitch = 1.5
            0xCD, 0x4C, 0x0C, 0x43,                         // Yaw = 140.3
            0x00, 0x00, 0x00,                               // Accuracy values
            0xFE,                                           // Rover ID = 254
            0x2F, 0x25                                      // Checksum
        };
    }

    std::vector<uint8_t> example_packet;
};

// Mock VIPS driver node for testing
class MockVipsDriverNode
{
public:
    bool parse_vips_packet(const std::vector<uint8_t> &buffer, size_t offset, vips_driver::VipsData &data)
    {
        return parse_packet_impl(buffer, offset, data);
    }

    uint16_t calculate_crc(const std::vector<uint8_t> &data, size_t start, size_t length)
    {
        return calculate_crc_impl(data, start, length);
    }

private:
    bool parse_packet_impl(const std::vector<uint8_t> &buffer, size_t offset, vips_driver::VipsData &data)
    {
        // Copy the implementation from VipsDriverNode::parse_vips_packet
        data = vips_driver::VipsData{};
        data.valid = false;

        if (offset + 34 > buffer.size())
        {
            return false;
        }

        size_t read_pos = offset;

        // Verify header
        if (buffer[read_pos] != vips_driver::VIPS_HEADER_1 || buffer[read_pos + 1] != vips_driver::VIPS_HEADER_2)
        {
            return false;
        }
        read_pos += 2;

        // Read message length
        uint16_t msg_length = static_cast<uint16_t>(buffer[read_pos]) |
                              (static_cast<uint16_t>(buffer[read_pos + 1]) << 8);
        read_pos += 2;

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

        // Read position
        data.is_global_position = (data.mask_flags & vips_driver::OUTPUT_GLOBAL_POSITION) != 0;

        if (data.is_global_position)
        {
            std::memcpy(&data.position.global.latitude, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.global.longitude, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.global.altitude, &buffer[read_pos], 4);
            read_pos += 4;
        }
        else
        {
            std::memcpy(&data.position.local.x, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.local.y, &buffer[read_pos], 8);
            read_pos += 8;
            std::memcpy(&data.position.local.z, &buffer[read_pos], 4);
            read_pos += 4;
        }

        // Read status block
        if (data.mask_flags & vips_driver::OUTPUT_STATUS)
        {
            data.has_status = true;
            data.beacons = buffer[read_pos++];
            data.solution_type = buffer[read_pos++];
            data.kf_status = static_cast<uint16_t>(buffer[read_pos]) |
                             (static_cast<uint16_t>(buffer[read_pos + 1]) << 8);
            read_pos += 2;
        }

        // Read orientation block
        if (data.mask_flags & vips_driver::OUTPUT_ORIENTATION)
        {
            data.has_orientation = true;
            std::memcpy(&data.roll, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.pitch, &buffer[read_pos], 4);
            read_pos += 4;
            std::memcpy(&data.yaw, &buffer[read_pos], 4);
            read_pos += 4;
        }

        // Read accuracy data
        if (data.mask_flags & vips_driver::OUTPUT_ACCURACY)
        {
            data.has_accuracy = true;
            data.pos_residual = static_cast<float>(buffer[read_pos]) / 20.0f;
            read_pos++;
            data.reliability_flags = buffer[read_pos++];
            data.vel_residual = static_cast<float>(buffer[read_pos]) / 10.0f;
            read_pos++;
            data.rover_id = buffer[read_pos++];
        }

        data.valid = true;
        return true;
    }

    uint16_t calculate_crc_impl(const std::vector<uint8_t> &data, size_t start, size_t length)
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
                    crc = crc ^ vips_driver::VIPS_CRC_POLYNOMIAL;
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
};

TEST_F(VipsParsingTest, ParseExamplePacket)
{
    MockVipsDriverNode node;
    vips_driver::VipsData data;

    ASSERT_TRUE(node.parse_vips_packet(example_packet, 0, data));
    ASSERT_TRUE(data.valid);

    // Check basic packet info
    EXPECT_EQ(data.time_ms, 24400);
    EXPECT_EQ(data.mask_flags, 0x46); // STATUS | ORIENTATION | ACCURACY
    EXPECT_FALSE(data.is_global_position);

    // Check position
    EXPECT_NEAR(data.position.local.x, 3.456, 0.001);
    EXPECT_NEAR(data.position.local.y, -1.234, 0.001);
    EXPECT_NEAR(data.position.local.z, 1.8, 0.001);

    // Check status
    EXPECT_TRUE(data.has_status);
    EXPECT_EQ(data.beacons, 12);
    EXPECT_EQ(data.solution_type, 32); // VIPS
    EXPECT_EQ(data.kf_status, 0x0235);

    // Check orientation
    EXPECT_TRUE(data.has_orientation);
    EXPECT_NEAR(data.roll, -1.1, 0.001);
    EXPECT_NEAR(data.pitch, 1.5, 0.001);
    EXPECT_NEAR(data.yaw, 140.3, 0.1);

    // Check accuracy
    EXPECT_TRUE(data.has_accuracy);
    EXPECT_EQ(data.rover_id, 254);
}

TEST_F(VipsParsingTest, InvalidHeader)
{
    MockVipsDriverNode node;
    vips_driver::VipsData data;

    auto bad_packet = example_packet;
    bad_packet[0] = 0x23; // Wrong header

    EXPECT_FALSE(node.parse_vips_packet(bad_packet, 0, data));
}

TEST_F(VipsParsingTest, InvalidChecksum)
{
    MockVipsDriverNode node;
    vips_driver::VipsData data;

    auto bad_packet = example_packet;
    bad_packet[bad_packet.size() - 1] = 0x00; // Wrong checksum

    EXPECT_FALSE(node.parse_vips_packet(bad_packet, 0, data));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
