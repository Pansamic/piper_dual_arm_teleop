#pragma once
#include <cstdint>
#include <codec.hpp>
#include <crc.h>

class DualArmStateMsg
{
public:
    static constexpr size_t packet_size = 27;
    /**
     * @brief data packet structure for dual arm state, total 27 bytes
     * 
     */
    struct Packet
    {
        uint8_t header;               // 0XFB
        uint64_t time;                // timestamp unit: ns
        uint64_t left_arm_joint_pos;  // compressed left arm joint position, 10-bit quantization
        uint64_t right_arm_joint_pos; // compressed right arm joint position, 10-bit quantization
        uint16_t crc_bits;
    };


    struct Definition
    {
        uint64_t time;
        double left_arm_joint1;
        double left_arm_joint2;
        double left_arm_joint3;
        double left_arm_joint4;
        double left_arm_joint5;
        double left_arm_joint6;
        double right_arm_joint1;
        double right_arm_joint2;
        double right_arm_joint3;
        double right_arm_joint4;
        double right_arm_joint5;
        double right_arm_joint6;
    };

    static void encode(std::array<std::byte,packet_size>& buffer, const Definition& data)
    {
        buffer[0] = std::byte{0xFB};     // header

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();

        std::uint64_t nanoseconds = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
        std::memcpy(static_cast<void*>(&buffer[1]), static_cast<const void*>(&nanoseconds), sizeof(nanoseconds));

        std::uint64_t left_arm_joint_pos_enc = EncodeArmStatus<double,10>(data.left_arm_joint1, data.left_arm_joint2, data.left_arm_joint3, data.left_arm_joint4, data.left_arm_joint5, data.left_arm_joint6, 4.5);
        std::memcpy(static_cast<void*>(&buffer[9]), static_cast<const void*>(&left_arm_joint_pos_enc), sizeof(left_arm_joint_pos_enc));

        std::uint64_t right_arm_joint_pos_enc = EncodeArmStatus<double,10>(data.right_arm_joint1, data.right_arm_joint2, data.right_arm_joint3, data.right_arm_joint4, data.right_arm_joint5, data.right_arm_joint6, 4.5);
        std::memcpy(static_cast<void*>(&buffer[17]), static_cast<const void*>(&right_arm_joint_pos_enc), sizeof(right_arm_joint_pos_enc));

        std::uint16_t crc = CRC::CalculateBits(buffer.data(), packet_size-sizeof(uint16_t), CRC::CRC_16_KERMIT());
        std::memcpy(static_cast<void*>(&buffer[25]), static_cast<const void*>(&crc), sizeof(crc));
    };

    static void decode(const std::array<std::byte,packet_size>& buffer, Definition& data)
    {
        /* Header Checking */
        if ( static_cast<uint8_t>(buffer[0]) != 0xFB )
        {
            throw std::runtime_error("Invalid header");
        }
        /* CRC Checking */
        uint16_t crc = CRC::CalculateBits(buffer.data(), packet_size-sizeof(uint16_t), CRC::CRC_16_KERMIT());
        if (crc != *(uint16_t*)buffer[packet_size-sizeof(uint16_t)] )
        {
            throw std::runtime_error("CRC check failed");
        }
        memset(&data, 0, sizeof(data));
        
        Packet packet = {0};
        size_t ofs = 0;
        packet.header = *(decltype(packet.header)*)(&buffer[ofs]);
        ofs += sizeof(packet.header);
        packet.time = *(decltype(packet.time)*)(&buffer[ofs]);
        ofs += sizeof(packet.time);
        packet.left_arm_joint_pos = *(decltype(packet.left_arm_joint_pos)*)(&buffer[ofs]);
        ofs += sizeof(packet.left_arm_joint_pos);
        packet.right_arm_joint_pos = *(uint64_t*)(&buffer[ofs]);
        ofs += sizeof(packet.right_arm_joint_pos);
        packet.crc_bits = *(decltype(packet.crc_bits)*)(&buffer[ofs]);

        std::tie(data.left_arm_joint1, data.left_arm_joint2, data.left_arm_joint3, data.left_arm_joint4, data.left_arm_joint5, data.left_arm_joint6) = DecodeArmStatus<double,10>(packet.left_arm_joint_pos, 4.5);
        std::tie(data.right_arm_joint1, data.right_arm_joint2, data.right_arm_joint3, data.right_arm_joint4, data.right_arm_joint5, data.right_arm_joint6) = DecodeArmStatus<double,10>(packet.right_arm_joint_pos, 4.5);
    };
};
