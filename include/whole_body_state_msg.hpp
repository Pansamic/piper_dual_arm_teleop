#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <codec.hpp>
#include <crc.h>

class WholeBodyStateMsg
{
public:
    static constexpr size_t packet_size = 112;

    struct Packet
    {
        // mask explanation
        // enable: 1
        // disable: 0
        // Control Enable | base control | left hand control | right hand control | base vel valid | left hand vel valid | right hand vel valid | padding
        uint8_t header = 0xFB;        // 0XFB
        uint8_t mask;                 //
        uint32_t cnt;                 // packet count
        uint64_t time;                // timestamp unit: ns
        float base_x, base_y, base_z; // global position of baselink in FLU coordinate
        uint32_t left_hand_pos;       // position of left hand relative to base in FLU coordinate   10-bit compressed
        uint32_t right_hand_pos;      // position of right hand relative to base in FLU coordinate  10-bit compressed

        // uint64_t rotation_upper; // containing 3 quaternion:
        // uint64_t rotation_lower; // rotation of head, right hand and left hand

        float base_quat[4]; // quaternion of head
        float left_hand_quat[4];  // quaternion of left hand
        float right_hand_quat[4]; // quaternion of right hand

        uint32_t base_vel;   // relative velocity of base in FLU coordinate                         10-bit compressed
        uint32_t base_omega; // relative angular velocity of base in FLU coordinate                 10-bit compressed

        uint32_t left_hand_vel;    // relative velocity of left hand in FLU coordinate              10-bit compressed
        uint32_t right_hand_vel;   // relative velocity of right hand in FLU coordinate             10-bit compressed
        uint32_t left_hand_omega;  // relative angular velocity of left hand in FLU coordinate      10-bit compressed
        uint32_t right_hand_omega; // relative angular velocity of right hand in FLU coordinate     10-bit compressed
        uint16_t left_gripper_ctrl; // left gripper control
        uint16_t right_gripper_ctrl; // right gripper control
        uint16_t crc_bits;         // 16-bit KERMIT CRC
    };

    struct Definition
    {
        // mask explanation
        // enable: 1
        // disable: 0
        // Control Enable | base control | left hand control | right hand control | base vel valid | left hand vel valid | right hand vel valid | padding
        uint8_t mask;                   //
        uint32_t cnt;                   // packet count
        uint64_t time;                  // timestamp unit: ns
        Eigen::Vector3d base_pos;       // global position of baselink in FLU coordinate
        Eigen::Vector3d left_hand_pos;  // position of left hand relative to base in FLU coordinate
        Eigen::Vector3d right_hand_pos; // position of right hand relative to base in FLU coordinate

        Eigen::Quaternionf base_quat;       //
        Eigen::Quaternionf left_hand_quat;  //
        Eigen::Quaternionf right_hand_quat; //

        Eigen::Vector3d base_lin_vel; // relative velocity of base in FLU coordinate
        Eigen::Vector3d base_ang_vel; // relative angular velocity of base in FLU coordinate

        Eigen::Vector3d left_hand_lin_vel;  // relative velocity of left hand in FLU coordinate
        Eigen::Vector3d right_hand_lin_vel; // relative velocity of right hand in FLU coordinate
        Eigen::Vector3d left_hand_ang_vel;  // relative angular velocity of left hand in FLU coordinate
        Eigen::Vector3d right_hand_ang_vel; // relative angular velocity of right hand in FLU coordinate
        double left_gripper_ctrl; // left gripper control
        double right_gripper_ctrl; // right gripper control
    };

    static void encode(std::array<std::byte,packet_size>& buffer, const Definition& data)
    {
        Packet packet;
        packet.header = 0xFB;
        packet.mask = data.mask;
        packet.cnt = data.cnt;
        packet.time = data.time;
        packet.base_x = static_cast<float>(data.base_pos(0));
        packet.base_y = static_cast<float>(data.base_pos(1));
        packet.base_z = static_cast<float>(data.base_pos(2));
        packet.left_hand_pos = Encode3D<double,10>(data.left_hand_pos(0),data.left_hand_pos(1),data.left_hand_pos(2));
        packet.right_hand_pos = Encode3D<double,10>(data.right_hand_pos(0),data.right_hand_pos(1),data.right_hand_pos(2));
        // std::tie(packet.rotation_upper, packet.rotation_lower) = EncodeQuaternions<double>(data.base_quat, data.left_hand_quat, data.right_hand_quat);
        packet.base_vel = Encode3D<double,10>(data.base_lin_vel(0),data.base_lin_vel(1),data.base_lin_vel(2), 3.0);
        packet.base_omega = Encode3D<double,10>(data.base_ang_vel(0),data.base_ang_vel(1),data.base_ang_vel(2), 3.14);
        packet.left_hand_vel = Encode3D<double,10>(data.left_hand_lin_vel(0),data.left_hand_lin_vel(1),data.left_hand_lin_vel(2), 5.0);
        packet.left_hand_omega = Encode3D<double,10>(data.left_hand_ang_vel(0),data.left_hand_ang_vel(1),data.left_hand_ang_vel(2), 5.0);
        packet.right_hand_vel = Encode3D<double,10>(data.right_hand_lin_vel(0),data.right_hand_lin_vel(1),data.right_hand_lin_vel(2), 6.28);
        packet.right_hand_omega = Encode3D<double,10>(data.right_hand_ang_vel(0),data.right_hand_ang_vel(1),data.right_hand_ang_vel(2), 6.28);
        packet.left_gripper_ctrl = EncodeFloating<double,16>(data.left_gripper_ctrl, 1.0);
        packet.right_gripper_ctrl = EncodeFloating<double,16>(data.right_gripper_ctrl, 1.0);

        size_t offset = 0;
        memcpy(&buffer[offset], &packet.header, sizeof(packet.header));
        offset += sizeof(packet.header);
        memcpy(&buffer[offset], &packet.mask, sizeof(packet.mask));
        offset += sizeof(packet.mask);
        memcpy(&buffer[offset], &packet.cnt, sizeof(packet.cnt));
        offset += sizeof(packet.cnt);
        memcpy(&buffer[offset], &packet.time, sizeof(packet.time));
        offset += sizeof(packet.time);
        memcpy(&buffer[offset], &packet.base_x, sizeof(packet.base_x));
        offset += sizeof(packet.base_x);
        memcpy(&buffer[offset], &packet.base_y, sizeof(packet.base_y));
        offset += sizeof(packet.base_y);
        memcpy(&buffer[offset], &packet.base_z, sizeof(packet.base_z));
        offset += sizeof(packet.base_z);
        memcpy(&buffer[offset], &packet.left_hand_pos, sizeof(packet.left_hand_pos));
        offset += sizeof(packet.left_hand_pos);
        memcpy(&buffer[offset], &packet.right_hand_pos, sizeof(packet.right_hand_pos));
        offset += sizeof(packet.right_hand_pos);
        // memcpy(&buffer[offset], &packet.rotation_upper, sizeof(packet.rotation_upper));
        // offset += sizeof(packet.rotation_upper);
        // memcpy(&buffer[offset], &packet.rotation_lower, sizeof(packet.rotation_lower));
        // offset += sizeof(packet.rotation_lower);
        memcpy(&buffer[offset], &packet.base_quat, sizeof(packet.base_quat));
        offset += sizeof(packet.base_quat);
        memcpy(&buffer[offset], &packet.left_hand_quat, sizeof(packet.left_hand_quat));
        offset += sizeof(packet.left_hand_quat);
        memcpy(&buffer[offset], &packet.right_hand_quat, sizeof(packet.right_hand_quat));
        offset += sizeof(packet.right_hand_quat);
        memcpy(&buffer[offset], &packet.base_vel, sizeof(packet.base_vel));
        offset += sizeof(packet.base_vel);
        memcpy(&buffer[offset], &packet.base_omega, sizeof(packet.base_omega));
        offset += sizeof(packet.base_omega);
        memcpy(&buffer[offset], &packet.left_hand_vel, sizeof(packet.left_hand_vel));
        offset += sizeof(packet.left_hand_vel);
        memcpy(&buffer[offset], &packet.right_hand_vel, sizeof(packet.right_hand_vel));
        offset += sizeof(packet.right_hand_vel);
        memcpy(&buffer[offset], &packet.left_hand_omega, sizeof(packet.left_hand_omega));
        offset += sizeof(packet.left_hand_omega);
        memcpy(&buffer[offset], &packet.right_hand_omega, sizeof(packet.right_hand_omega));
        offset += sizeof(packet.right_hand_omega);
        memcpy(&buffer[offset], &packet.left_gripper_ctrl, sizeof(packet.left_gripper_ctrl));
        offset += sizeof(packet.left_gripper_ctrl);
        memcpy(&buffer[offset], &packet.right_gripper_ctrl, sizeof(packet.right_gripper_ctrl));
        offset += sizeof(packet.right_gripper_ctrl);
        packet.crc_bits = CRC::CalculateBits(buffer.data(), 78, CRC::CRC_16_KERMIT());
        memcpy(&buffer[offset], &packet.crc_bits, sizeof(packet.crc_bits));
        offset += sizeof(packet.crc_bits);
    };

    static void decode(const std::array<std::byte,packet_size>& buffer, Definition& data)
    {   
        memset(&data, 0, sizeof(data));
        uint16_t crc = CRC::CalculateBits(buffer.data(), packet_size-sizeof(Packet::crc_bits), CRC::CRC_16_KERMIT());
        if (crc != *(uint16_t*)&buffer[packet_size-sizeof(Packet::crc_bits)])
        {
            throw std::runtime_error("CRC check failed");
        }

        Packet packet;
        size_t ofs = 0;
        packet.header = *(decltype(packet.header)*)(&buffer[ofs]);
        ofs += sizeof(packet.header);
        packet.mask = *(decltype(packet.mask)*)(&buffer[ofs]);
        ofs += sizeof(packet.mask);
        packet.cnt = *(decltype(packet.cnt)*)(&buffer[ofs]);
        ofs += sizeof(packet.cnt);
        packet.time = *(decltype(packet.time)*)(&buffer[ofs]);
        ofs += sizeof(packet.time);
        packet.base_x = *(decltype(packet.base_x)*)(&buffer[ofs]);
        ofs += sizeof(packet.base_x);
        packet.base_y = *(decltype(packet.base_y)*)(&buffer[ofs]);
        ofs += sizeof(packet.base_y);
        packet.base_z = *(decltype(packet.base_z)*)(&buffer[ofs]);
        ofs += sizeof(packet.base_z);
        packet.left_hand_pos = *(decltype(packet.left_hand_pos)*)(&buffer[ofs]);
        ofs += sizeof(packet.left_hand_pos);
        packet.right_hand_pos = *(decltype(packet.right_hand_pos)*)(&buffer[ofs]);
        ofs += sizeof(packet.right_hand_pos);
        // packet.rotation_upper = *(decltype(packet.rotation_upper)*)(&buffer[ofs]);
        // ofs += sizeof(packet.rotation_upper);
        // packet.rotation_lower = *(decltype(packet.rotation_lower)*)(&buffer[ofs]);
        // ofs += sizeof(packet.rotation_lower);
        memcpy(&packet.base_quat, &buffer[ofs], sizeof(packet.base_quat));
        ofs += sizeof(packet.base_quat);
        memcpy(&packet.left_hand_quat, &buffer[ofs], sizeof(packet.left_hand_quat));
        ofs += sizeof(packet.left_hand_quat);
        memcpy(&packet.right_hand_quat, &buffer[ofs], sizeof(packet.right_hand_quat));
        ofs += sizeof(packet.right_hand_quat);
        packet.base_vel = *(decltype(packet.base_vel)*)(&buffer[ofs]);
        ofs += sizeof(packet.base_vel);
        packet.base_omega = *(decltype(packet.base_omega)*)(&buffer[ofs]);
        ofs += sizeof(packet.base_omega);
        packet.left_hand_vel = *(decltype(packet.left_hand_vel)*)(&buffer[ofs]);
        ofs += sizeof(packet.left_hand_vel);
        packet.right_hand_vel = *(decltype(packet.right_hand_vel)*)(&buffer[ofs]);
        ofs += sizeof(packet.right_hand_vel);
        packet.left_hand_omega = *(decltype(packet.left_hand_omega)*)(&buffer[ofs]);
        ofs += sizeof(packet.left_hand_omega);
        packet.right_hand_omega = *(decltype(packet.right_hand_omega)*)(&buffer[ofs]);
        ofs += sizeof(packet.right_hand_omega);
        packet.right_gripper_ctrl = *(decltype(packet.right_gripper_ctrl)*)(&buffer[ofs]);
        ofs += sizeof(packet.right_gripper_ctrl);
        packet.left_gripper_ctrl = *(decltype(packet.left_gripper_ctrl)*)(&buffer[ofs]);
        ofs += sizeof(packet.left_gripper_ctrl);
        packet.crc_bits = *(decltype(packet.crc_bits)*)(&buffer[ofs]);

        data.mask = packet.mask;
        data.cnt = packet.cnt;
        data.time = packet.time;
        data.base_pos << packet.base_x, packet.base_y, packet.base_z;

        /* Left Hand Position */
        auto [left_hand_pos_x, left_hand_pos_y, left_hand_pos_z] = Decode3D<double,10>(packet.left_hand_pos, 2.0);
        data.left_hand_pos << left_hand_pos_x, left_hand_pos_y, left_hand_pos_z;

        /* Right Hand Position */
        auto [right_hand_pos_x, right_hand_pos_y, right_hand_pos_z] = Decode3D<double,10>(packet.right_hand_pos, 2.0);
        data.right_hand_pos << right_hand_pos_x, right_hand_pos_y, right_hand_pos_z;

        /* Orientation of Head, Left Hand and Right Hand */
        // std::tie(data.base_quat, data.left_hand_quat, data.right_hand_quat) = DecodeQuaternions<double>(packet.rotation_upper, packet.rotation_lower);
        memcpy(data.base_quat.coeffs().data(), &packet.base_quat, sizeof(packet.base_quat));
        memcpy(data.left_hand_quat.coeffs().data(), &packet.left_hand_quat, sizeof(packet.left_hand_quat));
        memcpy(data.right_hand_quat.coeffs().data(), &packet.right_hand_quat, sizeof(packet.right_hand_quat));

        /* Base Linear Velocity */
        auto [base_lin_vel_x, base_lin_vel_y, base_lin_vel_z] = Decode3D<double,10>(packet.base_vel, 3.0);
        data.base_lin_vel << base_lin_vel_x, base_lin_vel_y, base_lin_vel_z;

        /* Base Angular Velocity */
        auto [base_ang_vel_x, base_ang_vel_y, base_ang_vel_z] = Decode3D<double,10>(packet.base_omega, 3.14);
        data.base_ang_vel << base_ang_vel_x, base_ang_vel_y, base_ang_vel_z;

        /* Left Hand Linear Velocity */
        auto [left_hand_lin_vel_x, left_hand_lin_vel_y, left_hand_lin_vel_z] = Decode3D<double,10>(packet.left_hand_vel, 5.0);
        data.left_hand_lin_vel << left_hand_lin_vel_x, left_hand_lin_vel_y, left_hand_lin_vel_z;

        /* Left Hand Angular Velocity */
        auto [left_hand_ang_vel_x, left_hand_ang_vel_y, left_hand_ang_vel_z] = Decode3D<double,10>(packet.right_hand_vel, 6.28);
        data.left_hand_ang_vel << left_hand_ang_vel_x, left_hand_ang_vel_y, left_hand_ang_vel_z;

        /* Right Hand Linear Velocity */
        auto [right_hand_lin_vel_x, right_hand_lin_vel_y, right_hand_lin_vel_z] = Decode3D<double,10>(packet.right_hand_vel, 5.0);
        data.right_hand_lin_vel << right_hand_lin_vel_x, right_hand_lin_vel_y, right_hand_lin_vel_z;

        /* Right Hand Angular Velocity */
        auto [right_hand_ang_vel_x, right_hand_ang_vel_y, right_hand_ang_vel_z] = Decode3D<double,10>(packet.right_hand_vel, 6.28);
        data.right_hand_ang_vel << right_hand_ang_vel_x, right_hand_ang_vel_y, right_hand_ang_vel_z;

        double left_gripper_ctrl = DecodeFloating<double,16>(packet.left_gripper_ctrl, 1.0);
        double right_gripper_ctrl = DecodeFloating<double,16>(packet.right_gripper_ctrl, 1.0);

        data.left_gripper_ctrl = left_gripper_ctrl;
        data.right_gripper_ctrl = right_gripper_ctrl;
    }
};
