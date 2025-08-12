/**
 * @file messenger.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Communication processor with message receiver and sender.
 * @version 0.1
 * @date 2025-08-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <asio.hpp>
#include <itc/backend/RingBuf.hpp>
#include <comm_channel.hpp>
#include <msgs/whole_body_msg/whole_body_msg.h>
#include <msgs/whole_body_msg/whole_body_receiver.hpp>
#include <msgs/whole_body_msg/whole_body_sender.hpp>
#include <msgs/nav_state_msg/nav_state_msg.h>
#include <msgs/nav_state_msg/nav_state_receiver.hpp>
#include <msgs/nav_state_msg/nav_state_sender.hpp>

template<ChannelMode Mode>
class Messenger
{
public:
    Messenger() = delete;

    explicit Messenger(std::string local_endpoint, std::string remote_endpoint) : 
        channel_(this->io_context_, local_endpoint, remote_endpoint){}

    explicit Messenger(std::string local_ipv4_addr, int local_port, std::string remote_ipv4_addr, int remote_port) : 
        channel_(this->io_context_, local_ipv4_addr, local_port, remote_ipv4_addr, remote_port){}

    ~Messenger() = default;

    void start(bool enable_sender, bool enable_receiver)
    {
        this->sender_enabled = enable_sender;
        this->receiver_enabled = enable_receiver;

        if ( enable_sender )
        {
            this->channel_.bind_message_queue("whole_body_sender", ParserType::Sender, this->send_mq_);
            this->channel_.enable_sender();
        }

        if ( enable_receiver )
        {
            this->channel_.bind_message_queue("whole_body_receiver", ParserType::Receiver, this->recv_mq_);
            this->channel_.enable_receiver();
        }

        this->io_context_thread_ = std::thread([&]() { this->io_context_.run(); });
    }
    void stop()
    {
        this->io_context_.stop();
        this->io_context_thread_.join();
    }
    /**
     * @brief Send `whole_body_msg` to client.
     * 
     * @tparam T float precision type, e.g. `float`, `double`. 
     * @param enable Enable arm movement.
     * @param left_arm_joint_pos
     * @param right_arm_joint_pos 
     * @return true 
     * @return false 
     */
    template <typename T>
    bool sendDualArmJointPosition(bool enable, const std::array<T, 6>& left_arm_joint_pos, const std::array<T, 6>& right_arm_joint_pos)
    {
        if ( !this->sender_enabled )
        {
            return false;
        }
        whole_body_msg msg;
        msg.mask = 0;
        if ( enable )
        {
            msg.mask = (1<<15) | (1<<13) | (1<<12);
        }
        for ( std::size_t i=0 ; i<6 ; i++ )
        {
            msg.left_arm_joint_pos[i] = static_cast<float>(left_arm_joint_pos[i]);
            msg.right_arm_joint_pos[i] = static_cast<float>(right_arm_joint_pos[i]);
        }
        this->send_mq_.enqueue(msg);
        return true;
    }

    template <typename T>
    bool sendDualArmHandPose(bool enable,
        const std::array<T, 3>& left_hand_position, const std::array<T, 4>& left_hand_orientation,
        const std::array<T, 3>& right_hand_position, const std::array<T, 4>& right_hand_orientation)
    {
        if ( !this->sender_enabled )
        {
            return false;
        }
        whole_body_msg msg;
        msg.mask = 0;
        if ( enable )
        {
            msg.mask = (1<<15) | (1<<13) | (1<<12);
        }
        msg.left_hand_pos[0] = static_cast<float>(left_hand_position[0]);
        msg.left_hand_pos[1] = static_cast<float>(left_hand_position[1]);
        msg.left_hand_pos[2] = static_cast<float>(left_hand_position[2]);

        msg.left_hand_quat[0] = static_cast<float>(left_hand_orientation[0]);
        msg.left_hand_quat[1] = static_cast<float>(left_hand_orientation[1]);
        msg.left_hand_quat[2] = static_cast<float>(left_hand_orientation[2]);
        msg.left_hand_quat[3] = static_cast<float>(left_hand_orientation[3]);

        msg.right_hand_pos[0] = static_cast<float>(right_hand_position[0]);
        msg.right_hand_pos[1] = static_cast<float>(right_hand_position[1]);
        msg.right_hand_pos[2] = static_cast<float>(right_hand_position[2]);

        msg.right_hand_quat[0] = static_cast<float>(right_hand_orientation[0]);
        msg.right_hand_quat[1] = static_cast<float>(right_hand_orientation[1]);
        msg.right_hand_quat[2] = static_cast<float>(right_hand_orientation[2]);
        msg.right_hand_quat[3] = static_cast<float>(right_hand_orientation[3]);

        this->send_mq_.enqueue(msg);
        return true;
    }

    template <typename T>
    bool recvDualArmJointPosition(bool& enable, std::array<T, 6>& left_arm_joint_pos, std::array<T, 6>& right_arm_joint_pos)
    {
        if ( !this->receiver_enabled )
        {
            return false;
        }
        if ( this->recv_mq_.empty() )
        {
            return false;
        }
        whole_body_msg msg;
        this->recv_mq_.dequeue(msg);
        enable = (msg.mask & (1<<15)) && (msg.mask & (1<<13)) && (msg.mask & (1<<12));
        if ( enable )
        {
            for ( std::size_t i=0 ; i<6 ; i++ )
            {
                left_arm_joint_pos[i] = static_cast<T>(msg.left_arm_joint_pos[i]);
                right_arm_joint_pos[i] = static_cast<T>(msg.right_arm_joint_pos[i]);
            }
        }
        return true;
    }
    /**
     * @brief 
     * 
     * @tparam T floating-point type, double or float.
     * @param enable bool type flag to indicate whether the dual arm are enabled.
     * @param left_hand_position 
     * @param left_hand_orientation 
     * @param right_hand_position 
     * @param right_hand_orientation 
     * @return true 
     * @return false 
     */
    template <typename T>
    bool recvDualArmHandPose(bool& enable,
        std::array<T, 3>& left_hand_position, std::array<T, 4>& left_hand_orientation,
        std::array<T, 3>& right_hand_position, std::array<T, 4>& right_hand_orientation)
    {
        if ( !this->receiver_enabled )
        {
            return false;
        }
        if ( this->recv_mq_.empty() )
        {
            return false;
        }
        whole_body_msg msg;
        this->recv_mq_.dequeue(msg);
        enable = (msg.mask & (1<<15)) && (msg.mask & (1<<13)) && (msg.mask & (1<<12));
        if ( enable )
        {
            left_hand_position[0] = static_cast<T>(msg.left_hand_pos[0]);
            left_hand_position[1] = static_cast<T>(msg.left_hand_pos[1]);
            left_hand_position[2] = static_cast<T>(msg.left_hand_pos[2]);

            left_hand_orientation[0] = static_cast<T>(msg.left_hand_quat[0]);
            left_hand_orientation[1] = static_cast<T>(msg.left_hand_quat[1]);
            left_hand_orientation[2] = static_cast<T>(msg.left_hand_quat[2]);
            left_hand_orientation[3] = static_cast<T>(msg.left_hand_quat[3]);

            right_hand_position[0] = static_cast<T>(msg.right_hand_pos[0]);
            right_hand_position[1] = static_cast<T>(msg.right_hand_pos[1]);
            right_hand_position[2] = static_cast<T>(msg.right_hand_pos[2]);

            right_hand_orientation[0] = static_cast<T>(msg.right_hand_quat[0]);
            right_hand_orientation[1] = static_cast<T>(msg.right_hand_quat[1]);
            right_hand_orientation[2] = static_cast<T>(msg.right_hand_quat[2]);
            right_hand_orientation[2] = static_cast<T>(msg.right_hand_quat[2]);
        }
        return true;
    }

private:

    asio::io_context io_context_;
    std::thread io_context_thread_;
    CommChannel<Mode, WholeBodySender, WholeBodyReceiver> channel_;

    bool sender_enabled = false;
    bool receiver_enabled = false;
    
    MsgQueue send_mq_{RingBuffer<whole_body_msg>{32}};
    MsgQueue recv_mq_{RingBuffer<whole_body_msg>{32}};
};