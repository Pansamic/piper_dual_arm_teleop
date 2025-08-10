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
    bool send(bool enable, const std::array<T, 6>& left_arm_joint_pos, const std::array<T, 6>& right_arm_joint_pos)
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
        msg.left_arm_joint_pos = left_arm_joint_pos;
        msg.right_arm_joint_pos = right_arm_joint_pos;
        this->send_mq_.enqueue(msg);
        return true;
    }

    template <typename T>
    bool recv(bool& enable, std::array<T, 6>& left_arm_joint_pos, std::array<T, 6>& right_arm_joint_pos)
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
            left_arm_joint_pos = msg.left_arm_joint_pos;
            right_arm_joint_pos = msg.right_arm_joint_pos;
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