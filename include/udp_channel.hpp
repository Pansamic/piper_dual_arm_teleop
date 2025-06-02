#pragma once

#include <concepts>
#include <memory>
#include <typeinfo>
#include <asio.hpp>
#include <asio/steady_timer.hpp>
#include "ringbuf.hpp"
#include <log.h>

template <typename T>
class UdpChannelReceiver
{
public:
    using ReceiverBuffer = RingBuffer<std::shared_ptr<typename T::Definition>>;
    using ReceiverBufferPtr = std::shared_ptr<ReceiverBuffer>;

    UdpChannelReceiver() = delete; // default constructor is deleted

    UdpChannelReceiver(asio::io_context &io_context, std::string local_ip, int local_port, std::string remote_ip, int remote_port):
        local_endpoint_(asio::ip::make_address(local_ip), local_port),
        remote_endpoint_(asio::ip::make_address(remote_ip), remote_port),
        socket_(io_context, asio::ip::udp::v4())
    {
        socket_.bind(local_endpoint_);
        receiver_buffer_ = std::make_shared<ReceiverBuffer>(100);
    }

    void enable_receiver()
    {
        this->socket_.async_receive_from(asio::buffer(receiver_bin_data_), remote_endpoint_,
            std::bind(&UdpChannelReceiver::receiver_handler, this, std::placeholders::_1, std::placeholders::_2));
    }

    auto get_receiver_buffer()
    {
        return this->receiver_buffer_;
    }

private:
    void receiver_handler(const asio::error_code &ec, std::size_t bytes_transferred)
    {
        if (!ec && bytes_transferred > 0)
        {
            std::shared_ptr<typename T::Definition> data = std::make_shared<typename T::Definition>();
            try
            {
                T::decode(this->receiver_bin_data_, *data);
                this->receiver_buffer_->push(data);
            }
            catch (const std::exception &e)
            {
                LOG_ERROR("{} CRC invalid.", typeid(T).name());
            }
            
        }
        this->socket_.async_receive_from(asio::buffer(receiver_bin_data_), remote_endpoint_,
            std::bind(&UdpChannelReceiver::receiver_handler, this, std::placeholders::_1, std::placeholders::_2));
    }

    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint local_endpoint_;
    asio::ip::udp::endpoint remote_endpoint_;
    ReceiverBufferPtr receiver_buffer_;
    std::array<std::byte,T::packet_size> receiver_bin_data_;
};

template <typename T>
class UdpChannelSender
{
public:
    using SenderBuffer = RingBuffer<std::shared_ptr<typename T::Definition>>;
    using SenderBufferPtr = std::weak_ptr<SenderBuffer>;

    UdpChannelSender() = delete; // default constructor is deleted

    UdpChannelSender(asio::io_context &io_context, std::string remote_ip, int remote_port):
        remote_endpoint_(asio::ip::make_address(remote_ip), remote_port),
        socket_(io_context, asio::ip::udp::v4()), timer_(io_context) {}

    void enable_sender()
    {
        if (sender_buffer_.expired())
        {
            throw std::runtime_error("sender_buffer_ is null, cannot enable sender.");
        }
        timer_.async_wait(std::bind(&UdpChannelSender::timer_handler, this, std::placeholders::_1));
    }

    void register_sender_buffer(std::shared_ptr<SenderBuffer> ptr)
    {
        this->sender_buffer_ = ptr;
    }

private:
    void timer_handler(const asio::error_code &ec)
    {
        if (!ec)
        {
            auto tmp_sender_buffer_ = this->sender_buffer_.lock();
            if (!tmp_sender_buffer_->empty())
            {
                auto data = tmp_sender_buffer_->pop();

                T::encode(this->sender_bin_data_, *data);
                this->socket_.async_send_to(asio::buffer(this->sender_bin_data_), this->remote_endpoint_,
                                            [](const asio::error_code &error, std::size_t bytes_transferred) {});
                
            }
        }
        timer_.expires_after(asio::chrono::milliseconds(10));
        timer_.async_wait(std::bind(&UdpChannelSender::timer_handler, this, std::placeholders::_1));
    }

    asio::ip::udp::socket socket_;
    asio::steady_timer timer_;
    asio::ip::udp::endpoint remote_endpoint_;

    SenderBufferPtr sender_buffer_;
    std::array<std::byte,T::packet_size> sender_bin_data_;
};
