/**
 * @file ringbuffer.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Ring buffer
 * @version 0.1
 * @date 2025-06-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __RINGBUFFER_HPP__
#define __RINGBUFFER_HPP__

template<typename T>
class RingBuffer
{
public:
    explicit RingBuffer(size_t capacity):
        head_(0), tail_(0), capacity_(capacity), size_(0), empty_(true), full_(false)
    {
        this->buffer_.resize(capacity);
    }
    ~RingBuffer() = default;
    void push(const T& value)
    {
        std::lock_guard<std::mutex> lock(this->mtx_);

        if ( this->full_ )
        {
            throw std::overflow_error("ring buffer overflow");
        }

        buffer_[tail_] = value;
        tail_ = (tail_ + 1) % capacity_;
        ++size_;

        if (size_ == capacity_)
        {
            full_ = true;
        }
        empty_ = false;
    }
    void pop(T& value)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if ( this->empty_ )
        {
            throw std::underflow_error("ring buffer underflow");
        }

        value = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --size_;

        if (size_ == 0)
        {
            empty_ = true;
        }

        full_ = false;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return empty_;
    }

    bool full() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return full_;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return size_;
    }
private:
    int head_;
    int tail_;
    size_t capacity_;
    size_t size_;
    std::mutex mtx_;
    bool empty_;
    bool full_;
    std::vector<T> buffer_;
};


#endif // __RINGBUFFER_HPP__