/**
 * @file arm_planner.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm trajectory planner.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ARM_PLANNER_H__
#define __ARM_PLANNER_H__

#include <condition_variable>
#include <arm_model.h>

struct WayPoint
{
    Eigen::Vector<double,ArmModel::num_dof_> joint_pos;
    Eigen::Vector<double,ArmModel::num_dof_> joint_vel;
    Eigen::Vector<double,ArmModel::num_dof_> joint_acc;
};

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
    void write(const T& waypoint)
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
    void read(T& waypoint)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if ( this->empty_ )
        {
            throw std::underflow_error("ring buffer underflow")
        }

        waypoint = buffer_[head_];
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

class ArmPlanner
{
public:
    ArmPlanner(size_t update_interval, size_t waypoint_length, size_t trajectory_length);
    ~ArmPlanner() = default;
    void getLeftArmWayPoint(WayPoint& waypoint);
    void getRightArmWayPoint(WayPoint& waypoint);
private:
    bool running = false;
    /* planner loop interval */
    size_t update_interval_;
    /* amount of waypoints in a plan */
    size_t waypoint_length_;
    /* total trajectory length (buffer length) */
    size_t trajectory_length_;
    /* used to mark the end waypoint of last trajectory slice
     * for continuous planning */
    Eigen::Vector<double, ArmModel::num_dof_> left_arm_last_end_waypoint_;
    Eigen::Vector<double, ArmModel::num_dof_> right_arm_last_end_waypoint_;

    RingBuffer<WayPoint> left_arm_trajectory_buffer_;
    RingBuffer<WayPoint> right_arm_trajectory_buffer_;

    void plan(
        const Eigen::Vector<double,ArmModel::num_dof_>& begin,
        const Eigen::Vector<double,ArmModel::num_dof_>& end);
};

#endif // __ARM_PLANNER_H__