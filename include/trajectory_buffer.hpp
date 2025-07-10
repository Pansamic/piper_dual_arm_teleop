/**
 * @file trajectory_buffer.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Trajectory buffer stores the trajectory points and has a series of trajectory management.
 * @version 0.1
 * @date 2025-07-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TRAJECTORY_BUFFER_HPP__
#define __TRAJECTORY_BUFFER_HPP__
#pragma once
#include <iostream>
#include <vector>
#include <chrono>
#include <log.hpp>
#include <joint_state.h>

template <std::size_t Capacity = 10>
class TrajectoryBuffer
{
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    struct TrajectoryPoint
    {
        TimePoint timestamp;
        JointState state;
    };

    struct StaticMemoryBuffer
    {
        std::array<TrajectoryPoint, Capacity> buffer;
        std::size_t size;
        std::size_t head;
    };

    enum InterpolationType
    {
        LINEAR,
        QUINTIC_POLYNOMIAL,
        B_SPLINE
    };

    struct QuinticPolyCoeffs
    {
        double a0;
        double a1;
        double a2;
        double a3;
        double a4;
        double a5;
    };

    TrajectoryBuffer():
        quintic_polynomial_coefficients_ready_(false),
        write_buffer_(&this->buffer_a_), read_buffer_(&this->buffer_b_)
    {
        this->buffer_a_.head = 0;
        this->buffer_a_.size = 0;
        this->buffer_b_.head = 0;
        this->buffer_b_.size = 0;
    };

    ~TrajectoryBuffer() = default;

    bool push(const TrajectoryPoint &point)
    {
        if ( this->write_buffer_->head >= Capacity )
        {
            return false;
        }
        this->write_buffer_->buffer.at(this->write_buffer_->head) = point;
        this->write_buffer_->head = (this->write_buffer_->head + 1) % Capacity;
        this->write_buffer_->size++;

        return true;
    };

    void clear()
    {
        this->write_buffer_ = &this->buffer_a_;
        this->write_buffer_->head = 0;
        this->write_buffer_->size = 0;
        this->read_buffer_ = &this->buffer_b_;
        this->read_buffer_->head = 0;
        this->read_buffer_->size = 0;
    }

    std::size_t size() const
    {
        return this->read_buffer_->size;
    };
    /**
     * @brief Commit the trajectory write operation and interpolation coefficients pre-computation.
     * This function will swap the write and read buffer atomic pointer.
     */
    void commit()
    {
        // Memory barrier: ensure all writes are visible before swapping
        std::atomic_thread_fence(std::memory_order_release);

        // Atomically swap reader pointer
        this->read_buffer_.store(this->write_buffer_, std::memory_order_release);

        // Now use the old read buffer as the next write buffer
        this->write_buffer_ = (this->write_buffer_ == &this->buffer_a_) ? &this->buffer_b_ : &this->buffer_a_;

        this->write_buffer_->head = 0;
        this->write_buffer_->size = 0;
    };
    /**
     * @brief Perform interpolation algorithm with given choice.
     * 
     * @param type `InterpolationType` interpolation type choice.
     * @param query_time time of querying the target joint state.
     * @return JointState the target joint state at time point `query_time`.
     */
    JointState interpolate(InterpolationType type, TimePoint query_time) const
    {
        JointState joint_state;

        switch (type)
        {
        case LINEAR:
            try
            {
                joint_state = interpolateLinear(query_time);
            }
            catch(const std::exception& e)
            {
                LOG_WARN(e.what());
            }
            return joint_state;
        case QUINTIC_POLYNOMIAL:
            try
            {
                joint_state = interpolateQuinticPolynomial(query_time);
            }
            catch(const std::exception& e)
            {
                LOG_WARN(e.what());
            }
            return joint_state;
        case B_SPLINE:
            try
            {
                joint_state = interpolateBSpline(query_time);
            }
            catch(const std::exception& e)
            {
                LOG_WARN(e.what());
            }
            return joint_state;
        default:
            throw std::invalid_argument("Unknown interpolation type");
        }
    };
    /**
     * @brief pre-compute coefficients of quintic polynomial interpolation
     * using the waypoints in `buffer_`.
     * @note This method should be called after `push()` and before `commit()`
     * because this method operates on `this->write_buffer_`, not `this->read_buffer_`.
     */
    void computeQuinticPolynomialCoefficients()
    {
        for ( size_t i=0 ; i<Capacity-1 ; i++ )
        {
            double T = (this->write_buffer_->buffer.at(i+1).timestamp - this->write_buffer_->buffer.at(i).timestamp).count();
            double T2 = T * T;
            double T3 = T * T2;
            double T4 = T * T3;
            double T5 = T * T4;

            Eigen::Matrix<double,6,6> M = Eigen::Matrix<double,6,6>::Zero();
            M(0,0) = 1;
            M(1,0) = 1;
            M(1,1) = T;
            M(1,2) = T2;
            M(1,3) = T3;
            M(1,4) = T4;
            M(1,5) = T5;
            M(2,1) = 1;
            M(3,1) = 1;
            M(3,2) = 2*T;
            M(3,3) = 3*T2;
            M(3,4) = 4*T3;
            M(3,5) = 5*T4;
            M(4,2) = 2;
            M(5,2) = 2;
            M(5,3) = 6*T;
            M(5,4) = 12*T2;
            M(5,5) = 20*T3;

            Eigen::Matrix<double,6,6> M_inv = M.inverse();

            for ( int j=0 ; j<ArmModel::num_dof_ ; j++ )
            {
                Eigen::Vector<double,6> V(
                    this->write_buffer_->buffer.at(i).state.joint_pos(j),
                    this->write_buffer_->buffer.at(i+1).state.joint_pos(j),
                    this->write_buffer_->buffer.at(i).state.joint_vel(j),
                    this->write_buffer_->buffer.at(i+1).state.joint_vel(j),
                    this->write_buffer_->buffer.at(i).state.joint_acc(j),
                    this->write_buffer_->buffer.at(i+1).state.joint_acc(j)
                );
                Eigen::Vector<double,6> coeffs = M_inv * V;
                this->quintic_polynomial_coefficients_[j][i].a0 = coeffs(0);
                this->quintic_polynomial_coefficients_[j][i].a1 = coeffs(1);
                this->quintic_polynomial_coefficients_[j][i].a2 = coeffs(2);
                this->quintic_polynomial_coefficients_[j][i].a3 = coeffs(3);
                this->quintic_polynomial_coefficients_[j][i].a4 = coeffs(4);
                this->quintic_polynomial_coefficients_[j][i].a5 = coeffs(5);
            }
        }
        this->quintic_polynomial_coefficients_ready_.store(true, std::memory_order_release);
    };
private:

    StaticMemoryBuffer buffer_a_;
    StaticMemoryBuffer buffer_b_;
    StaticMemoryBuffer* write_buffer_;
    std::atomic<StaticMemoryBuffer*> read_buffer_;

    /* A bool signal to determine whether the coefficients
     * for new waypoints are computed or not. */
    std::atomic<bool> quintic_polynomial_coefficients_ready_;
    /* A set of quintic polynomial coefficients for each joint and each trajectory segment */
    QuinticPolyCoeffs quintic_polynomial_coefficients_[ArmModel::num_dof_][Capacity-1];

    std::size_t index(std::size_t i) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);
        return (buf->head + Capacity - buf->size + i) % Capacity;
    };
    const TrajectoryPoint &at_index(std::size_t i) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);
        return buf->buffer.at(index(i));
    };
    /**
     * @brief Perform linear interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateLinear(TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        if (buf->size < 2)
        {
            throw std::runtime_error("Not enough points for linear interpolation");
        }

        for (std::size_t i = 0; i < buf->size - 1; ++i)
        {
            const TrajectoryPoint &p0 = at_index(i);
            const TrajectoryPoint &p1 = at_index(i + 1);

            if (p0.timestamp <= query_time && query_time <= p1.timestamp)
            {
                double alpha = std::chrono::duration<double>(query_time - p0.timestamp).count() /
                    std::chrono::duration<double>(p1.timestamp - p0.timestamp).count();

                JointState result;
                result.joint_pos  = (1.0 - alpha) * p0.state.joint_pos  + alpha * p1.state.joint_pos;
                result.joint_vel  = (1.0 - alpha) * p0.state.joint_vel  + alpha * p1.state.joint_vel;
                result.joint_acc  = (1.0 - alpha) * p0.state.joint_acc  + alpha * p1.state.joint_acc;
                result.joint_torq = (1.0 - alpha) * p0.state.joint_torq + alpha * p1.state.joint_torq;
                return result;
            }
        }

        if (query_time < at_index(0).timestamp)
        {
            return at_index(0).state;
        }

        return at_index(buf->size - 1).state;
    };

    /**
     * @brief Perform quintic polynomial interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateQuinticPolynomial(TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        if ( buf->size < 2 )
        {
            throw std::runtime_error("Not enough points for quintic interpolation");
        }

        if ( this->quintic_polynomial_coefficients_ready_ == false )
        {
            throw std::runtime_error("quintic polynomial coefficients are not pre-computed");
        }

        size_t waypoint_begin_id = 0;
        size_t waypoint_end_id = 0;

        /* Find the nearest two waypoints in buffer. */
        for ( size_t i=0 ; i<buf->size ; i++ )
        {
            if ( buf->buffer.at(i).timestamp <= query_time )
            {
                waypoint_begin_id = i;
            }
            if ( buf->buffer.at(i).timestamp > query_time )
            {
                waypoint_end_id = i;
                break;
            }
        }

        if ( waypoint_begin_id == 0 && waypoint_end_id == 0 )
        {
            LOG_INFO("Query time:{:d}. Trajectory time span:{:d}~{:d}",
                std::chrono::duration_cast<std::chrono::nanoseconds>(query_time.time_since_epoch()).count(),
                std::chrono::duration_cast<std::chrono::nanoseconds>(buf->buffer.at(0).timestamp.time_since_epoch()).count(),
                std::chrono::duration_cast<std::chrono::nanoseconds>(buf->buffer.at(buf->size-1).timestamp.time_since_epoch()).count());
            throw std::runtime_error("cannot find query timepoint in trajectory waypoints");
        }

        JointState interpolation;

        double t1 = (query_time - buf->buffer.at(waypoint_begin_id).timestamp).count();
        double t2 = t1 * t1;
        double t3 = t1 * t2;
        double t4 = t1 * t3;
        double t5 = t1 * t4;

        for ( int joint_id=0 ; joint_id<ArmModel::num_dof_ ; joint_id++ )
        {
            const QuinticPolyCoeffs& coeffs = this->quintic_polynomial_coefficients_[joint_id][waypoint_begin_id];
            interpolation.joint_pos(joint_id) =
                coeffs.a0 +
                coeffs.a1 * t1 +
                coeffs.a2 * t2 +
                coeffs.a3 * t3 +
                coeffs.a4 * t4 +
                coeffs.a5 * t5;
            interpolation.joint_vel(joint_id) =
                coeffs.a1 +
                2 * coeffs.a2 * t1 +
                3 * coeffs.a3 * t2 +
                4 * coeffs.a4 * t3 +
                5 * coeffs.a5 * t4;
            interpolation.joint_acc(joint_id) =
                2 * coeffs.a2 + 
                6 * coeffs.a3 * t1 +
                12 * coeffs.a4 * t2 +
                20 * coeffs.a5 * t3;
        }

        return interpolation;
    };
    /**
     * @brief Perform B-spline interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateBSpline(TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        if (buf->size < 4)
        {
            throw std::runtime_error("Not enough points for B-spline interpolation");
        }

        throw std::runtime_error("B-spline interpolation not yet implemented");
    };
};
#endif //__TRAJECTORY_BUFFER_H__