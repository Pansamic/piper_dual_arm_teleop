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
#include <error_codes.h>
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

    struct StaticMemoryBuffer
    {
        std::array<TrajectoryPoint, Capacity> buffer;
        std::size_t size;
        std::size_t head;

        /* A bool signal to determine whether the coefficients
        * for new waypoints are computed or not. */
        std::atomic<bool> quintic_polynomial_ready;

        /* A set of quintic polynomial coefficients for each joint and each trajectory segment */
        QuinticPolyCoeffs quintic_polynomial_coeffs[ArmModel::num_dof_][Capacity-1];
        /* @todo Add B-Spline coefficients in this struct. */
    };

    TrajectoryBuffer():
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
    ErrorCode interpolate(JointState& joint_state, InterpolationType type, TimePoint query_time) const
    {
        ErrorCode err = OK;

        switch (type)
        {
        case LINEAR:
            err = interpolateLinear(joint_state, query_time);
            break;
        case QUINTIC_POLYNOMIAL:
            err = interpolateQuinticPolynomial(joint_state, query_time);
            break;
        case B_SPLINE:
            err = interpolateBSpline(joint_state, query_time);
            break;
        default:
            err = InvalidArgument;
            break;
        }
        return err;
    };
    /**
     * @brief pre-compute coefficients of quintic polynomial interpolation
     * using the waypoints in `write_buffer_`.
     * @note This method should be called after `push()` and before `commit()`
     * because this method operates on `this->write_buffer_`, not `this->read_buffer_`.
     */
    void computeQuinticPolynomialCoefficients()
    {
        for ( size_t i=0 ; i<Capacity-1 ; i++ )
        {
            std::chrono::duration<double> elapsed_time = this->write_buffer_->buffer.at(i+1).timestamp - this->write_buffer_->buffer.at(i).timestamp;
            double T = elapsed_time.count();
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
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a0 = coeffs(0);
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a1 = coeffs(1);
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a2 = coeffs(2);
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a3 = coeffs(3);
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a4 = coeffs(4);
                this->write_buffer_->quintic_polynomial_coeffs[j][i].a5 = coeffs(5);
                // LOG_INFO("Quintic polynomial coefficients:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                //     coeffs(0), coeffs(1), coeffs(2), coeffs(3), coeffs(4), coeffs(5));
            }
        }
        this->write_buffer_->quintic_polynomial_ready.store(true, std::memory_order_release);
    };
private:

    StaticMemoryBuffer buffer_a_;
    StaticMemoryBuffer buffer_b_;
    StaticMemoryBuffer* write_buffer_;
    std::atomic<StaticMemoryBuffer*> read_buffer_;

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
     * @param joint_state Reference of output joint state.
     * @param waypoints waypoint array, usually from planner function.
     * @retval - OK Compute linear interpolation trajectory point successfully.
     *         - InvalidData Not enough points in trajectory buffer for linear interpolation.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    ErrorCode interpolateLinear(JointState& joint_state, TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        ErrorCode err = OK;

        if (buf->size < 2)
        {
            /* Not enough points for linear interpolation. */
            err = InvalidData;
            return err;
        }

        for (std::size_t i = 0; i < buf->size - 1; ++i)
        {
            const TrajectoryPoint &p0 = at_index(i);
            const TrajectoryPoint &p1 = at_index(i + 1);

            if (p0.timestamp <= query_time && query_time <= p1.timestamp)
            {
                double alpha = std::chrono::duration<double>(query_time - p0.timestamp).count() /
                    std::chrono::duration<double>(p1.timestamp - p0.timestamp).count();

                joint_state.joint_pos  = (1.0 - alpha) * p0.state.joint_pos  + alpha * p1.state.joint_pos;
                joint_state.joint_vel  = (1.0 - alpha) * p0.state.joint_vel  + alpha * p1.state.joint_vel;
                joint_state.joint_acc  = (1.0 - alpha) * p0.state.joint_acc  + alpha * p1.state.joint_acc;
                joint_state.joint_torq = (1.0 - alpha) * p0.state.joint_torq + alpha * p1.state.joint_torq;

                return err;
            }
        }

        if (query_time < at_index(0).timestamp)
        {
            joint_state = at_index(0).state;
            return err;
        }

        joint_state = at_index(buf->size - 1).state;

        return err;
    };

    /**
     * @brief Perform quintic polynomial interpolation on the trajectory waypoints.
     * 
     * @param joint_state Reference of output joint state.
     * @param waypoints waypoint array, usually from planner function.
     * @retval - OK Compute quintic polynomial interpolation trajectory point successfully.
     *         - InvalidData Not enough points in trajectory buffer for quintic polynomial interpolation
     *           or quintic polynomial coefficients haven't been pre-computed by planner.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    ErrorCode interpolateQuinticPolynomial(JointState& joint_state, TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        ErrorCode err = OK;

        if ( buf->size < 2 )
        {
            /* Not enough points for quintic polynomial interpolation. */
            return InvalidData;
        }

        if ( buf->quintic_polynomial_ready.load(std::memory_order_acquire) == false )
        {
            /* Quintic polynomial coefficients are not pre-computed. */
            err = InvalidData;
            return err;
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
            /* Cannot find query timepoint in trajectory waypoints */
            err = InvalidData;
            return err;
        }

        std::chrono::duration<double> elapsed_time = query_time - buf->buffer.at(waypoint_begin_id).timestamp;
        double t1 = elapsed_time.count();
        double t2 = t1 * t1;
        double t3 = t1 * t2;
        double t4 = t1 * t3;
        double t5 = t1 * t4;

        // LOG_INFO("Quintic polynomial ordered time:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}", t1, t2, t3, t4, t5);

        for ( int joint_id=0 ; joint_id<ArmModel::num_dof_ ; joint_id++ )
        {
            const QuinticPolyCoeffs& coeffs = this->write_buffer_->quintic_polynomial_coeffs[joint_id][waypoint_begin_id];
            joint_state.joint_pos(joint_id) =
                coeffs.a0 +
                coeffs.a1 * t1 +
                coeffs.a2 * t2 +
                coeffs.a3 * t3 +
                coeffs.a4 * t4 +
                coeffs.a5 * t5;
            joint_state.joint_vel(joint_id) =
                coeffs.a1 +
                2 * coeffs.a2 * t1 +
                3 * coeffs.a3 * t2 +
                4 * coeffs.a4 * t3 +
                5 * coeffs.a5 * t4;
            joint_state.joint_acc(joint_id) =
                2 * coeffs.a2 + 
                6 * coeffs.a3 * t1 +
                12 * coeffs.a4 * t2 +
                20 * coeffs.a5 * t3;
        }

        return err;
    };
    /**
     * @brief Perform B-spline interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    ErrorCode interpolateBSpline(JointState& joint_state, TimePoint query_time) const
    {
        StaticMemoryBuffer* buf = this->read_buffer_.load(std::memory_order_acquire);

        ErrorCode err = OK;

        if (buf->size < 4)
        {
            /* Not enough points for B-spline interpolation. */
            err = InvalidData;
            return err;
        }

        err = NotImplemented;
        return err;
    };
};
#endif //__TRAJECTORY_BUFFER_H__