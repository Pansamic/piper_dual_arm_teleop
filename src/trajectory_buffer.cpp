/**
 * @file trajectory_buffer.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Trajectory buffer stores the trajectory points and has a series of trajectory management.
 * @version 0.1
 * @date 2025-07-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <stdexcept>
#include <cassert>

#include <trajectory_buffer.h>

template <std::size_t Capacity>
TrajectoryBuffer<Capacity>::TrajectoryBuffer():
    size_(0), head_(0), quintic_polynomial_coefficients_ready_(false),
    write_buffer_(&this->buffer_a_), read_buffer_(&this->buffer_b_)
{

}

template <std::size_t Capacity>
bool TrajectoryBuffer<Capacity>::push(const TrajectoryPoint &point)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if ( this->head_ >= Capacity )
    {
        return false;
    }
    this->buffer_[this->head_] = point;
    this->head_ = (this->head_ + 1) % Capacity;
    if (this->size_ < Capacity)
    {
        this->size_++;
    }
    this->quintic_polynomial_coefficients_ready_.store(false, std::memory_order_release);
    return true;
}

template <std::size_t Capacity>
void TrajectoryBuffer<Capacity>::clear()
{
    this->write_buffer_ = &this->buffer_a_;
    this->read_buffer_ = &this->buffer_b_;
    this->head_ = 0;
    this->size_ = 0;
}

template <std::size_t Capacity>
std::size_t TrajectoryBuffer<Capacity>::size() const
{
    return this->size_;
}

template <std::size_t Capacity>
void TrajectoryBuffer<Capacity>::commit()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    std::swap(this->write_buffer_, this->read_buffer_);
    this->head_ = 0;
    this->size_ = 0;
}

template <std::size_t Capacity>
JointState TrajectoryBuffer<Capacity>::interpolate(InterpolationType type, TimePoint query_time) const
{
    switch (type)
    {
        case LINEAR:
            return interpolateLinear(query_time);
        case QUINTIC_POLYNOMIAL:
            return interpolateQuinticPolynomial(query_time);
        case B_SPLINE:
            return interpolateBSpline(query_time);
        default:
            throw std::invalid_argument("Unknown interpolation type");
    }
}

template <std::size_t Capacity>
std::size_t TrajectoryBuffer<Capacity>::index(std::size_t i) const
{
    return (this->head_ + Capacity - this->size_ + i) % Capacity;
}

template <std::size_t Capacity>
const typename TrajectoryBuffer<Capacity>::TrajectoryPoint &TrajectoryBuffer<Capacity>::at_index(std::size_t i) const
{
    return this->buffer_[index(i)];
}

template <std::size_t Capacity>
JointState TrajectoryBuffer<Capacity>::interpolateLinear(TimePoint query_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (this->size_ < 2)
    {
        throw std::runtime_error("Not enough points for linear interpolation");
    }

    for (std::size_t i = 0; i < this->size_ - 1; ++i)
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

    return at_index(this->size_ - 1).state;
}

template <std::size_t Capacity>
void TrajectoryBuffer<Capacity>::computeQuinticPolynomialCoefficients()
{
    for ( size_t i=0 ; i<Capacity-1 ; i++ )
    {
        double T = (this->buffer_[i+1].timestamp - this->buffer_[i].timestamp).count();
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
                this->buffer_[i].joint_state.joint_pos(j),
                this->buffer_[i+1].joint_state.joint_pos(j),
                this->buffer_[i].joint_state.joint_vel(j),
                this->buffer_[i+1].joint_state.joint_vel(j),
                this->buffer_[i].joint_state.joint_acc(j),
                this->buffer_[i+1].joint_state.joint_acc(j)
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
}

template <std::size_t Capacity>
JointState TrajectoryBuffer<Capacity>::interpolateQuinticPolynomial(TimePoint query_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if ( this->size_ < 2 )
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
    for ( size_t i=0 ; i<this->read_buffer_->size() ; i++ )
    {
        if ( this->read_buffer_[i].timestamp <= query_time )
        {
            waypoint_begin_id = i;
        }
        if ( this->read_buffer_[i].timestamp > query_time )
        {
            waypoint_end_id = i;
            break;
        }
    }

    if ( waypoint_begin_id == 0 && waypoint_end_id == 0 )
    {
        throw std::runtime_error("cannot find query timepoint in trajectory waypoints");
    }

    JointState interpolation;

    double t1 = (query_time - this->read_buffer_[waypoint_begin_id].timestamp).count();
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
}

template <std::size_t Capacity>
JointState TrajectoryBuffer<Capacity>::interpolateBSpline(TimePoint query_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (this->size_ < 4)
    {
        throw std::runtime_error("Not enough points for B-spline interpolation");
    }

    throw std::runtime_error("B-spline interpolation not yet implemented");
}
