/**
 * @file trajectory_buffer.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief Trajectory buffer stores the trajectory points and has a series of trajectory management.
 * @version 0.1
 * @date 2025-07-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TRAJECTORY_BUFFER_H__
#define __TRAJECTORY_BUFFER_H__

#include <vector>
#include <chrono>
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

    TrajectoryBuffer();
    ~TrajectoryBuffer() = default;

    bool push(const TrajectoryPoint &point);
    void clear();
    std::size_t size() const;
    /**
     * @brief Commit the trajectory write operation and interpolation coefficients pre-computation.
     * This function will swap the write and read buffer atomic pointer.
     */
    void commit();
    /**
     * @brief Perform interpolation algorithm with given choice.
     * 
     * @param type `InterpolationType` interpolation type choice.
     * @param query_time time of querying the target joint state.
     * @return JointState the target joint state at time point `query_time`.
     */
    JointState interpolate(InterpolationType type, TimePoint query_time) const;
private:
    std::array<TrajectoryPoint, Capacity> buffer_a_;
    std::array<TrajectoryPoint, Capacity> buffer_b_;
    std::array<TrajectoryPoint, Capacity>* write_buffer_;
    std::array<TrajectoryPoint, Capacity>* read_buffer_;

    std::size_t size_;
    std::size_t head_;
    mutable std::mutex mutex_;

    /* A bool signal to determine whether the coefficients
     * for new waypoints are computed or not. */
    std::atomic<bool> quintic_polynomial_coefficients_ready_;
    /* A set of quintic polynomial coefficients for each joint and each trajectory segment */
    QuinticPolyCoeffs quintic_polynomial_coefficients_[ArmModel::num_dof_][Capacity-1];

    std::size_t index(std::size_t i) const;
    const TrajectoryPoint &at_index(std::size_t i) const;

    /**
     * @brief Perform linear interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateLinear(TimePoint query_time) const;
    /**
     * @brief pre-compute coefficients of quintic polynomial interpolation
     * using the waypoints in `buffer_`.
     */
    void computeQuinticPolynomialCoefficients();
    /**
     * @brief Perform quintic polynomial interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateQuinticPolynomial(TimePoint query_time) const;
    /**
     * @brief Perform B-spline interpolation on the trajectory waypoints.
     * 
     * @param waypoints waypoint array, usually from planner function.
     * 
     * @note Write interpolated trajectory to `this->left_arm_trajectory_buffer_` or
     * `this->right_arm_trajectory_buffer_`
     */
    JointState interpolateBSpline(TimePoint query_time) const;
};
#endif //__TRAJECTORY_BUFFER_H__