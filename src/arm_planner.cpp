/**
 * @file arm_planner.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm trajectory planner.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <log.hpp>
#include <arm_planner.h>

ArmPlanner::ArmPlanner(
    TrajectoryBuffer<num_plan_waypoint_>& left_arm_trajectory_buffer,
    TrajectoryBuffer<num_plan_waypoint_>& right_arm_trajectory_buffer,
    size_t freq_plan):
    dt_plan_(1.0/static_cast<double>(freq_plan)),
    left_arm_trajectory_buffer_(left_arm_trajectory_buffer),
    right_arm_trajectory_buffer_(right_arm_trajectory_buffer),
    plan_thread_(std::thread(&ArmPlanner::threadPlan, this)), running_(true)
{

}

void ArmPlanner::stop()
{
    this->running_.store(false, std::memory_order_release);
    if ( this->plan_thread_.joinable() )
    {
        plan_thread_.join();  // Ensure thread finishes before destruction
    }
}

void ArmPlanner::setLeftArmTargetJointState(const JointState& joint_state)
{
    std::lock_guard<std::mutex> lock(this->left_arm_target_joint_state_mtx_);
    this->left_arm_target_joint_state_ = joint_state;
}

void ArmPlanner::setRightArmTargetJointState(const JointState& joint_state)
{
    std::lock_guard<std::mutex> lock(this->right_arm_target_joint_state_mtx_);
    this->right_arm_target_joint_state_ = joint_state;
}

void ArmPlanner::planDualArmLinear(
    const std::chrono::steady_clock::time_point& start_timepoint,
    const JointState& left_arm_begin,
    const JointState& left_arm_end,
    const JointState& right_arm_begin,
    const JointState& right_arm_end)
{
    /* Convert the interval of plan loop to std::chrono duration type at the first run. */
    // static std::chrono::steady_clock::duration plan_duration(static_cast<int64_t>(this->dt_plan_*1e9));
    static std::chrono::steady_clock::duration plan_duration = 
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(this->dt_plan_/this->num_plan_waypoint_));

    /* Create a trajectory waypoint */
    TrajectoryBuffer<this->num_plan_waypoint_>::TrajectoryPoint waypoint;

    /* zero velocities/accelerations/torque here, to be filled by interpolation */
    waypoint.state.joint_vel.setZero();
    waypoint.state.joint_acc.setZero();
    waypoint.state.joint_torq.setZero();

    waypoint.timestamp = start_timepoint;

    // fill internal_waypoints_[0..N-1]
    for ( size_t i = 0; i < this->num_plan_waypoint_; ++i )
    {
        double alpha = double(i+1) / (this->num_plan_waypoint_ + 1);

        /* Compute left arm waypoint. */
        waypoint.state.joint_pos = left_arm_begin.joint_pos + alpha * (left_arm_end.joint_pos - left_arm_begin.joint_pos);

        /* Push waypoint to left arm trajectory buffer. */
        this->left_arm_trajectory_buffer_.push(waypoint);

        LOG_DEBUG("Left arm linear plan waypoint({:d}):{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",i,
            waypoint.state.joint_pos(0),waypoint.state.joint_pos(1),waypoint.state.joint_pos(2),
            waypoint.state.joint_pos(3),waypoint.state.joint_pos(4),waypoint.state.joint_pos(5));

        /* Compute right arm waypoint. */
        waypoint.state.joint_pos = right_arm_begin.joint_pos + alpha * (right_arm_end.joint_pos - right_arm_begin.joint_pos);

        /* Push waypoint to right arm trajectory buffer. */
        this->right_arm_trajectory_buffer_.push(waypoint);

        LOG_DEBUG("Right arm linear plan waypoint({:d}):{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",i,
            waypoint.state.joint_pos(0),waypoint.state.joint_pos(1),waypoint.state.joint_pos(2),
            waypoint.state.joint_pos(3),waypoint.state.joint_pos(4),waypoint.state.joint_pos(5));
        
        waypoint.timestamp += plan_duration;
    }
    /* Pre-compute the coefficients of quintic polynomial to reduce 
     * the computation resource comsumption due to the high frequency of
     * joint controller. */
    this->left_arm_trajectory_buffer_.computeQuinticPolynomialCoefficients();

    /* Commit the pre-computation operation to swap the ping-pong buffer in the trajectory buffer. */
    this->left_arm_trajectory_buffer_.commit();

    this->right_arm_trajectory_buffer_.computeQuinticPolynomialCoefficients();
    this->right_arm_trajectory_buffer_.commit();
}

void ArmPlanner::threadPlan()
{
    auto increase_time_spec = [](struct timespec* time, const struct timespec* increasement)
    {
        time->tv_sec += increasement->tv_sec;
        time->tv_nsec += increasement->tv_nsec;

        if (time->tv_nsec >= 1000000000)
        {
            time->tv_sec++;
            time->tv_nsec -= 1000000000;
        }
    };

    struct timespec wakeup_time = {0,0};
    static struct timespec cycletime = {0, static_cast<long>(this->dt_plan_ * 1e9)};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while ( this->running_.load(std::memory_order_acquire) )
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        // LOG_INFO("Planner time: {:d}", std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());

        std::lock_guard<std::mutex> left_arm_lock(this->left_arm_target_joint_state_mtx_);
        std::lock_guard<std::mutex> right_arm_lock(this->right_arm_target_joint_state_mtx_);

        /* Use linear plan to generate internal waypoints */
        this->planDualArmLinear(
            std::chrono::steady_clock::now(),
            this->left_arm_last_target_joint_state_, this->left_arm_target_joint_state_,
            this->right_arm_last_target_joint_state_, this->right_arm_target_joint_state_);

        /* Update history joint state */
        this->left_arm_last_target_joint_state_ = this->left_arm_target_joint_state_;
        this->right_arm_last_target_joint_state_ = this->right_arm_target_joint_state_;
    }
}