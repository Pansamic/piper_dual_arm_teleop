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
#include <arm_planner.h>

ArmPlanner::ArmPlanner(size_t update_interval, size_t waypoint_length, size_t trajectory_length):
    update_interval_(update_interval), waypoint_length_(waypoint_length), trajectory_length_(trajectory_length),
    left_arm_trajectory_buffer_(trajectory_length), right_arm_trajectory_buffer_(trajectory_length),
    left_arm_last_end_waypoint_(Eigen::Vector<double, ArmModel::num_dof_>::Zero()),
    right_arm_last_end_waypoint_(Eigen::Vector<double, ArmModel::num_dof_>::Zero())
{

}

void ArmPlanner::plan(
    const Eigen::Vector<double,ArmModel::num_dof_>& begin,
    const Eigen::Vector<double,ArmModel::num_dof_>& end)
{
    
}

void ArmPlanner::getLeftArmWayPoint(WayPoint& waypoint)
{

    if ( this->left_arm_trajectory_buffer_.empty() )
    {
        throw std::underflow_error("trajectory empty");
    }
    this->left_arm_trajectory_buffer_.read(waypoint);
}

void ArmPlanner::getRightArmWayPoint(WayPoint& waypoint)
{

    if ( this->right_arm_trajectory_buffer_.empty() )
    {
        throw std::underflow_error("trajectory empty");
    }
    this->right_arm_trajectory_buffer_.read(waypoint);
}