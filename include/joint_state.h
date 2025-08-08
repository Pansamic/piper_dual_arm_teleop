/**
 * @file joint_state.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief Joint state structure for robot arm control.
 * @version 0.1
 * @date 2025-06-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

#include <Eigen/Core>

template <typename T, std::size_t NumDof>
struct JointState
{
    /* Joint Position */
    Eigen::Vector<T, NumDof> joint_pos;
    /* Joint Velocity */
    Eigen::Vector<T, NumDof> joint_vel;
    /* Joint Acceleration */
    Eigen::Vector<T, NumDof> joint_acc;
    /* Joint Torque */
    Eigen::Vector<T, NumDof> joint_torq;
};

#endif // __WAYPOINT_H__