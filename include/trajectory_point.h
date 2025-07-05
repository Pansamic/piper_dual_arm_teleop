/**
 * @file trajectory_point.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief Trajectory point definition.
 * @version 0.1
 * @date 2025-07-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TRAJECTORY_POINT_H__
#define __TRAJECTORY_POINT_H__

#include <joint_state.h>

struct TrajectoryPoint
{
    unsigned long long timestamp;
    JointState joint_state;
};

#endif // __TRAJECTORY_POINT_H__