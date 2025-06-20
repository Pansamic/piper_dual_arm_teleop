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

ArmPlanner::ArmPlanner():
    active_trajectory_id_(0)
{

}

ArmPlanner::~ArmPlanner()
{

}

void ArmPlanner::plan()
{
    
}

const ArmPlanner::WayPoint& ArmPlanner::getWayPoint(unsigned int id) const
{
    return trajectory_buffer_[active_trajectory_id_][id];
}