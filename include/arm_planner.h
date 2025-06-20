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

#include <arm_model.h>


class ArmPlanner
{
public:
    struct WayPoint
    {
        Eigen::Vector<double,ArmModel::num_dof_> joint_pos;
        Eigen::Vector<double,ArmModel::num_dof_> joint_vel;
        Eigen::Vector<double,ArmModel::num_dof_> joint_acc;
    };

    static const unsigned int update_interval_ = 50; // unit: ms.
    static const unsigned int trajectory_length_ = 100;

    ArmPlanner();
    ~ArmPlanner();
    void plan(const Eigen::Vector<double,6>);
    const WayPoint& getWayPoint(unsigned int id) const;

private:
    bool running = false;
    std::atomic<int> active_trajectory_id_;
    std::array<std::array<WayPoint,trajectory_length_>,2> trajectory_buffer_;
};

#endif // __ARM_PLANNER_H__