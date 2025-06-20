/**
 * @file arm_controller.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief Joint space controller for trajectory tracking.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ARM_CONTROLLER_H__
#define __ARM_CONTROLLER_H__

#include <Eigen/Core>
#include <arm_interface.h>

class ArmController
{
public:
    ArmController();
    ~ArmController();
    void updateCurrentJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
    void updateCurrentJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateCurrentJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void updateCurrentJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateCurrentJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq);
    void updateCurrentJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);
private:
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kp_;
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kd_;
    bool running_;
    std::unique_ptr<ArmInterface> env;
    Eigen::Vector<double,ArmModel::num_dof_> target_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> target_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> target_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> target_joint_torq_;
    Eigen::Vector<double,ArmModel::num_dof_> actual_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> actual_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> actual_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> actual_joint_torq_;
    void control_loop_();
};

#endif // __ARM_CONTROLLER_H__