/**
 * @file teleop_task_runner.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief Task runner for dual arm teleoperation.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TELEOP_TASK_RUNNER_H__
#define __TELEOP_TASK_RUNNER_H__

#include <Eigen/Core>
#include <arm_model.h>
#include <arm_controller.h>
#include <arm_planner.h>

class TeleopTaskRunner
{
public:
    TeleopTaskRunner();
    ~TeleopTaskRunner();
    void run();
private:
    static const Eigen::Vector3d head_position_;
    static const Eigen::Quaterniond left_hand_orientation_offset_;
    static const Eigen::Quaterniond right_hand_orientation_offset_;
    static const Eigen::Matrix4d left_arm_base_transform_;
    static const Eigen::Matrix4d right_arm_base_transform_;

    Eigen::Vector3d left_hand_target_pos_;
    Eigen::Quaterniond left_hand_target_orientation_;
    Eigen::Matrix4d left_hand_target_pose_;
    Eigen::Quaterniond left_hand_actual_orientation_;
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_pos_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_vel_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_torque_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_pos_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_vel_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_torque_ = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    double left_gripper_control = 0;

    Eigen::Vector3d right_hand_target_pos(0.542092536439244,-0.500792536439244,0.398868333963670);
    Eigen::Quaterniond right_hand_target_orientation(0.000044497177102,-0.382683431921232,-0.923879531439719,-0.000018431334243);
    Eigen::Matrix4d right_hand_target_pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond right_hand_actual_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    double right_gripper_control = 0;

    std::array<Eigen::Matrix4d,ArmModel::num_link_> link_transform;
    std::array<Eigen::Matrix4d,ArmModel::num_link_> link_com_transform;
    std::array<Eigen::Matrix<double,6,ArmModel::num_dof_>,ArmModel::num_link_> link_com_jacobian;
    std::array<Eigen::Matrix<double,6,ArmModel::num_dof_>,ArmModel::num_link_> link_com_jacobian_dot;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_lin_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_ang_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_com_lin_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_com_ang_vel;
    Eigen::Matrix<double,ArmModel::num_dof_,ArmModel::num_dof_> generalized_mass_matrix;
    Eigen::Matrix<double,ArmModel::num_dof_,ArmModel::num_dof_> centrifugal_coriolis_matrix;
    Eigen::Vector<double,ArmModel::num_dof_> gravity_compensate;

    std::unique_ptr<ArmModel> left_arm = std::make_unique<ArmModel>(left_arm_base_transform);
    std::unique_ptr<ArmModel> right_arm = std::make_unique<ArmModel>(right_arm_base_transform);

    std::function<void(Eigen::Vector<double,6>)> getActualJointPosition_;
    std::function<void(Eigen::Vector<double,6>)> getActualJointVelocity_;
    std::function<void(Eigen::Vector<double,6>)> getActualJointTorque_;
    std::function<void()> setJointPosition_;
    std::function<void()> setJointVelocity_;
    std::function<void()> setJointTorque_;
    std::function<void()> setJointBiasTorque_;
};

#endif // __TELEOP_TASK_RUNNER_H__