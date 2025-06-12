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

class ArmController
{
public:
    ArmController(){};
    ~ArmController(){};
    void updateCurrentJointPosition(Eigen::Vector<double,ArmModel::num_dof_> joint_pos);
    void updateCurrentJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateCurrentJointVelocity(Eigen::Vector<double,ArmModel::num_dof_> joint_pos);
    void updateCurrentJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateCurrentJointTorque(Eigen::Vector<double,ArmModel::num_dof_> joint_pos);
    void updateCurrentJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);
private:
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kp(100,100,100,100,100,100);
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kd(20,20,20,20,20,20);
    static const Eigen::Vector3d head_position(0.325024,0,0.80246);
    static const Eigen::Quaterniond left_hand_orientation_offset = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    static const Eigen::Quaterniond right_hand_orientation_offset(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));

    Eigen::Vector3d base_position;
    Eigen::Quaterniond base_orientation;

    static const Eigen::Matrix4d left_arm_base_transform = (Eigen::Matrix4d() << 
        0, -0.707106781 ,  0.707106781 , 0.23805,
        0,  0.707106781 ,  0.707106781 , 0.19675,
        -1, 0           ,  0           , 0.74065,
        0,  0           ,  0           , 1).finished();
    Eigen::Vector3d left_hand_target_pos(0.542092536439244, 0.500792536439244, 0.398868333963670);
    Eigen::Quaterniond left_hand_target_orientation(0.000044497177102,0.382683431921232,-0.923879531439719,0.000018431334243);
    Eigen::Matrix4d left_hand_target_pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond left_hand_actual_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    double left_gripper_control = 0;

    static const Eigen::Matrix4d right_arm_base_transform = (Eigen::Matrix4d() <<
         0, 0.707106781, 0.707106781, 0.23805,
         0, 0.707106781,-0.707106781,-0.19675,
        -1, 0,           0,           0.74065,
         0, 0,           0,           1).finished();
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

};

#endif // __ARM_CONTROLLER_H__