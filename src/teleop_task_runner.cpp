/**
 * @file teleop_task_runner.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Task runner for dual arm teleoperation.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <teleop_task_runner.h>

const Eigen::Vector3d TeleopTaskRunner::head_position = Eigen::Vector3d(0.325024,0,0.80246);
const Eigen::Quaterniond TeleopTaskRunner::left_hand_orientation_offset = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
const Eigen::Quaterniond TeleopTaskRunner::right_hand_orientation_offset(
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));
const Eigen::Matrix4d TeleopTaskRunner::left_arm_base_transform = (Eigen::Matrix4d() << 
    0, -0.707106781 ,  0.707106781 , 0.23805,
    0,  0.707106781 ,  0.707106781 , 0.19675,
    -1, 0           ,  0           , 0.74065,
    0,  0           ,  0           , 1).finished();
const Eigen::Matrix4d TeleopTaskRunner::right_arm_base_transform = (Eigen::Matrix4d() <<
        0, 0.707106781, 0.707106781, 0.23805,
        0, 0.707106781,-0.707106781,-0.19675,
    -1, 0,           0,           0.74065,
        0, 0,           0,           1).finished();

TeleopTaskRunner::TeleopTaskRunner()
{
    this->left_hand_target_pos_ = Eigen::Vector3d(0.542092536439244, 0.500792536439244, 0.398868333963670),
    this->left_hand_target_orientation_ = Eigen::Quaterniond(0.000044497177102,0.382683431921232,-0.923879531439719,0.000018431334243);
    this->left_hand_target_pose = Eigen::Matrix4d::Identity();
    this->left_hand_actual_orientation = Eigen::Quaterniond::Identity();
    
}

TeleopTaskRunner::~TeleopTaskRunner()
{

}

void TeleopTaskRunner::run()
{

}

