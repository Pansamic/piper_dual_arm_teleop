/**
 * @file arm_controller.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Teleoperation controller.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <Eigen/Core>
#include <arm_model.h>
#include <arm_controller.h>

const Eigen::Vector<double,ArmModel::num_dof_> ArmController::joint_kp_ = Eigen::Vector<double,ArmModel::num_dof_>(100,100,100,100,100,100);

const Eigen::Vector<double,ArmModel::num_dof_> ArmController::joint_kd_ = Eigen::Vector<double,ArmModel::num_dof_>(20,20,20,20,20,20);

ArmController::ArmController()
{

}

ArmController::~ArmController()
{

}

