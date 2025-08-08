/**
 * @file arm_ik.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic manipulator inverse kinematics implmentation.
 * @version 0.1
 * @date 2025-08-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ARM_IK_H__
#define __ARM_IK_H__

#include <Eigen/Core>
#include <error_codes.h>
#include <piper_model.hpp>

ErrorCode getShoulderJointPos(
    Eigen::Vector3d& shoulder_joint_pos,
    const Eigen::Matrix4d& base_transform,
    const Eigen::Matrix4d& pose,
    const Eigen::Vector3d& ref_conf);
ErrorCode getInverseKinematics(
    Eigen::Vector<double, PiperArmNumDof>& joint_pos,
    const Eigen::Matrix4d& base_transform,
    const Eigen::Matrix4d& pose,
    const Eigen::Vector<double,6>& ref_conf);
ErrorCode getDampedLeastSquareInverseKinematics(
    Eigen::Vector<double, PiperArmNumDof>& joint_pos, 
    const Eigen::Matrix4d& base_transform,
    const double lambda,
    const Eigen::Vector<double,6> tolerance,
    const size_t max_iteration,
    const Eigen::Matrix4d& pose,
    const Eigen::Vector<double,6>& ref_conf);
#endif // __ARM_IK_H__