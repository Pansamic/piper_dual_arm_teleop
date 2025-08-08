/**
 * @file arm_ik.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic manipulator inverse kinematics implmentation.
 * @version 0.1
 * @date 2025-08-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <piper_model.hpp>
#include <arm_ik.h>
#include <error_codes.h>

ErrorCode getShoulderJointPos(
    Eigen::Vector3d& shoulder_joint_pos,
    const Eigen::Matrix4d &base_transform,
    const Eigen::Matrix4d &pose,
    const Eigen::Vector3d& ref_conf)
{
    ErrorCode err = OK;

    Eigen::Matrix<double, 3, 4> possible_joint_pos;

    int result_count = 0;

    double d1 = 0.123;
    double alpha2 = -M_PI / 2;
    double a3 = 0.28503;
    double d4 = 0.25075;
    double a4 = -0.02198;
    double alpha5 = -M_PI / 2;
    double d6 = 0.091;
    double alpha6 = M_PI / 2;

    // Inverse of base transform matrix
    Eigen::Matrix4d base_transform_inv = Eigen::Matrix4d::Identity();
    base_transform_inv.block<3, 3>(0, 0) = base_transform.block<3, 3>(0, 0).transpose();
    base_transform_inv.block<3, 1>(0, 3) = -base_transform_inv.block<3, 3>(0, 0) * base_transform.block<3, 1>(0, 3);

    // Pose in arm base frame
    Eigen::Matrix4d pose_base = base_transform_inv * pose;

    // Wrist position in arm base frame
    Eigen::Vector3d pw0 = pose_base.block<3, 1>(0, 3) - d6 * pose_base.block<3, 1>(0, 2);

    Eigen::Vector2d theta1;
    Eigen::Vector2d theta2;
    Eigen::Vector2d theta3;

    theta1(0) = atan2(pw0(1), pw0(0));

    if (theta1(0) > 0)
    {
        theta1(1) = theta1(0) - M_PI;
    }
    else
    {
        theta1(1) = theta1(0) + M_PI;
    }

    for (int i = 0; i < 2; ++i)
    {
        // Transform matrix from arm base frame to link 1 frame
        Eigen::Matrix4d T01;
        T01 << cos(theta1(i)), -sin(theta1(i)), 0, 0,
               sin(theta1(i)), cos(theta1(i)), 0, 0,
               0, 0, 1, d1,
               0, 0, 0, 1;
        Eigen::Matrix4d T01_inv = Eigen::Matrix4d::Identity();
        T01_inv.block<3, 3>(0, 0) = T01.block<3, 3>(0, 0).transpose();
        T01_inv.block<3, 1>(0, 3) = -T01_inv.block<3, 3>(0, 0) * T01.block<3, 1>(0, 3);

        // Wrist pose transformation matrix in link 1 frame
        Eigen::Vector4d pos_wrist_1 = T01_inv * Eigen::Vector4d(pw0(0),pw0(1),pw0(2),1);
        
        // X, Y, Z position of wrist in link 1 frame
        double xw1 = pos_wrist_1(0);
        double zw1 = pos_wrist_1(2);

        double temp1 = (xw1 * xw1 + zw1 * zw1 - d4 * d4 - a4 * a4 + a3 * a3) / (2 * zw1);
        double temp2 = xw1 / zw1;

        double delta = temp1 * temp1 * temp2 * temp2 - (1 + temp2 * temp2) * (temp1 * temp1 - a3 * a3);
        if (delta < 0)
        {
            if (delta > 1e-5)
            {
                delta = 0;
            }
            else
            {
                /* no wrist position solution. */
                err = NoResult;
                return err;
            }
        }

        double x31_1 = (temp1 * temp2 + sqrt(delta)) / (1 + temp2 * temp2);
        double x31_2 = (temp1 * temp2 - sqrt(delta)) / (1 + temp2 * temp2);

        double z31_1 = temp1 - temp2 * x31_1;
        double z31_2 = temp1 - temp2 * x31_2;

        theta2(0) = -atan2(z31_1, x31_1);
        theta2(1) = -atan2(z31_2, x31_2);

        double temp3 = atan2(d4, -a4);
        double temp4 = acos(std::max(-1.0, std::min(1.0, (a4 * a4 + d4 * d4 + x31_1 * x31_1 + z31_1 * z31_1 - xw1 * xw1 - zw1 * zw1) /
                                        (2 * sqrt(a4 * a4 + d4 * d4) * sqrt(x31_1 * x31_1 + z31_1 * z31_1)))));
        double temp5 = acos(std::max(-1.0, std::min(1.0, (a4 * a4 + d4 * d4 + x31_2 * x31_2 + z31_2 * z31_2 - xw1 * xw1 - zw1 * zw1) /
                                        (2 * sqrt(a4 * a4 + d4 * d4) * sqrt(x31_2 * x31_2 + z31_2 * z31_2)))));

        double theta3_1 = -temp3 - temp4;
        double theta3_2 = -temp3 - temp5;
        double theta3_3 = temp4 - temp3;
        double theta3_4 = temp5 - temp3;

        if (abs(a4 * cos(theta2(0) + theta3_1) + d4 * sin(theta2(0) + theta3_1) + a3 * cos(theta2(0)) - xw1) < 1e-3)
        {
            theta3(0) = theta3_1;
        }
        else
        {
            theta3(0) = theta3_3;
        }

        if (abs(a4 * cos(theta2(1) + theta3_2) + d4 * sin(theta2(1) + theta3_2) + a3 * cos(theta2(1)) - xw1) < 1e-3)
        {
            theta3(1) = theta3_2;
        }
        else
        {
            theta3(1) = theta3_4;
        }

        if (theta3(0) > -102.78 / 180 * M_PI)
        {
            theta3(0) = theta3(0) - 2 * M_PI;
        }
        if (theta3(1) > -102.78 / 180 * M_PI)
        {
            theta3(1) = theta3(1) - 2 * M_PI;
        }
        for ( int j=0 ; j<2 ; j++ )
        {
            possible_joint_pos.col(result_count) << theta1(i),
                                            theta2(j) + 172.22 / 180 * M_PI,
                                            theta3(j) + 102.78 / 180 * M_PI;
            result_count++;
        }
    }
    int best_id = 0;
    double min_movement = std::numeric_limits<double>::max();
    for (int i = 0; i < result_count; ++i)
    {
        int greater = 1;
        for ( int j=0 ; j<3 ; j++ )
        {
            if ( possible_joint_pos(j,i) < joint_pos_limit_low_[j] )
            {
                greater = 0;
            }
        }
        int less = 1;
        for ( int j=0 ; j<3 ; j++ )
        {
            if ( possible_joint_pos(j,i) > joint_pos_limit_high_[j] )
            {
                less = 0;
            }
        }
        if (less && greater)
        {
            double movement = (possible_joint_pos.col(i) - ref_conf).squaredNorm();
            if (movement < min_movement)
            {
                min_movement = movement;
                best_id = i;
            }
        }
    }
    shoulder_joint_pos = possible_joint_pos.col(best_id);

    return err;
}

ErrorCode getInverseKinematics(
    Eigen::Vector<double, PiperArmNumDof>& joint_pos,
    const Eigen::Matrix4d& base_transform,
    const Eigen::Matrix4d& pose,
    const Eigen::Vector<double,6>& ref_conf)
{
    Eigen::Matrix<double, 6, 8> joint_pos_list;

    int result_count = 0;

    double d1 = 0.123;
    double alpha2 = -M_PI / 2;
    double a3 = 0.28503;
    double d4 = 0.25075;
    double a4 = -0.02198;
    double alpha5 = -M_PI / 2;
    double d6 = 0.091;
    double alpha6 = M_PI / 2;

    // Inverse of base transform matrix
    Eigen::Matrix4d base_transform_inv = Eigen::Matrix4d::Identity();
    base_transform_inv.block<3, 3>(0, 0) = base_transform.block<3, 3>(0, 0).transpose();
    base_transform_inv.block<3, 1>(0, 3) = -base_transform_inv.block<3, 3>(0, 0) * base_transform.block<3, 1>(0, 3);

    // Pose in arm base frame
    Eigen::Matrix4d pose_base = base_transform_inv * pose;

    // Wrist position in arm base frame
    Eigen::Vector3d pw0 = pose_base.block<3, 1>(0, 3) - d6 * pose_base.block<3, 1>(0, 2);

    Eigen::Vector2d theta1;
    Eigen::Vector2d theta2;
    Eigen::Vector2d theta3;

    theta1(0) = atan2(pw0(1), pw0(0));

    if (theta1(0) > 0)
    {
        theta1(1) = theta1(0) - M_PI;
    }
    else
    {
        theta1(1) = theta1(0) + M_PI;
    }

    for (int i = 0; i < 2; ++i)
    {
        // Transform matrix from arm base frame to link 1 frame
        Eigen::Matrix4d T01;
        T01 << cos(theta1(i)), -sin(theta1(i)), 0, 0,
               sin(theta1(i)), cos(theta1(i)), 0, 0,
               0, 0, 1, d1,
               0, 0, 0, 1;
        Eigen::Matrix4d T01_inv;
        T01_inv.block<3, 3>(0, 0) = T01.block<3, 3>(0, 0).transpose();
        T01_inv.block<3, 1>(0, 3) = -T01_inv.block<3, 3>(0, 0) * T01.block<3, 1>(0, 3);
        T01_inv.block<1, 4>(3, 0) << 0, 0, 0, 1;

        // Wrist pose transformation matrix in link 1 frame
        // Eigen::Matrix4d pose_wrist_1 = T01_inv * pose_wrist_base;
        Eigen::Vector4d pos_wrist_1 = T01_inv * Eigen::Vector4d(pw0(0),pw0(1),pw0(2),1);

        // X, Y, Z position of wrist in link 1 frame
        double xw1 = pos_wrist_1(0);
        double zw1 = pos_wrist_1(2);

        double temp1 = (xw1 * xw1 + zw1 * zw1 - d4 * d4 - a4 * a4 + a3 * a3) / (2 * zw1);
        double temp2 = xw1 / zw1;

        double delta = temp1 * temp1 * temp2 * temp2 - (1 + temp2 * temp2) * (temp1 * temp1 - a3 * a3);
        if (delta < 0)
        {
            if (delta > 1e-5)
            {
                delta = 0;
            }
            else
            {
                /* no wrist position solution. */
                return NoResult;
            }
        }

        double x31_1 = (temp1 * temp2 + sqrt(delta)) / (1 + temp2 * temp2);
        double x31_2 = (temp1 * temp2 - sqrt(delta)) / (1 + temp2 * temp2);

        double z31_1 = temp1 - temp2 * x31_1;
        double z31_2 = temp1 - temp2 * x31_2;

        theta2(0) = -atan2(z31_1, x31_1);
        theta2(1) = -atan2(z31_2, x31_2);

        double temp3 = atan2(d4, -a4);
        double temp4 = acos(std::max(-1.0, std::min(1.0, (a4 * a4 + d4 * d4 + x31_1 * x31_1 + z31_1 * z31_1 - xw1 * xw1 - zw1 * zw1) /
                                        (2 * sqrt(a4 * a4 + d4 * d4) * sqrt(x31_1 * x31_1 + z31_1 * z31_1)))));
        double temp5 = acos(std::max(-1.0, std::min(1.0, (a4 * a4 + d4 * d4 + x31_2 * x31_2 + z31_2 * z31_2 - xw1 * xw1 - zw1 * zw1) /
                                        (2 * sqrt(a4 * a4 + d4 * d4) * sqrt(x31_2 * x31_2 + z31_2 * z31_2)))));

        double theta3_1 = -temp3 - temp4;
        double theta3_2 = -temp3 - temp5;
        double theta3_3 = temp4 - temp3;
        double theta3_4 = temp5 - temp3;

        if (abs(a4 * cos(theta2(0) + theta3_1) + d4 * sin(theta2(0) + theta3_1) + a3 * cos(theta2(0)) - xw1) < 1e-3)
        {
            theta3(0) = theta3_1;
        }
        else
        {
            theta3(0) = theta3_3;
        }

        if (abs(a4 * cos(theta2(1) + theta3_2) + d4 * sin(theta2(1) + theta3_2) + a3 * cos(theta2(1)) - xw1) < 1e-3)
        {
            theta3(1) = theta3_2;
        }
        else
        {
            theta3(1) = theta3_4;
        }

        if (theta3(0) > -102.78 / 180 * M_PI)
        {
            theta3(0) = theta3(0) - 2 * M_PI;
        }
        if (theta3(1) > -102.78 / 180 * M_PI)
        {
            theta3(1) = theta3(1) - 2 * M_PI;
        }

        for (int j = 0; j < 2; ++j)
        {
            Eigen::Matrix3d R1, R2, R3;
            R1 = Eigen::AngleAxisd(theta1(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            R2 = Eigen::AngleAxisd(theta2(j), Eigen::Vector3d::UnitY()).toRotationMatrix();
            R3 = Eigen::AngleAxisd(theta3(j), Eigen::Vector3d::UnitY()).toRotationMatrix();

            Eigen::Matrix3d R = R3.transpose() * R2.transpose() * R1.transpose() * pose_base.block<3, 3>(0, 0);

            Eigen::Vector2d theta4;
            Eigen::Vector2d theta5;
            Eigen::Vector2d theta6;

            theta4(0) = atan2(R(1, 2), R(0, 2));
            if (theta4(0) > 0)
            {
                theta4(1) = theta4(0) - M_PI;
            }
            else
            {
                theta4(1) = theta4(0) + M_PI;
            }

            theta5(0) = atan2(sqrt(R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1)), R(2, 2));
            theta5(1) = -theta5(0);

            theta6(0) = atan2(R(2, 1), -R(2, 0));
            if (theta6(0) > 0)
            {
                theta6(1) = theta6(0) - M_PI;
            }
            else
            {
                theta6(1) = theta6(0) + M_PI;
            }

            for (int k = 0; k < 2; ++k)
            {
                joint_pos_list.col(result_count) << theta1(i),
                                               theta2(j) + 172.22 / 180 * M_PI,
                                               theta3(j) + 102.78 / 180 * M_PI,
                                               theta4(k),
                                               theta5(k),
                                               theta6(k);
                result_count++;
            }
        }
    }

    int best_id = -1;
    double min_movement = std::numeric_limits<double>::max();

    for (int i = 0; i < result_count; ++i)
    {
        int greater = 1;
        for ( int j=0 ; j<num_dof_ ; j++ )
        {
            if ( joint_pos_list(j,i) < joint_pos_limit_low_[j] )
            {
                greater = 0;
            }
        }
        int less = 1;
        for ( int j=0 ; j<num_dof_ ; j++ )
        {
            if ( joint_pos_list(j,i) > joint_pos_limit_high_[j] )
            {
                less = 0;
            }
        }
        if (less && greater)
        {
            double movement = (joint_pos_list.col(i) - ref_conf).squaredNorm();
            if (movement < min_movement)
            {
                min_movement = movement;
                best_id = i;
            }
        }
    }
    if ( best_id == -1 )
    {
        return NoResult;
    }

    joint_pos = joint_pos_list.col(best_id);

    return OK;
}

ErrorCode ArmModel::getDampedLeastSquareInverseKinematics(
    Eigen::Vector<double, PiperArmNumDof>& joint_pos, 
    const double lambda,
    const Eigen::Vector<double,6> tolerance,
    const size_t max_iteration,
    const Eigen::Matrix4d &pose,
    const Eigen::Vector<double,6>& ref_conf) const
{
    auto getPoseDiff = [](const Eigen::Matrix4d& target_pose, const Eigen::Matrix4d& current_pose)
    {
        Eigen::Vector<double,6> pose_diff = Eigen::Vector<double,6>::Zero();
        pose_diff.block<3,1>(0,0) = target_pose.block<3,1>(0,3) - current_pose.block<3,1>(0,3);
        Eigen::Quaterniond target_quat(target_pose.block<3,3>(0,0));
        Eigen::Quaterniond current_quat(current_pose.block<3,3>(0,0));
        pose_diff.block<3,1>(3,0) = (target_quat * current_quat.conjugate()).vec();
        return pose_diff;
    };
    auto getDampedLeastSquares = [](const Eigen::Matrix<double,6,num_dof_>& J, const Eigen::Vector<double,6>& dx, double lambda = 0.1)
    {
        Eigen::Matrix<double,num_dof_,num_dof_> I = Eigen::Matrix<double,num_dof_,num_dof_>::Identity();
        Eigen::Matrix<double,num_dof_,num_dof_> JJT_plus_lambdaI = J * J.transpose() + lambda * lambda * I;
        Eigen::Matrix<double,num_dof_,num_dof_> dls_pinv = J.transpose() * JJT_plus_lambdaI.completeOrthogonalDecomposition().pseudoInverse();
        return dls_pinv * dx;
    };

    ErrorCode err = OK;
    
    Eigen::Vector<double,num_dof_> best_joint_pos = Eigen::Vector<double,num_dof_>::Zero();
    Eigen::Vector<double,num_dof_> joint_pos_diff = Eigen::Vector<double,num_dof_>::Zero();
    Eigen::Vector3d shoulder_joint_pos = Eigen::Vector3d::Zero();

    err = getShoulderJointPos(shoulder_joint_pos, pose, ref_conf.block<3,1>(0,0));

    if ( err == NoResult )
    {
        return err;
    }

    best_joint_pos.block<3,1>(0,0) = shoulder_joint_pos;

    for ( size_t i=0 ; i<max_iteration ; i++ )
    {
        std::array<Eigen::Matrix4d,ArmModel::num_link_> link_transform;
        std::array<Eigen::Matrix4d,ArmModel::num_link_> link_com_transform;
        std::array<Eigen::Matrix<double,6, PiperArmNumDof>,ArmModel::num_link_> link_jacobian;
        this->getTransform(link_transform, link_com_transform, best_joint_pos);
        this->getLinkSpaceJacobian(link_jacobian, link_transform);
        Eigen::Vector<double,6> pose_diff = getPoseDiff(pose, link_transform[5]);

        /* Check if the pose difference is within the tolerance */
        if ( (pose_diff.array() < tolerance.array()).all() )
        {
            joint_pos = best_joint_pos;
            return OK;
        }

        joint_pos_diff = getDampedLeastSquares(link_jacobian[5], pose_diff, lambda);

        best_joint_pos += joint_pos_diff;
    }
    joint_pos = best_joint_pos;

    return err;
}