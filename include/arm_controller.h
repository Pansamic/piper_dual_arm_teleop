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
    ArmController(std::unique_ptr<ArmModel> model, std::unique_ptr<ArmInterface> interface, const unsigned int control_interval);
    ~ArmController();

    void setLeftTargetJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
    void setLeftTargetJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void setLeftTargetJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void setLeftTargetJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void setLeftTargetJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void setLeftTargetJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6);
    void setLeftTargetJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq);
    void setLeftTargetJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);

    void setRightTargetJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
    void setRightTargetJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void setRightTargetJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void setRightTargetJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void setRightTargetJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void setRightTargetJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6);
    void setRightTargetJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq);
    void setRightTargetJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);

    void updateLeftActualJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
    void updateLeftActualJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateLeftActualJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void updateLeftActualJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateLeftActualJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void updateLeftActualJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateLeftActualJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq);
    void updateLeftActualJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);

    void updateRightActualJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
    void updateRightActualJointPosition(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateRightActualJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void updateRightActualJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateRightActualJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel);
    void updateRightActualJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6);
    void updateRightActualJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq);
    void updateRightActualJointTorque(double j1, double j2, double j3, double j4, double j5, double j6);
private:

    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kp_;
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kd_;

    std::atomic<bool> running_;

    std::unique_ptr<ArmInterface> interface_;

    std::unique_ptr<ArmModel> model_;

    std::thread control_thread_;

    std::mutex left_target_state_mtx_;
    std::mutex left_actual_state_mtx_;
    std::mutex right_target_state_mtx_;
    std::mutex right_actual_state_mtx_;

    Eigen::Vector<double,ArmModel::num_dof_> left_target_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> left_target_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> left_target_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> left_target_joint_torq_;
    Eigen::Vector<double,ArmModel::num_dof_> left_actual_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> left_actual_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> left_actual_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> left_actual_joint_torq_;

    Eigen::Vector<double,ArmModel::num_dof_> right_target_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> right_target_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> right_target_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> right_target_joint_torq_;
    Eigen::Vector<double,ArmModel::num_dof_> right_actual_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> right_actual_joint_vel_;
    Eigen::Vector<double,ArmModel::num_dof_> right_actual_joint_acc_;
    Eigen::Vector<double,ArmModel::num_dof_> right_actual_joint_torq_;

    void control_loop_();
};

#endif // __ARM_CONTROLLER_H__