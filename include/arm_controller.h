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
#include <joint_state.h>
#include <itc/backend/RingBuf.hpp>
#include <arm_planner.h>
#include <arm_interface.h>

class ArmController
{
public:
    ArmController(
        std::shared_ptr<ArmModel> left_arm_model,
        std::shared_ptr<ArmModel> right_arm_model,
        std::shared_ptr<ArmInterface> interface,
        RingBuffer<JointState>& left_arm_trajectory_buffer,
        RingBuffer<JointState>& right_arm_trajectory_buffer,
        const size_t freq_ctrl);
    ~ArmController();
private:

    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kp_;
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kd_;

    std::atomic<bool> running_;

    /* Arm Interface */
    std::shared_ptr<ArmInterface> interface_;

    /* Left and right arm model, including kinematics and dynamics */
    std::shared_ptr<ArmModel> left_arm_model_;
    std::shared_ptr<ArmModel> right_arm_model_;

    /* Target joint state buffer to store control sequence */
    RingBuffer<JointState>& left_arm_target_state_buffer_;
    RingBuffer<JointState>& right_arm_target_state_buffer_;

    /* Frequency of control loop */
    size_t freq_ctrl_;

    /* Thread for control loop */
    std::thread control_thread_;

    void threadControl();
};

#endif // __ARM_CONTROLLER_H__