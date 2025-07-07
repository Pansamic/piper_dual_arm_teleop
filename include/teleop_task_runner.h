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
#include <Eigen/Geometry>
#include <asio.hpp>
#include <itc/backend/RingBuf.hpp>
#include <comm_channel.hpp>
#include <msgs/whole_body_msg/whole_body_msg.h>
#include <msgs/whole_body_msg/whole_body_receiver.hpp>
#include <msgs/whole_body_msg/whole_body_sender.hpp>
#include <joint_state.h>
#include <arm_model.h>
#include <arm_controller.h>
#include <arm_planner.h>
#include <arm_interface.h>

class TeleopTaskRunner
{
public:
    TeleopTaskRunner(std::shared_ptr<ArmInterface> interface, size_t freq_plan, size_t freq_ctrl);
    ~TeleopTaskRunner() = default;
    void run();
private:
    static const Eigen::Vector3d head_position_;
    static const Eigen::Quaterniond left_hand_orientation_offset_;
    static const Eigen::Quaterniond right_hand_orientation_offset_;
    static const Eigen::Matrix4d left_arm_base_transform_;
    static const Eigen::Matrix4d right_arm_base_transform_;

    std::atomic<bool> running_;
    size_t freq_plan_;
    size_t freq_ctrl_;

    asio::io_context io_context_;
    CommChannel<ChannelMode::UDP, WholeBodySender, WholeBodyReceiver> channel_;
    MsgQueue send_mq_;
    MsgQueue recv_mq_;

    Eigen::Vector3d left_hand_target_pos_;
    Eigen::Quaterniond left_hand_target_orientation_;
    Eigen::Matrix4d left_hand_target_pose_;
    Eigen::Quaterniond left_hand_actual_orientation_;
    Eigen::Vector3d right_hand_target_pos_;
    Eigen::Quaterniond right_hand_target_orientation_;
    Eigen::Matrix4d right_hand_target_pose_;
    Eigen::Quaterniond right_hand_actual_orientation_;

    JointState left_arm_target_joint_state_;
    JointState left_arm_actual_joint_state_;
    JointState right_arm_target_joint_state_;
    JointState right_arm_actual_joint_state_;

    double left_gripper_control_ = 0;
    double right_gripper_control_ = 0;

    std::shared_ptr<ArmInterface> interface_;
    std::shared_ptr<ArmModel> left_arm_model_;
    std::shared_ptr<ArmModel> right_arm_model_;
    std::unique_ptr<ArmController> controller_;
    std::unique_ptr<ArmPlanner> planner_;

    /* trajectory amount in buffer, indicates how many continuous
     * trajectories are stored in buffer. */
    const size_t traj_buf_size_ = 3;

    /* This buffer stores the inverse kinematics joint state
     * for planner. This buffer is assigned to planner object
     * when planner is created and planner can access this buffer
     * to get target joint state from task runner. */
    // RingBuffer<JointState> left_arm_target_joint_state_buffer_;
    // RingBuffer<JointState> right_arm_target_joint_state_buffer_;

    /* Trajectory is a sequence of joint states. This buffer is assigned to
     * planner object and controller object and they share this buffer 
     * because the planner will write trajectory joint states into this 
     * buffer and controller will read trajectory from this buffer.*/
    RingBuffer<JointState> left_arm_trajectory_buffer_;
    RingBuffer<JointState> right_arm_trajectory_buffer_;

    RingBuffer<Eigen::Vector<double,ArmModel::num_dof_>> left_arm_target_joint_pos_history_; 
    RingBuffer<Eigen::Vector<double,ArmModel::num_dof_>> right_arm_target_joint_pos_history_;

    void scaleLeftHandPose(Eigen::Matrix4d& pose);
    void scaleRightHandPose(Eigen::Matrix4d& pose);

    /**
     * @brief Check joint position jitter, or 
     * 
     * @param joint_pos 
     * @return true 
     * @return false 
     */
    bool checkInvalidJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
};

#endif // __TELEOP_TASK_RUNNER_H__