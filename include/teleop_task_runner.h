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
#include <msgs/nav_state_msg/nav_state_msg.h>
#include <msgs/nav_state_msg/nav_state_receiver.hpp>
#include <msgs/nav_state_msg/nav_state_sender.hpp>
#include <error_codes.h>
#include <joint_state.h>
#include <trajectory_buffer.hpp>
#include <arm_model.h>
#include <arm_controller.h>
#include <arm_planner.h>
#include <arm_interface.h>

class TeleopTaskRunner
{
public:
    TeleopTaskRunner() = delete;
    TeleopTaskRunner(std::shared_ptr<ArmInterface> interface, size_t freq_plan, size_t freq_ctrl);
    ~TeleopTaskRunner() = default;
    void initialize();
    void run();
    void stop();
    void setHomeConfiguration();
private:
    static const Eigen::Vector3d head_position_;
    static const Eigen::Quaterniond left_hand_orientation_offset_;
    static const Eigen::Quaterniond right_hand_orientation_offset_;
    static const Eigen::Matrix4d left_arm_base_transform_;
    static const Eigen::Matrix4d right_arm_base_transform_;

    bool running_;
    size_t freq_plan_;
    size_t freq_ctrl_;

    asio::io_context io_context_;
    std::thread io_context_thread_;
    CommChannel<ChannelMode::Unix, NavStateSender, WholeBodyReceiver> channel_;
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

    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_pos_;
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_pos_;

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

    /* Trajectory is a sequence of joint states. This buffer is assigned to
     * planner object and controller object and they share this buffer 
     * because the planner will write trajectory joint states into this 
     * buffer and controller will read trajectory from this buffer.*/
    TrajectoryBuffer<ArmPlanner::num_plan_waypoint_> left_arm_trajectory_buffer_;
    TrajectoryBuffer<ArmPlanner::num_plan_waypoint_> right_arm_trajectory_buffer_;

    RingBuffer<Eigen::Vector<double,ArmModel::num_dof_>> left_arm_joint_state_history_; 
    RingBuffer<Eigen::Vector<double,ArmModel::num_dof_>> right_arm_joint_state_history_;

    /**
     * @brief Program termination signal handler.
     * 
     * @param signum signal number.
     */
    // static void handleSignal(int signum);
    /**
     * @brief Apply a scaler on position, add position offset and apply a orientation offset.
     * 
     * @param position Position of end effector, unit: m.
     * @param orientation Orientation of end effector, represented in quaternion.
     */
    void scaleLeftHandPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
    /**
     * @brief Apply a scaler on position, add position offset and apply a orientation offset.
     * 
     * @param position Position of end effector, unit: m.
     * @param orientation Orientation of end effector, represented in quaternion.
     */
    void scaleRightHandPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
    /**
     * @brief Check joint position jitter or inaccessible joint configuration.
     * 
     * @param joint_pos Joint position.
     * @return true Joint position is valid.
     * @return false Joint position is invalid.
     */
    bool checkInvalidJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos);
};

#endif // __TELEOP_TASK_RUNNER_H__