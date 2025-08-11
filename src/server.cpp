/**
 * @file server.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief MuJoCo GUI server to simulate robotics manipulator and send control messages to client.
 * @version 0.1
 * @date 2025-08-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <time.h>
#include <iostream>
#include <log.hpp>
#include <config.h>
#include <termination.h>
#include <utils.hpp>
#include <mujoco_frontend.hpp>
#include <mujoco_backend.hpp>
#include <messenger.hpp>
#include <piper_model.hpp>
#include <trajectory_buffer.hpp>
#include <arm_planner.hpp>
#include <arm_controller.hpp>
#include <mujoco_interface.hpp>

int main(void)
{
    constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;

    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "server");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested = false;
    /* Register SIGINT handler. */
    TerminationHandler::setup();

    Messenger<ChannelMode::UDP> msg_sender(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
    msg_sender.start(true, false);

    // MujocoBackend<double, NumDof> mj_backend;
    // MujocoFrontend mj_frontend(mj_backend.getMujocoModel(), mj_backend.getMujocoData());
    // mj_backend.start();
    // mj_frontend.start();
    ArmSimulationInterface<double, NumDof> interface;
    interface.start(PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque_full.xml");

    PiperArmModel<double> left_arm_model(
        CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
        CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW);

    PiperArmModel<double> right_arm_model(
        CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
        CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW);
    
    ArmController<double, PiperArmModel<double>::NumLink, NumDof> controller(
        1.0/CONFIG_CONTROLLER_FREQUENCY,
        {100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
        {0, 0, 0, 0, 0, 0},
        {10.0, 10.0, 10.0, 10.0, 10.0});
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;

    auto update_plan_left_arm = [&]()
    {
        Eigen::Vector<double, 3> left_hand_mocap_pos = interface.getLeftHandSitePosition();
        Eigen::Matrix<double, 3, 3> left_hand_mocap_ori = interface.getLeftHandSiteOrientation();
        Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
        left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori;
        left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
        Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = interface.getLeftArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( left_arm_model.getInverseKinematics(ik_result, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult)
        {
            if ( left_arm_model.getDampedLeastSquareInverseKinematics(ik_result, left_arm_model, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult )
            {
                LOG_WARN("left arm inverse kinematics fails.");
                return ;
            }
        }
        auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_left_arm_joint_pos, ik_result);
        left_arm_trajectory_buffer.write(time_point, trajectory);
    };

    auto update_plan_right_arm = [&]()
    {
        Eigen::Vector<double, 3> right_hand_mocap_pos = interface.getRightHandSitePosition();
        Eigen::Matrix<double, 3, 3> right_hand_mocap_ori = interface.getRightHandSiteOrientation();
        Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
        right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori;
        right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
        Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = interface.getRightArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( right_arm_model.getInverseKinematics(ik_result, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
        {
            if ( right_arm_model.getDampedLeastSquareInverseKinematics(ik_result, right_arm_model, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
            {
                LOG_WARN("right arm inverse kinematics fails.");
                return ;
            }
        }
        auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_right_arm_joint_pos, ik_result);
        right_arm_trajectory_buffer.write(time_point, trajectory);                
    };

    auto update_control = [&]()
    {
        std::array<double, NumDof> left_arm_target_joint_pos, right_arm_target_joint_pos;
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            for ( int i=0 ; i<NumDof ; i++ )
                left_arm_target_joint_pos[i] = target_joint_pos(i);
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {interface.getLeftArmJointPosition(), interface.getLeftArmJointVelocity(), interface.getLeftArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(left_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            interface.setLeftArmJointControl(target_joint_torque);
            LOG_TRACE("left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
            LOG_TRACE("left arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
            LOG_TRACE("left arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
            LOG_TRACE("left arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
            LOG_TRACE("left arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
                current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
            LOG_TRACE("left arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
                current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));
        }
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            for ( int i=0 ; i<NumDof ; i++ )
                right_arm_target_joint_pos[i] = target_joint_pos(i);
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {interface.getRightArmJointPosition(), interface.getRightArmJointVelocity(), interface.getRightArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(right_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            interface.setRightArmJointControl(target_joint_torque);
            LOG_TRACE("right arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
            LOG_TRACE("right arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
            LOG_TRACE("right arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
            LOG_TRACE("right arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
            LOG_TRACE("right arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
                current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
            LOG_TRACE("right arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
                current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));            
        }
        msg_sender.send<double>(true, left_arm_target_joint_pos, right_arm_target_joint_pos);
    };

    struct timespec wakeup_time = {0, 0};
    struct timespec cycletime = {0, 0}; // Initialize to zero
    
    // Convert frequency (Hz) to timespec
    double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
    cycletime.tv_sec = static_cast<time_t>(period_seconds);
    cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    std::size_t count = 0;
    while ( !TerminationHandler::stop_requested )
    {
        increaseTimeSpec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        update_control();
        
        count++;

        if ( count == 10 )
        {
            update_plan_left_arm();
            update_plan_right_arm();

            count = 0;
        }
    }

    msg_sender.stop();
    interface.stop();

    spdlog::drop_all();

    return 0;
}
