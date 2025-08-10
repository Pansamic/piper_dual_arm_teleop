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

int main(void)
{
    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "agent");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested = false;
    /* Register SIGINT handler. */
    TerminationHandler::setup();

    Messenger<ChannelMode::UDP> msg_sender(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
    msg_sender.start(true, false);

    MujocoBackend<float, PiperArmModel<float>::NumDof> mj_backend;
    MujocoFrontend mj_frontend(mj_backend.getMujocoModel(), mj_backend.getMujocoData());
    mj_backend.start();
    mj_frontend.start();

    // std::array<float, 3> left_arm_base_translation{CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z};
    // std::array<float, 3> left_arm_base_orientation{CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW};
    // auto left_arm_builder = PiperArmModel::getPiperArmModelBuilder<float>(left_arm_base_translation, left_arm_base_orientation);
    // RigidBodyTree<float, PiperArmModel::NumLink, PiperArmModel::NumDof> left_arm_model;
    // left_arm_model.build(left_arm_builder);
    PiperArmModel<float> left_arm_model(
        CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
        CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW);
    
    // std::array<float, 3> right_arm_base_translation{CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z};
    // std::array<float, 3> right_arm_base_orientation{CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW};
    // auto right_arm_builder = PiperArmModel::getPiperArmModelBuilder<float>(right_arm_base_translation, right_arm_base_orientation);
    // RigidBodyTree<float, PiperArmModel::NumLink, PiperArmModel::NumDof> right_arm_model;
    // right_arm_model.build(right_arm_builder);
    PiperArmModel<float> right_arm_model(
        CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
        CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW);

    // static const Eigen::Vector<float, PiperArmModel::NumDof> controller_kp =
    //     Eigen::Vector<float, PiperArmModel::NumDof>(5.0, 5.0, 5.0, 5.0, 5.0, 5.0);
    // static const Eigen::Vector<float, PiperArmModel::NumDof> controller_ki =
    //     Eigen::Vector<float, PiperArmModel::NumDof>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
    // static const Eigen::Vector<float, PiperArmModel::NumDof> controller_kd =
    //     Eigen::Vector<float, PiperArmModel::NumDof>(0.3, 0.3, 0.3, 0.3, 0.3, 0.3);
    // ArmController<float, PiperArmModel::NumLink, PiperArmModel::NumDof> controller(0.05, controller_kp, controller_ki, controller_kd);

    // LinearArmPlanner<Eigen::Vector<float, PiperArmModel<float>::NumDof>> planner;

    BsplineTrajectoryBuffer<float, PiperArmModel<float>::NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<float, PiperArmModel<float>::NumDof> right_arm_trajectory_buffer;

    auto update_plan = [&]()
    {
        {
            Eigen::Vector<float, 3> left_hand_mocap_pos = mj_backend.getLeftHandMocapPosition();
            Eigen::Quaternion<float> left_hand_mocap_ori = mj_backend.getLeftHandMocapOrientation();
            Eigen::Matrix<float, 4, 4> left_hand_mocap_pose = Eigen::Matrix<float, 4, 4>::Identity();
            left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori.toRotationMatrix();
            left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
            Eigen::Matrix<float, 4, 4> left_hand_mocap_pose_in_base = PiperArmModel<float>::getInverseTransform(left_arm_model.getBaseTransform()) * left_hand_mocap_pose;
            Eigen::Vector<float, PiperArmModel<float>::NumDof> actual_left_arm_joint_pos = mj_backend.getLeftArmJointPosition();
            Eigen::Vector<float, PiperArmModel<float>::NumDof> ik_result;
            left_arm_model.getDampedLeastSquareInverseKinematics(
                ik_result, left_arm_model, 0.1, Eigen::Vector<float, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, 
                left_hand_mocap_pose_in_base, actual_left_arm_joint_pos);
            auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            auto [time_point, trajectory] = LinearArmPlanner<Eigen::Vector<float, PiperArmModel<float>::NumDof>>::plan(
                std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
                target_left_arm_joint_pos, ik_result, CONFIG_WAYPOINT_AMOUNT);
            left_arm_trajectory_buffer.write(time_point, trajectory);
        }
        {
            Eigen::Vector<float, 3> right_hand_mocap_pos = mj_backend.getRightHandMocapPosition();
            Eigen::Quaternion<float> right_hand_mocap_ori = mj_backend.getRightHandMocapOrientation();
            Eigen::Matrix<float, 4, 4> right_hand_mocap_pose = Eigen::Matrix<float, 4, 4>::Identity();
            right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori.toRotationMatrix();
            right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
            Eigen::Matrix<float, 4, 4> right_hand_mocap_pose_in_base = PiperArmModel<float>::getInverseTransform(right_arm_model.getBaseTransform()) * right_hand_mocap_pose;
            Eigen::Vector<float, PiperArmModel<float>::NumDof> actual_right_arm_joint_pos = mj_backend.getRightArmJointPosition();
            Eigen::Vector<float, PiperArmModel<float>::NumDof> ik_result;
            right_arm_model.getDampedLeastSquareInverseKinematics(
                ik_result, right_arm_model, 0.1, Eigen::Vector<float, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200,
                right_hand_mocap_pose_in_base, actual_right_arm_joint_pos);
            auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            auto [time_point, trajectory] = LinearArmPlanner<Eigen::Vector<float, PiperArmModel<float>::NumDof>>::plan(
                std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
                target_right_arm_joint_pos, ik_result, CONFIG_WAYPOINT_AMOUNT);
            right_arm_trajectory_buffer.write(time_point, trajectory);
        }
    };

    auto execute_trajectory = [&]()
    {
        auto [left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        mj_backend.setLeftArmJointPosition(left_arm_joint_pos);
        mj_backend.setLeftArmJointPosition(right_arm_joint_pos);
    };

    std::thread planner_thread = registerRealTimeLoop(update_plan, CONFIG_PLANNER_FREQUENCY, &TerminationHandler::stop_requested);
    std::thread executor_thread = registerRealTimeLoop(execute_trajectory, CONFIG_CONTROLLER_FREQUENCY, &TerminationHandler::stop_requested);

    executor_thread.join();
    planner_thread.join();
    msg_sender.stop();
    mj_backend.stop();
    mj_frontend.stop();
    return 0;
}
