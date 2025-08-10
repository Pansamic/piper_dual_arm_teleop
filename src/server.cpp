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

// class MujocoGUIServer
// {
// public:
//     MujocoGUIServer() :
//         msg_sender_(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT),
//         mj_backend_(),
//         mj_frontend_(mj_backend_.getMujocoModel(), mj_backend_.getMujocoData()),
//         left_arm_model_(
//             CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
//             CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW),
//         right_arm_model_(
//             CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
//             CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW),
//         termination_(false)
//     {
//         // Initialize the messenger
//         msg_sender_.start(true, false);
        
//         // Start MuJoCo backend and frontend
//         mj_backend_.start();
//         mj_frontend_.start();
//     }
//     ~MujocoGUIServer() = default;

//     void start()
//     {
//         // Set termination flag to false
//         termination_ = false;
        
//         // Start the threads
//         planner_thread_ = registerRealTimeLoop(
//             [this]() { updatePlan(); }, 
//             CONFIG_PLANNER_FREQUENCY, 
//             &termination_
//         );
        
//         executor_thread_ = registerRealTimeLoop(
//             [this]() { executeTrajectory(); }, 
//             CONFIG_CONTROLLER_FREQUENCY, 
//             &termination_
//         );
//     }

//     void stop()
//     {
//         // Set termination flag to true
//         termination_ = true;
        
//         // Join threads if they are joinable
//         if (planner_thread_.joinable())
//         {
//             planner_thread_.join();
//         }
//         if (executor_thread_.joinable())
//         {
//             executor_thread_.join();
//         }
        
//         // Stop components
//         msg_sender_.stop();
//         mj_backend_.stop();
//         mj_frontend_.stop();
//     }

// private:
//     void updatePlan()
//     {
//         // Left arm planning
//         {
//             Eigen::Vector<double, 3> left_hand_mocap_pos = mj_backend_.getLeftHandMocapPosition();
//             Eigen::Quaternion<double> left_hand_mocap_ori = mj_backend_.getLeftHandMocapOrientation();
            
//             Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori.toRotationMatrix();
//             left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
            
//             Eigen::Matrix<double, 4, 4> left_hand_mocap_pose_in_base = 
//                 PiperArmModel<double>::getInverseTransform(left_arm_model_.getBaseTransform()) * left_hand_mocap_pose;
            
//             Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = mj_backend_.getLeftArmJointPosition();
//             Eigen::Vector<double, NumDof> ik_result;
            
//             left_arm_model_.getDampedLeastSquareInverseKinematics(
//                 ik_result, 
//                 left_arm_model_, 
//                 0.1f, 
//                 Eigen::Vector<double, 6>(0.05f, 0.05f, 0.05f, 0.1f, 0.1f, 0.1f), 
//                 200,
//                 left_hand_mocap_pose_in_base, 
//                 actual_left_arm_joint_pos
//             );
            
//             auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = 
//                 left_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
                
//             auto [time_point, trajectory] = LinearArmPlanner<Eigen::Vector<double, NumDof>>::plan(
//                 std::chrono::steady_clock::now(), 
//                 1.0f / CONFIG_PLANNER_FREQUENCY,
//                 target_left_arm_joint_pos, 
//                 ik_result, 
//                 CONFIG_WAYPOINT_AMOUNT
//             );
            
//             left_arm_trajectory_buffer_.write(time_point, trajectory);
//         }
        
//         // Right arm planning
//         {
//             Eigen::Vector<double, 3> right_hand_mocap_pos = mj_backend_.getRightHandMocapPosition();
//             Eigen::Quaternion<double> right_hand_mocap_ori = mj_backend_.getRightHandMocapOrientation();
            
//             Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori.toRotationMatrix();
//             right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
            
//             Eigen::Matrix<double, 4, 4> right_hand_mocap_pose_in_base = 
//                 PiperArmModel<double>::getInverseTransform(right_arm_model_.getBaseTransform()) * right_hand_mocap_pose;
            
//             Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = mj_backend_.getRightArmJointPosition();
//             Eigen::Vector<double, NumDof> ik_result;
            
//             right_arm_model_.getDampedLeastSquareInverseKinematics(
//                 ik_result, 
//                 right_arm_model_, 
//                 0.1f, 
//                 Eigen::Vector<double, 6>(0.05f, 0.05f, 0.05f, 0.1f, 0.1f, 0.1f), 
//                 200,
//                 right_hand_mocap_pose_in_base, 
//                 actual_right_arm_joint_pos
//             );
            
//             auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = 
//                 right_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
                
//             auto [time_point, trajectory] = LinearArmPlanner<Eigen::Vector<double, NumDof>>::plan(
//                 std::chrono::steady_clock::now(), 
//                 1.0f / CONFIG_PLANNER_FREQUENCY,
//                 target_right_arm_joint_pos, 
//                 ik_result, 
//                 CONFIG_WAYPOINT_AMOUNT
//             );
            
//             right_arm_trajectory_buffer_.write(time_point, trajectory);
//         }
//     }

//     void executeTrajectory()
//     {
//         // Execute left arm trajectory
//         auto [left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = 
//             left_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//         mj_backend_.setLeftArmJointPosition(left_arm_joint_pos);
        
//         // Execute right arm trajectory
//         auto [right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = 
//             right_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//         mj_backend_.setRightArmJointPosition(right_arm_joint_pos);
//     }

//     // Components
//     Messenger<ChannelMode::UDP> msg_sender_;
//     MujocoBackend<double, NumDof> mj_backend_;
//     MujocoFrontend mj_frontend_;
//     PiperArmModel<double> left_arm_model_;
//     PiperArmModel<double> right_arm_model_;
//     BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer_;
//     BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer_;

//     // Threads
//     std::thread planner_thread_;
//     std::thread executor_thread_;

//     // Termination flag
//     bool termination_;
// };

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

    MujocoBackend<double, NumDof> mj_backend;
    MujocoFrontend mj_frontend(mj_backend.getMujocoModel(), mj_backend.getMujocoData());
    mj_backend.start();
    mj_frontend.start();

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
        {20.0, 20.0, 20.0, 20.0, 20.0});
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;

    auto update_plan = [&]()
    {
        {
            // Eigen::Vector<double, 3> left_hand_mocap_pos = mj_backend.getLeftHandMocapPosition();
            // Eigen::Quaternion<double> left_hand_mocap_ori = mj_backend.getLeftHandMocapOrientation();
            // Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
            Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
            left_hand_mocap_pose.block<3, 3>(0, 0) = Eigen::Quaternion<double>(0.71, 0, 0.71, 0).toRotationMatrix();
            left_hand_mocap_pose(0, 3) = 0.8;
            left_hand_mocap_pose(1, 3) = 0.2;
            left_hand_mocap_pose(2, 3) = 0.6;
            // std::cout << left_hand_mocap_pose << std::endl;
            // left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori.toRotationMatrix();
            // left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
            // Eigen::Matrix<double, 4, 4> left_hand_mocap_pose_in_base = PiperArmModel<double>::getInverseTransform(left_arm_model.getBaseTransform()) * left_hand_mocap_pose;
            // std::cout << "left base trans:" << std::endl << left_arm_model.getBaseTransform() << std::endl;
            Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = mj_backend.getLeftArmJointPosition();
            Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
            // ErrorCode err = left_arm_model.getDampedLeastSquareInverseKinematics(
            //     ik_result, left_arm_model, 0.1, Eigen::Vector<double, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, 
            //     left_hand_mocap_pose, actual_left_arm_joint_pos);
            ErrorCode err = left_arm_model.getInverseKinematics(ik_result, left_hand_mocap_pose, actual_left_arm_joint_pos);
            if ( err == ErrorCode::NoResult )
            {
                LOG_ERROR("left arm inverse kinematics fails.");
            }
            auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
                std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
                target_left_arm_joint_pos, ik_result);
            // std::cout << "trajectory:" << trajectory << std::endl;
            left_arm_trajectory_buffer.write(time_point, trajectory);
            // for ( std::size_t i=0 ; i<trajectory.size() ; i++ )
            // {
            //     LOG_INFO("left arm trajectory[{:d}]:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}", i,
            //         trajectory[i](0), trajectory[i](1), trajectory[i](2), trajectory[i](3), trajectory[i](4), trajectory[i](5));
            // }
        }
        // {
        //     Eigen::Vector<double, 3> right_hand_mocap_pos = mj_backend.getRightHandMocapPosition();
        //     Eigen::Quaternion<double> right_hand_mocap_ori = mj_backend.getRightHandMocapOrientation();
        //     Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
        //     // right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori.toRotationMatrix();
        //     // right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
        //     // Eigen::Matrix<double, 4, 4> right_hand_mocap_pose_in_base = PiperArmModel<double>::getInverseTransform(right_arm_model.getBaseTransform()) * right_hand_mocap_pose;
        //     Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = mj_backend.getRightArmJointPosition();
        //     Eigen::Vector<double, NumDof> ik_result;
        //     ErrorCode err = right_arm_model.getDampedLeastSquareInverseKinematics(
        //         ik_result, right_arm_model, 0.1, Eigen::Vector<double, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200,
        //         right_hand_mocap_pose, actual_right_arm_joint_pos);
        //     if ( err == ErrorCode::NoResult )
        //         LOG_ERROR("right arm inverse kinematics fails.");
        //     auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        //     auto [time_point, trajectory] = LinearArmPlanner<Eigen::Vector<double, NumDof>>::plan(
        //         std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
        //         target_right_arm_joint_pos, ik_result, CONFIG_WAYPOINT_AMOUNT);
        //     right_arm_trajectory_buffer.write(time_point, trajectory);
        // }
    };

    auto update_control = [&]()
    {
        auto [left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        JointState<double, NumDof> target_joint_state = {left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc};
        JointState<double, NumDof> current_joint_state = {mj_backend.getLeftArmJointPosition(), mj_backend.getLeftArmJointVelocity(), mj_backend.getLeftArmJointAcceleration()};
        auto left_arm_joint_torque = controller.computeTorqueControlOutput(left_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
        mj_backend.setLeftArmJointControl(left_arm_joint_torque);

        LOG_DEBUG("left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            left_arm_joint_pos(0), left_arm_joint_pos(1), left_arm_joint_pos(2), left_arm_joint_pos(3), left_arm_joint_pos(4), left_arm_joint_pos(5));
        LOG_DEBUG("left arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            left_arm_joint_vel(0), left_arm_joint_vel(1), left_arm_joint_vel(2), left_arm_joint_vel(3), left_arm_joint_vel(4), left_arm_joint_vel(5));
        LOG_DEBUG("left arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            left_arm_joint_acc(0), left_arm_joint_acc(1), left_arm_joint_acc(2), left_arm_joint_acc(3), left_arm_joint_acc(4), left_arm_joint_acc(5));
        LOG_DEBUG("left arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            left_arm_joint_torque(0), left_arm_joint_torque(1), left_arm_joint_torque(2), left_arm_joint_torque(3), left_arm_joint_torque(4), left_arm_joint_torque(5));
        LOG_DEBUG("left arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
            current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
        LOG_DEBUG("left arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
            current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));

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
            update_plan();

            count = 0;
        }
    }

    msg_sender.stop();
    mj_backend.stop();
    mj_frontend.stop();

    spdlog::drop_all();

    return 0;
}
