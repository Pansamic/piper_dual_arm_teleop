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
#include <config.h>
#include <log.hpp>
#include <utils.hpp>
#include <termination.h>
#include <messenger.hpp>
#include <trajectory_buffer.hpp>
#include <arm_planner.hpp>
#include <arm_controller.hpp>
#include <mujoco_interface.hpp>
#include <piper_model.hpp>

class DualArmTeleopServer
{
public:
    static constexpr std::size_t NumLink = PiperArmModel<double>::NumLink;
    static constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;
    explicit DualArmTeleopServer()
    :   left_arm_model(CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z, CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW),
        right_arm_model(CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z, CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW),
        controller(1.0/CONFIG_CONTROLLER_FREQUENCY, {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}, {0, 0, 0, 0, 0, 0}, {10.0, 10.0, 10.0, 10.0, 10.0}),
        messenger(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT){}
    ~DualArmTeleopServer() = default;
    bool initialize()
    {
        interface.start(PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque.xml");
        messenger.start(true, true);
        return true;
    }
    void stop()
    {
        interface.stop();
        messenger.stop();
    }
    void updatePlan()
    {
        auto [left_hand_pos, left_hand_ori] = updateArmPlan(left_arm_model, left_arm_trajectory_buffer);
        auto [right_hand_pos, right_hand_ori] = updateArmPlan(right_arm_model, right_arm_trajectory_buffer);
        std::array<double, 3> left_hand_position, right_hand_position;
        std::array<double, 4> left_hand_orientation, right_hand_orientation;

        left_hand_position[0] = left_hand_pos[0];
        left_hand_position[1] = left_hand_pos[1];
        left_hand_position[2] = left_hand_pos[2];

        left_hand_orientation[0] = left_hand_ori.x();
        left_hand_orientation[1] = left_hand_ori.y();
        left_hand_orientation[2] = left_hand_ori.z();
        left_hand_orientation[3] = left_hand_ori.w();

        right_hand_position[0] = right_hand_pos[0];
        right_hand_position[1] = right_hand_pos[1];
        right_hand_position[2] = right_hand_pos[2];

        right_hand_orientation[0] = right_hand_ori.x();
        right_hand_orientation[1] = right_hand_ori.y();
        right_hand_orientation[2] = right_hand_ori.z();
        right_hand_orientation[3] = right_hand_ori.w();

        messenger.sendDualArmHandPose(true, left_hand_position, left_hand_orientation, right_hand_position, right_hand_orientation);
    }
    void updateControl()
    {
        updateArmControl(left_arm_model, left_arm_trajectory_buffer);
        updateArmControl(right_arm_model, right_arm_trajectory_buffer);
    }
private:
    MujocoInterface<double, NumDof> interface;
    PiperArmModel<double> left_arm_model;
    PiperArmModel<double> right_arm_model;
    ArmController<double, NumLink, NumDof> controller;
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;
    Messenger<ChannelMode::UDP> messenger;

    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> updateArmPlan(const PiperArmModel<double>& model, BsplineTrajectoryBuffer<double, NumDof>& trajbuffer)
    {
        /* Rotation offset for Meta Quest VR controller pose */
        static const Eigen::Quaterniond offset(Eigen::AngleAxisd(M_PI * 3 / 4, Eigen::Vector3d::UnitY()));

        Eigen::Vector3d hand_site_pos;
        Eigen::Matrix3d hand_site_ori;
        Eigen::Vector<double, NumDof> actual_arm_joint_pos;
        if ( &model == &left_arm_model )
        {
            hand_site_pos = interface.getLeftHandSitePosition();
            hand_site_ori = interface.getLeftHandSiteOrientation();
            actual_arm_joint_pos = interface.getLeftArmJointPosition();
        }
        else
        {
            hand_site_pos = interface.getRightHandSitePosition();
            hand_site_ori = interface.getRightHandSiteOrientation();
            actual_arm_joint_pos = interface.getRightArmJointPosition();
        }
        Eigen::Matrix4d hand_site_pose = Eigen::Matrix4d::Identity();
        hand_site_pose.block<3, 3>(0, 0) = hand_site_ori /** offset.toRotationMatrix()*/;
        hand_site_pose.block<3, 1>(0, 3) = hand_site_pos;

        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( model.getInverseKinematics(ik_result, hand_site_pose, actual_arm_joint_pos) == ErrorCode::NoResult )
        {
            if ( model.getDampedLeastSquareInverseKinematics(ik_result, model, hand_site_pose, actual_arm_joint_pos) == ErrorCode::NoResult )
            {
                LOG_WARN("inverse kinematics failed.");
                return std::make_tuple(hand_site_pos, Eigen::Quaterniond(hand_site_ori));
            }
        }
        auto [target_arm_joint_pos, target_arm_joint_vel, target_arm_joint_acc] = trajbuffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_arm_joint_pos, ik_result);
        trajbuffer.write(time_point, trajectory);

        return std::make_tuple(hand_site_pos, Eigen::Quaterniond(hand_site_ori));
    }

    void updateArmControl(const PiperArmModel<double>& model, BsplineTrajectoryBuffer<double, NumDof>& trajbuffer)
    {
        auto [target_joint_pos, target_joint_vel, target_joint_acc] = trajbuffer.interpolate(std::chrono::steady_clock::now());
        JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
        JointState<double, NumDof> current_joint_state;
        if ( &model == &this->left_arm_model )
            current_joint_state = JointState<double, NumDof>{interface.getLeftArmJointPosition(), interface.getLeftArmJointVelocity(), interface.getLeftArmJointAcceleration()};
        else
            current_joint_state = JointState<double, NumDof>{interface.getRightArmJointPosition(), interface.getRightArmJointVelocity(), interface.getRightArmJointAcceleration()};
        auto target_joint_torque = controller.computeTorqueControlOutput(model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
        if ( &model == &this->left_arm_model )
            interface.setLeftArmJointControl(target_joint_torque);
        else
            interface.setRightArmJointControl(target_joint_torque);
    }
};

int main(void)
{
    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "client");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested = false;
    /* Register SIGINT handler. */
    TerminationHandler::setup();

    DualArmTeleopServer client;
    client.initialize();

    std::thread control_thread([&]()
    {
        struct timespec wakeup_time = {0, 0};
        struct timespec cycletime = {0, 0};
        double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
        cycletime.tv_sec = static_cast<time_t>(period_seconds);
        cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);
        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        while ( !TerminationHandler::stop_requested )
        {
            increaseTimeSpec(&wakeup_time, &cycletime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

            client.updateControl();
        }
    });

    struct timespec wakeup_time = {0, 0};
    struct timespec cycletime = {0, 0};
    double period_seconds = 1.0 / static_cast<double>(CONFIG_PLANNER_FREQUENCY);
    cycletime.tv_sec = static_cast<time_t>(period_seconds);
    cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    while ( !TerminationHandler::stop_requested )
    {
        increaseTimeSpec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        client.updatePlan();
    }

    control_thread.join();
    client.stop();

    spdlog::drop_all();
    return 0;
}
// #include <time.h>
// #include <iostream>
// #include <log.hpp>
// #include <config.h>
// #include <termination.h>
// #include <utils.hpp>
// #include <mujoco_frontend.hpp>
// #include <mujoco_backend.hpp>
// #include <messenger.hpp>
// #include <piper_model.hpp>
// #include <trajectory_buffer.hpp>
// #include <arm_planner.hpp>
// #include <arm_controller.hpp>
// #include <mujoco_interface.hpp>

// int main(void)
// {
//     constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;

//     /* Create log file. */
//     initLogger(PROJECT_PATH"/logs", "server");
//     /* Initialize program stop flag as false. */
//     TerminationHandler::stop_requested = false;
//     /* Register SIGINT handler. */
//     TerminationHandler::setup();

//     Messenger<ChannelMode::UDP> msg_sender(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
//     msg_sender.start(true, false);

//     // MujocoBackend<double, NumDof> mj_backend;
//     // MujocoFrontend mj_frontend(mj_backend.getMujocoModel(), mj_backend.getMujocoData());
//     // mj_backend.start();
//     // mj_frontend.start();
//     ArmSimulationInterface<double, NumDof> interface;
//     interface.start(PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque.xml");

//     PiperArmModel<double> left_arm_model(
//         CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
//         CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW);

//     PiperArmModel<double> right_arm_model(
//         CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
//         CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW);
    
//     ArmController<double, PiperArmModel<double>::NumLink, NumDof> controller(
//         1.0/CONFIG_CONTROLLER_FREQUENCY,
//         {100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
//         {0, 0, 0, 0, 0, 0},
//         {10.0, 10.0, 10.0, 10.0, 10.0});
//     BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
//     BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;

//     auto update_plan_left_arm = [&]()
//     {
//         Eigen::Vector<double, 3> left_hand_mocap_pos = interface.getLeftHandSitePosition();
//         Eigen::Matrix<double, 3, 3> left_hand_mocap_ori = interface.getLeftHandSiteOrientation();
//         Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//         left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori;
//         left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
//         Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = interface.getLeftArmJointPosition();
//         Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
//         if ( left_arm_model.getInverseKinematics(ik_result, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult)
//         {
//             if ( left_arm_model.getDampedLeastSquareInverseKinematics(ik_result, left_arm_model, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult )
//             {
//                 LOG_WARN("left arm inverse kinematics fails.");
//                 return ;
//             }
//         }
//         auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
//         auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
//             std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
//             target_left_arm_joint_pos, ik_result);
//         left_arm_trajectory_buffer.write(time_point, trajectory);
//     };

//     auto update_plan_right_arm = [&]()
//     {
//         Eigen::Vector<double, 3> right_hand_mocap_pos = interface.getRightHandSitePosition();
//         Eigen::Matrix<double, 3, 3> right_hand_mocap_ori = interface.getRightHandSiteOrientation();
//         Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//         right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori;
//         right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
//         Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = interface.getRightArmJointPosition();
//         Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
//         if ( right_arm_model.getInverseKinematics(ik_result, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
//         {
//             if ( right_arm_model.getDampedLeastSquareInverseKinematics(ik_result, right_arm_model, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
//             {
//                 LOG_WARN("right arm inverse kinematics fails.");
//                 return ;
//             }
//         }
//         auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
//         auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
//             std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
//             target_right_arm_joint_pos, ik_result);
//         right_arm_trajectory_buffer.write(time_point, trajectory);                
//     };

//     auto update_control = [&]()
//     {
//         std::array<double, NumDof> left_arm_target_joint_pos, right_arm_target_joint_pos;
//         {
//             auto [target_joint_pos, target_joint_vel, target_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
//             for ( int i=0 ; i<NumDof ; i++ )
//                 left_arm_target_joint_pos[i] = target_joint_pos(i);
//             JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
//             JointState<double, NumDof> current_joint_state = {interface.getLeftArmJointPosition(), interface.getLeftArmJointVelocity(), interface.getLeftArmJointAcceleration()};
//             auto target_joint_torque = controller.computeTorqueControlOutput(left_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
//             interface.setLeftArmJointControl(target_joint_torque);
//             LOG_TRACE("left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
//             LOG_TRACE("left arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
//             LOG_TRACE("left arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
//             LOG_TRACE("left arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
//             LOG_TRACE("left arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
//                 current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
//             LOG_TRACE("left arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
//                 current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));
//         }
//         {
//             auto [target_joint_pos, target_joint_vel, target_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
//             for ( int i=0 ; i<NumDof ; i++ )
//                 right_arm_target_joint_pos[i] = target_joint_pos(i);
//             JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
//             JointState<double, NumDof> current_joint_state = {interface.getRightArmJointPosition(), interface.getRightArmJointVelocity(), interface.getRightArmJointAcceleration()};
//             auto target_joint_torque = controller.computeTorqueControlOutput(right_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
//             interface.setRightArmJointControl(target_joint_torque);
//             LOG_TRACE("right arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
//             LOG_TRACE("right arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
//             LOG_TRACE("right arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
//             LOG_TRACE("right arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
//             LOG_TRACE("right arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
//                 current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
//             LOG_TRACE("right arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
//                 current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));            
//         }
//         msg_sender.sendDualArmJointPosition<double>(true, left_arm_target_joint_pos, right_arm_target_joint_pos);
//     };

//     struct timespec wakeup_time = {0, 0};
//     struct timespec cycletime = {0, 0}; // Initialize to zero
    
//     // Convert frequency (Hz) to timespec
//     double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
//     cycletime.tv_sec = static_cast<time_t>(period_seconds);
//     cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

//     clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
//     std::size_t count = 0;
//     while ( !TerminationHandler::stop_requested )
//     {
//         increaseTimeSpec(&wakeup_time, &cycletime);
//         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

//         update_control();
        
//         count++;

//         if ( count == 10 )
//         {
//             update_plan_left_arm();
//             update_plan_right_arm();

//             count = 0;
//         }
//     }

//     msg_sender.stop();
//     interface.stop();

//     spdlog::drop_all();

//     return 0;
// }
