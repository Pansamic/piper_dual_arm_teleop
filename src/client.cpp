/**
 * @file client.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Real AgileX Piper robotic manipualtor control client.
 * @version 0.1
 * @date 2025-08-10
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
#include <piper_interface.hpp>
#include <piper_model.hpp>

class DualArmTeleopClient
{
public:
    static constexpr std::size_t NumLink = PiperArmModel<double>::NumLink;
    static constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;
    explicit DualArmTeleopClient()
    :   left_arm_model(CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z, CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW),
        right_arm_model(CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z, CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW),
        controller(1.0/CONFIG_CONTROLLER_FREQUENCY, {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}, {0, 0, 0, 0, 0, 0}, {10.0, 10.0, 10.0, 10.0, 10.0}){}
    ~DualArmTeleopClient() = default;
    bool initialize()
    {
        left_arm_interface.initCan();
        right_arm_interface.initCan();
        left_arm_interface.enterCANControlMode();
        right_arm_interface.enterCANControlMode();
        left_arm_interface.listen();
        right_arm_interface.listen();
        left_arm_interface.clearAllJointErrorCode();
        right_arm_interface.clearAllJointErrorCode();
        bool left_arm_enabled = left_arm_interface.enableAllMotorsUntilConfirmed(20);
        bool right_arm_enabled = right_arm_interface.enableAllMotorsUntilConfirmed(20);
        if ( !left_arm_enabled || !right_arm_enabled )
        {
            LOG_ERROR("failed to enable dual arms, exiting...");
            left_arm_interface.stop();
            right_arm_interface.stop();
            spdlog::drop_all();
            return false;
        }
        left_arm_interface.setCollisionProtectionLevel(0);
        left_arm_interface.setAllJointParameterAsDefault();
        right_arm_interface.setCollisionProtectionLevel(0);
        right_arm_interface.setAllJointParameterAsDefault();

        return true;
    }
    void stop()
    {
        left_arm_interface.stop();
        right_arm_interface.stop();
    }
    void updatePlan(
        const std::array<double, 3>& left_hand_position, const std::array<double, 4>& left_hand_orientation,
        const std::array<double, 3>& right_hand_position, const std::array<double, 4>& right_hand_orientation)
    {
        /* Rotation offset for Meta Quest VR controller pose */
        static const Eigen::Quaterniond offset(Eigen::AngleAxisd(M_PI * 3 / 4, Eigen::Vector3d::UnitY()));

        Eigen::Matrix4d left_hand_pose = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond left_hand_orientation_quat(left_hand_orientation[3], left_hand_orientation[0], left_hand_orientation[1], left_hand_orientation[2]);
        left_hand_orientation_quat = left_hand_orientation_quat * offset;
        left_hand_pose.block<3, 3>(0, 0) = left_hand_orientation_quat.toRotationMatrix();
        left_hand_pose(0, 3) = left_hand_position[0];
        left_hand_pose(1, 3) = left_hand_position[1];
        left_hand_pose(2, 3) = left_hand_position[2];
        updateArmPlanning(left_hand_pose, left_arm_model, left_arm_interface, left_arm_trajectory_buffer);

        Eigen::Matrix4d right_hand_pose = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond right_hand_orientation_quat(right_hand_orientation[3], right_hand_orientation[0], right_hand_orientation[1], right_hand_orientation[2]);
        right_hand_orientation_quat = right_hand_orientation_quat * offset;
        right_hand_pose.block<3, 3>(0, 0) = right_hand_orientation_quat.toRotationMatrix();
        right_hand_pose(0, 3) = right_hand_position[0];
        right_hand_pose(1, 3) = right_hand_position[1];
        right_hand_pose(2, 3) = right_hand_position[2];
        updateArmPlanning(right_hand_pose, right_arm_model, right_arm_interface, right_arm_trajectory_buffer);
    }
    void updateControl()
    {
        updateArmControl(left_arm_model, left_arm_interface, left_arm_trajectory_buffer);
        updateArmControl(right_arm_model, right_arm_interface, right_arm_trajectory_buffer);
    }
private:
    PiperInterface<double> left_arm_interface;
    PiperInterface<double> right_arm_interface;
    PiperArmModel<double> left_arm_model;
    PiperArmModel<double> right_arm_model;
    ArmController<double, NumLink, NumDof> controller;
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;

    void updateArmPlanning(Eigen::Matrix4d pose, const PiperArmModel<double>& model, const PiperInterface<double>& interface, BsplineTrajectoryBuffer<double, NumDof>& trajbuffer)
    {
        Eigen::Vector<double, NumDof> actual_arm_joint_pos(
            interface.getJointFeedbackAngle(0),
            interface.getJointFeedbackAngle(1),
            interface.getJointFeedbackAngle(2),
            interface.getJointFeedbackAngle(3),
            interface.getJointFeedbackAngle(4),
            interface.getJointFeedbackAngle(5));
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( model.getInverseKinematics(ik_result, pose, actual_arm_joint_pos) == ErrorCode::NoResult )
        {
            if ( model.getDampedLeastSquareInverseKinematics(ik_result, model, pose, actual_arm_joint_pos) == ErrorCode::NoResult )
            {
                return ;
            }
        }
        auto [target_arm_joint_pos, target_arm_joint_vel, target_arm_joint_acc] = trajbuffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_arm_joint_pos, ik_result);
        trajbuffer.write(time_point, trajectory);
    };

    void updateArmControl(const PiperArmModel<double>& model, const PiperInterface<double>& interface, BsplineTrajectoryBuffer<double, NumDof>& trajbuffer)
    {
        auto [target_joint_pos, target_joint_vel, target_joint_acc] = trajbuffer.interpolate(std::chrono::steady_clock::now());
        JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
        JointState<double, NumDof> current_joint_state;
        current_joint_state.joint_acc.setZero();
        current_joint_state.joint_torq.setZero();
        for ( std::size_t i=0 ; i<NumDof ; i++ )
        {
            current_joint_state.joint_pos(i) = interface.getJointFeedbackAngle(i);
            current_joint_state.joint_vel(i) = interface.getJointFeedbackVelocity(i);
        }
        auto target_joint_torque = controller.computeTorqueControlOutput(model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
        interface.setControlMode(PiperInterface<double>::MoveMode::MOVE_J);
        std::array<double, NumDof> target_joint_pos_vec;
        memcpy(target_joint_pos_vec.data(), current_joint_state.joint_pos.data(), NumDof);
        interface.setJointPosition(target_joint_pos_vec);
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
    
    Messenger<ChannelMode::UDP> messenger(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
    messenger.start(true, true);

    DualArmTeleopClient client;
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
    bool system_enable;
    std::array<double, 3> left_hand_position, right_hand_position;
    std::array<double, 4> left_hand_orientation, right_hand_orientation;
    while ( !TerminationHandler::stop_requested )
    {
        increaseTimeSpec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        if ( messenger.recvDualArmHandPose(system_enable, left_hand_position, left_hand_orientation, right_hand_position, right_hand_orientation) )
        {
            if ( !system_enable )
            {
                continue ;
            }
        }

        client.updatePlan(left_hand_position, left_hand_orientation, right_hand_position, right_hand_orientation);
    }

    control_thread.join();
    client.stop();

    spdlog::drop_all();
    return 0;
}