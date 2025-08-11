#include <chrono>
#include <thread>
#include <iostream>
#include <config.h>
#include <utils.hpp>
#include <error_codes.h>
#include <piper_model.hpp>
#include <trajectory_buffer.hpp>
#include <arm_planner.hpp>
#include <arm_controller.hpp>
#include <mujoco_backend.hpp>

class TaskRunner
{
public:
    static constexpr std::size_t NumLink = 7;
    static constexpr std::size_t NumDof = 6;
    explicit TaskRunner()
    :   left_arm_model(CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z, CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW),
        right_arm_model(CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z, CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW),
        controller(1.0/CONFIG_CONTROLLER_FREQUENCY, {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}, {0, 0, 0, 0, 0, 0}, {10.0, 10.0, 10.0, 10.0, 10.0}){}
    ~TaskRunner() = default;
    void updateLeftArmPlan(Eigen::Matrix<double, 4, 4> pose)
    {
        Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = mj_backend.getLeftArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( left_arm_model.getInverseKinematics(ik_result, pose, actual_left_arm_joint_pos) == ErrorCode::NoResult)
        {
            if ( left_arm_model.getDampedLeastSquareInverseKinematics(ik_result, left_arm_model, pose, actual_left_arm_joint_pos) == ErrorCode::NoResult )
            {
                return ;
            }
        }
        std::cout << "left arm inverse kinematics: " << ik_result.transpose() << std::endl;
        auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_left_arm_joint_pos, ik_result);
        left_arm_trajectory_buffer.write(time_point, trajectory);
    }
    void updateRightArmPlan(Eigen::Matrix<double, 4, 4> pose)
    {
        Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = mj_backend.getRightArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( right_arm_model.getInverseKinematics(ik_result, pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
        {
            if ( right_arm_model.getDampedLeastSquareInverseKinematics(ik_result, right_arm_model, pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
            {
                return ;
            }
        }
        std::cout << "right arm inverse kinematics: " << ik_result.transpose() << std::endl;
        auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_right_arm_joint_pos, ik_result);
        right_arm_trajectory_buffer.write(time_point, trajectory);                
    };
    void updateControl()
    {
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {mj_backend.getLeftArmJointPosition(), mj_backend.getLeftArmJointVelocity(), mj_backend.getLeftArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(left_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            mj_backend.setLeftArmJointControl(target_joint_torque);
            std::cout << "target left arm joint position: " << target_joint_pos.transpose() << std::endl;
            std::cout << "current left arm joint position: " << current_joint_state.joint_pos.transpose() << std::endl;
        }
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {mj_backend.getRightArmJointPosition(), mj_backend.getRightArmJointVelocity(), mj_backend.getRightArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(right_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            mj_backend.setRightArmJointControl(target_joint_torque);
            std::cout << "target right arm joint position: " << target_joint_pos.transpose() << std::endl;
            std::cout << "current right arm joint position: " << current_joint_state.joint_pos.transpose() << std::endl;
        }
    }
    MujocoBackend<double, NumDof> mj_backend;
    PiperArmModel<double> left_arm_model;
    PiperArmModel<double> right_arm_model;
    ArmController<double, NumLink, NumDof> controller;
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;
};

int main(void)
{
    TaskRunner runner;
    
    bool terminate = false;

    Eigen::Matrix4d left_hand_pose;
    left_hand_pose << 0,0,1,0.8,0,1,0,0.2,-1,0,0,0.8,0,0,0,1;
    Eigen::Matrix4d right_hand_pose;
    right_hand_pose << 0,0,1,0.8,0,1,0,-0.2,-1,0,0,0.8,0,0,0,1;

    std::thread plan_thread([&]()
    {
        struct timespec wakeup_time = {0, 0};
        struct timespec cycletime = {0, 0}; // Initialize to zero
        
        // Convert frequency (Hz) to timespec
        double period_seconds = 1.0 / static_cast<double>(CONFIG_PLANNER_FREQUENCY);
        cycletime.tv_sec = static_cast<time_t>(period_seconds);
        cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        std::size_t count = 0;
        while ( !terminate )
        {
            increaseTimeSpec(&wakeup_time, &cycletime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

            runner.updateLeftArmPlan(left_hand_pose);
            runner.updateRightArmPlan(right_hand_pose);
        }
    });
    std::thread control_thread([&]()
    {
        struct timespec wakeup_time = {0, 0};
        struct timespec cycletime = {0, 0}; // Initialize to zero
        
        // Convert frequency (Hz) to timespec
        double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
        cycletime.tv_sec = static_cast<time_t>(period_seconds);
        cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        std::size_t count = 0;
        while ( !terminate )
        {
            increaseTimeSpec(&wakeup_time, &cycletime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

            runner.updateControl();
        }
    });
    std::thread physics_thread([&]()
    {
        while( !terminate )
        {
            runner.mj_backend.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(5));

    terminate = true;

    plan_thread.join();
    control_thread.join();
    physics_thread.join();

    return 0;
}