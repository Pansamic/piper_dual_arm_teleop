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
#include <piper_interface.hpp>

int main(void)
{
    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "client");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested = false;
    /* Register SIGINT handler. */
    TerminationHandler::setup();

    Messenger<ChannelMode::UDP> msg_receiver(CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT, CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT);
    // Messenger<ChannelMode::Unix> msg_receiver(CONFIG_CLIENT_MSG_QUEUE, CONFIG_SERVER_MSG_QUEUE);
    msg_receiver.start(false, true);

    PiperInterface<double> left_arm_interface("can1");
    PiperInterface<double> right_arm_interface("can2");
    left_arm_interface.initCan();
    right_arm_interface.initCan();
    std::thread left_arm_interface_query_thread([&left_arm_interface](){
        while(!TerminationHandler::stop_requested){left_arm_interface.queryState();}});
    std::thread right_arm_interface_query_thread([&right_arm_interface](){
        while(!TerminationHandler::stop_requested){right_arm_interface.queryState();}});
    
    LOG_INFO("start to enable actuators.");
    static const int trails = 100;
    bool left_arm_enabled = false;
    bool right_arm_enabled = false;
    for ( int i=0 ; i<trails ; i++ )
    {
        left_arm_interface.enableAllMotors();
        right_arm_interface.enableAllMotors();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::array<bool, 6> left_arm_enable_status, right_arm_enable_status;
        for ( int i=0 ; i<6 ; i++ )
        {
            left_arm_enable_status[i] = left_arm_interface.getMotorEnabled(i);
            right_arm_enable_status[i] = right_arm_interface.getMotorEnabled(i);
        }
        if ( !left_arm_enabled && std::all_of(left_arm_enable_status.begin(), left_arm_enable_status.end(), [&](const bool& val){return val==true;}) )
        {
            left_arm_enabled = true;
            LOG_INFO("left arm actuators are all enabled.");
        }
        else
        {
            LOG_INFO("left arm actuator enable status:[1]{},[2]{},[3]{},[4]{},[5]{},[6]{}",
                left_arm_enable_status[0], left_arm_enable_status[1], left_arm_enable_status[2],
                left_arm_enable_status[3], left_arm_enable_status[4], left_arm_enable_status[5]);
        }
        if ( !right_arm_enabled && std::all_of(right_arm_enable_status.begin(), right_arm_enable_status.end(), [&](const bool& val){return val==true;}) )
        {
            right_arm_enabled = true;
            LOG_INFO("right arm actuators are all enabled.");
        }
        else
        {
            LOG_INFO("right arm actuator enable status:[1]{},[2]{},[3]{},[4]{},[5]{},[6]{}",
                right_arm_enable_status[0], right_arm_enable_status[1], right_arm_enable_status[2],
                right_arm_enable_status[3], right_arm_enable_status[4], right_arm_enable_status[5]);
        }
        if ( left_arm_enabled && right_arm_enabled )
            break;
    }
    if ( !left_arm_enabled || !right_arm_enabled )
    {
        LOG_ERROR("failed to enable dual arms, exiting...");
        msg_receiver.stop();
        spdlog::drop_all();
        return 0;
    }

    struct timespec wakeup_time = {0, 0};
    struct timespec cycletime = {0, 0}; // Initialize to zero
    
    // Convert frequency (Hz) to timespec
    double period_seconds = 1.0 / static_cast<double>(100);
    cycletime.tv_sec = static_cast<time_t>(period_seconds);
    cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    std::size_t count = 0;
    while ( !TerminationHandler::stop_requested )
    {
        increaseTimeSpec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        left_arm_interface.queryState();
        right_arm_interface.queryState();

        bool enable = false;
        std::array<double, 6> left_arm_target_joint_pos, right_arm_target_joint_pos;
        if ( msg_receiver.recv(enable, left_arm_target_joint_pos, right_arm_target_joint_pos) )
        {
            if ( !enable )
            {
                continue;
            }
            left_arm_interface.setControlMode(PiperInterface<double>::MoveMode::MOVE_J);
            left_arm_interface.setJointPosition(left_arm_target_joint_pos);
            right_arm_interface.setControlMode(PiperInterface<double>::MoveMode::MOVE_J);
            right_arm_interface.setJointPosition(right_arm_target_joint_pos);

            LOG_INFO("receive left arm joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                left_arm_target_joint_pos[0], left_arm_target_joint_pos[1], left_arm_target_joint_pos[2],
                left_arm_target_joint_pos[3], left_arm_target_joint_pos[4], left_arm_target_joint_pos[5]);
            LOG_INFO("receive right arm joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                right_arm_target_joint_pos[0], right_arm_target_joint_pos[1], right_arm_target_joint_pos[2],
                right_arm_target_joint_pos[3], right_arm_target_joint_pos[4], right_arm_target_joint_pos[5]);
        }
    }

    msg_receiver.stop();

    spdlog::drop_all();
    return 0;
}