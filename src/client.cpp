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

    Messenger<ChannelMode::UDP> msg_receiver(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
    // Messenger<ChannelMode::Unix> msg_receiver(CONFIG_CLIENT_MSG_QUEUE, CONFIG_SERVER_MSG_QUEUE);
    msg_receiver.start(false, true);

    PiperInterface<double> left_arm_interface("can1");
    PiperInterface<double> right_arm_interface("can2");
    left_arm_interface.initialize();
    right_arm_interface.initialize();

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