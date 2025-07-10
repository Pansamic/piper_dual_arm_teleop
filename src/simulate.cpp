/**
 * @file simulate.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Main program of simulation.
 * @version 0.1
 * @date 2025-07-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <log.hpp>
#include <termination.h>
#include <arm_model.h>
#include <arm_simulation_interface.h>
#include <arm_planner.h>
#include <teleop_task_runner.h>

int main(int argv, char** argc)
{
    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "simulate");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested.store(false);
    /* Register SIGINT handler. */
    TerminationHandler::setup();
    /* Create simulation interface. */
    auto interface = std::make_shared<ArmSimulationInterface>(PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_position.xml");
    /* Create task runner. */
    TeleopTaskRunner runner(interface, 20, 200);
    /* Infinite loop. */
    runner.run();
    spdlog::drop_all();
    return 0;
}