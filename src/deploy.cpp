/**
 * @file main.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Main program of hardware control.
 * @version 0.1
 * @date 2025-07-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <termination.h>
#include <arm_model.h>
#include <arm_hardware_interface.h>
#include <arm_planner.h>
#include <teleop_task_runner.h>

int main(int argv, char** argc)
{
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested.store(false);
    /* Register SIGINT handler. */
    TerminationHandler::setup();
    /* Create simulation interface. */
    std::shared_ptr<ArmHardwareInterface> interface;
    /* Create task runner. */
    TeleopTaskRunner runner(interface, 20, 200);
    /* Infinite loop. */
    runner.run();
    return 0;
}