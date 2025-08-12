/**
 * @file pipermanager.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Toolset to monitor, configure and control AgileX Piper robotic manipulator.
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <argparse.hpp>
#include <piper_interface.hpp>


int main(int argc, char* argv[])
{
    // argparse::ArgumentParser program("pipermanager");

    // program.add_argument("--control");
    PiperInterface<double> interface("can1");
    interface.initCan();
    interface.listen();

    interface.enableAllMotors();

    interface.stop();

    return 0;
}
