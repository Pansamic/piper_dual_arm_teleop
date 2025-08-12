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
    argparse::ArgumentParser program("pipermanager");

    program.add_argument("--enable").flag();
    program.add_argument("--disable").flag();
    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    PiperInterface<double> interface("can1");
    interface.initCan();
    interface.listen();

    if ( program.get<bool>("--enable") )
    {
        interface.enableAllMotors();
    }
    else if ( program.get<bool>("--disable") )
    {
        interface.disableAllMotors();
    }

    interface.stop();

    return 0;
}
