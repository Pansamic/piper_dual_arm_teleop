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

    program.add_argument("-d", "--device")
        .help("device interface name. e.g., can1");
    program.add_argument("-c", "--control")
        .help("enable disable zero");
    program.add_argument("-r", "--read");
    program.add_argument("-s", "--set");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }
    if ( !program.is_used("-d") )
    {
        LOG_ERROR("\'-d\' is required");
        std::cout << program << std::endl;
        return 1;
    }
    PiperInterface<double> interface(program.get("-d"));
    interface.initCan();
    interface.listen();

    std::string control_function = program.get<std::string>("-c");
    if ( control_function == "enable" )
    {
        interface.enableAllMotorsUntilConfirmed(20);
    }
    else if ( control_function == "disable" )
    {
        interface.disableAllMotorsUntilConfirmed(20);
    }
    else if ( control_function == "zero" )
    {
        for ( int i=0 ; i<20 ; i++ )
        {
            interface.setControlMode(PiperInterface<double>::MoveMode::MOVE_J);
            interface.setJointPosition({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    }

    interface.stop();

    return 0;
}
