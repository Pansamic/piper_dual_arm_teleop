/**
 * @file arm_interface.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm communication and packet parser.
 * @version 0.1
 * @date 2025-05-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <unistd.h>
#include <poll.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <log.h>
#include <arm_interface.h>

/**
 * @brief Initialize SocketCAN
 * 
 */
ArmInterface::ArmInterface(const char* can_interface)
{
    can_socket_ = -1;
    
    joint_pos_.setZero();
    joint_vel_.setZero();
    joint_torque_.setZero();

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_ifr_.ifr_name, can_interface);
    ioctl(can_socket_, SIOCGIFINDEX, &can_ifr_);
    can_addr_.can_family = AF_CAN;
    can_addr_.can_ifindex = can_ifr_.ifr_ifindex;
    bind(can_socket_, (struct sockaddr *)&can_addr_, sizeof(can_addr_));

    /* Set interface running flag. */
    running_ = 1;

    /* Start listening on actuator feedback information */
    read_thread_ = std::thread(&ArmInterface::can_receive_thread, this);

    /* Enable all actuators */
    int actuator_enabled = 0;
    for(int i=0 ; i<20 ; i++)
    {
        enableActuators();
        sleep(0.1);
        if (!(actuator_status_[0] & ACTUATOR_STATUS_DISABLED) &&
            !(actuator_status_[1] & ACTUATOR_STATUS_DISABLED) &&
            !(actuator_status_[2] & ACTUATOR_STATUS_DISABLED) &&
            !(actuator_status_[3] & ACTUATOR_STATUS_DISABLED) &&
            !(actuator_status_[4] & ACTUATOR_STATUS_DISABLED) &&
            !(actuator_status_[5] & ACTUATOR_STATUS_DISABLED) )
        {
            actuator_enabled = 1;
            LOG_INFO("Actuators enabled.");
            break;
        }
    }
    if ( !actuator_enabled )
    {
        LOG_ERROR("Failed to enable actuators. Exiting...");
        exit(EXIT_FAILURE);
    }
}

ArmInterface::~ArmInterface()
{
    setJointPositionMode();
    // int actuator_disabled = 0;
    // for(int i=0 ; i<20 ; i++)
    // {
    //     disableActuators();
    //     sleep(0.1);
    //     if ((actuator_status_[0] & ACTUATOR_STATUS_DISABLED) &&
    //         (actuator_status_[1] & ACTUATOR_STATUS_DISABLED) &&
    //         (actuator_status_[2] & ACTUATOR_STATUS_DISABLED) &&
    //         (actuator_status_[3] & ACTUATOR_STATUS_DISABLED) &&
    //         (actuator_status_[4] & ACTUATOR_STATUS_DISABLED) &&
    //         (actuator_status_[5] & ACTUATOR_STATUS_DISABLED) )
    //     {
    //         actuator_disabled = 1;
    //         LOG_INFO("Actuators disabled.");
    //         break;
    //     }
    // }
    // if ( !actuator_disabled )
    // {
    //     LOG_ERROR("Failed to disable actuators.");
    // }
    if (can_socket_ != -1)
    {
        close(can_socket_);
        running_ = 0;
        read_thread_.join();
    }
    LOG_INFO("Arm Interface is terminated.");
}

void ArmInterface::can_receive_thread()
{
    struct can_frame frame;
    struct pollfd fds;

    fds.fd = can_socket_;
    fds.events = POLLIN;
    
    while (running_)
    {
        /* poll until data received */
        int ret = poll(&fds, 1, -1);
        if ( ret == -1 )
        {
            LOG_ERROR("CAN receipt poll error: {}", strerror(errno));
        }
        if ( fds.revents & POLLIN )
        {
            int recvbytes = read(can_socket_, &frame, sizeof(struct can_frame));
            if (recvbytes > 0)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                /* parse feedback frame data */
                parseCanFrame(&frame);
            }
            else if (recvbytes < 0)
            {
                LOG_ERROR("CAN read error: {}", strerror(errno));
                break;
            }
        }
    }
}

void ArmInterface::setMITMode() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_MIT;   // MIT move mode.
    frame.data[2] = 50;     // Move rate.
    frame.data[3] = 0xAD;   // MIT mode.
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setJointPositionMode() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_JOINT; // Joint move mode.
    frame.data[2] = 100;    // Move rate.
    frame.data[3] = 0x00;   // Joint position control.
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setEndEffectorPoseMode() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_POSITION; // End effector move mode.
    frame.data[2] = 100;    // Move rate.
    frame.data[3] = 0x00;   // Joint position control.
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::parseCanFrame(struct can_frame* msg)
{
    /* status */
    if ( msg->can_id == 0x2A1 )
    {
        if ( msg->data[0] != MODE_CAN )
        {
            LOG_WARN("CAN mode error, expected MODE_CAN but got 0x{:02x}", msg->data[0]);
        }
        switch ( msg->data[1] )
        {
        case STATUS_EMERGENCY_STOP:
            LOG_WARN("Emergency stop triggered.");
            break;
        case STATUS_NO_SOLUTION:
            LOG_WARN("No solution found.");
            break;
        case STATUS_SIGULAR:
            LOG_WARN("Singularity encountered.");
            break;
        case STATUS_OVER_POSITION_LIMIT:
            LOG_WARN("Joint over position limit.");
            break;
        case STATUS_JOINT_COMMUNICATION_ERROR:
            LOG_WARN("Joint communication error.");
            break;
        case STATUS_JOINT_BRAKE_UNRELEASED:
            LOG_WARN("Joint brake unreleased.");
            break;
        case STATUS_SELF_COLLISION:
            LOG_WARN("Manipulator self collision");
            break;
        case STATUS_TEACH_OVERSPEED:
            LOG_WARN("Teach mode overspeed.");
            break;
        case STATUS_JOINT_EXCEPTION:
            LOG_WARN("Joint unexpected error.");
            break;
        case STATUS_TEACH_RECORD:
            LOG_DEBUG("Teach mode recording.");
            break;
        case STATUS_TEACH_EXECUTE:
            LOG_DEBUG("Teach mode executing.");
            break;
        case STATUS_TEACH_SUSPEND:
            LOG_INFO("Teach mode suspended.");
            break;
        case STATUS_NTC_OVER_TEMPERATURE:
            LOG_WARN("NTC over temperature");
            break;
        case STATUS_RELEASE_RES_OVER_TEMPERATURE:
            LOG_WARN("Release resistor over temperature.");
            break;
        default:
            break;
        }

        switch ( msg->data[2] )
        {
        case MOVE_MODE_POSITION:
            LOG_DEBUG("Feedback Position Mode.");
            break;
        case MOVE_MODE_JOINT:
            LOG_DEBUG("Feedback Joint Mode.");
            break;
        case MOVE_MODE_LINEAR:
            LOG_DEBUG("Feedback Linear Mode.");
            break;
        case MOVE_MODE_CIRCULAR:
            LOG_DEBUG("Feedback Circular Mode.");
            break;
        case MOVE_MODE_MIT:
            LOG_DEBUG("Feedback MIT Mode.");
            break;
        }

        if ( msg->data[6] & (1<<0) )
        {
            LOG_ERROR("Joint 1 over position limit.");
        }
        if ( msg->data[6] & (1<<1) )
        {
            LOG_ERROR("Joint 2 over position limit.");
        }
        if ( msg->data[6] & (1<<2) )
        {
            LOG_ERROR("Joint 3 over position limit.");
        }
        if ( msg->data[6] & (1<<3) )
        {
            LOG_ERROR("Joint 4 over position limit.");
        }
        if ( msg->data[6] & (1<<4) )
        {
            LOG_ERROR("Joint 5 over position limit.");
        }
        if ( msg->data[6] & (1<<5) )
        {
            LOG_ERROR("Joint 6 over position limit.");
        }

        if ( msg->data[7] & (1<<0) )
        {
            LOG_ERROR("Joint 1 communication error.");
        }
        if ( msg->data[7] & (1<<1) )
        {
            LOG_ERROR("Joint 2 communication error.");
        }
        if ( msg->data[7] & (1<<2) )
        {
            LOG_ERROR("Joint 3 communication error.");
        }
        if ( msg->data[7] & (1<<3) )
        {
            LOG_ERROR("Joint 4 communication error.");
        }
        if ( msg->data[7] & (1<<4) )
        {
            LOG_ERROR("Joint 5 communication error.");
        }
        if ( msg->data[7] & (1<<5) )
        {
            LOG_ERROR("Joint 6 communication error.");
        }
        return ;
    }

    /* actuator high speed feedback */
    if ( msg->can_id >= 0x251 && msg->can_id <= 0x256 )
    {
        uint16_t actuator_id = msg->can_id - 0x251;
        int16_t velocity = msg->data[0] << 8 | msg->data[1];
        uint16_t current = msg->data[2] << 8 | msg->data[3];
        int32_t position = msg->data[4] << 24 | msg->data[5] << 16 | msg->data[6] << 8 | msg->data[7];
        joint_vel_(actuator_id) = static_cast<double>(velocity) / 1000.0;
        joint_pos_(actuator_id) = static_cast<double>(position) / 1000.0; // Warn: haven't checked this scaler.
        if ( actuator_id == 0 || actuator_id == 1 || actuator_id == 2 )
        {
            joint_torque_(actuator_id) = static_cast<double>(current) / 1000.0 * 1.18125;
        }
        else if ( actuator_id == 3 || actuator_id == 4 || actuator_id == 5 )
        {
            joint_torque_(actuator_id) = static_cast<double>(current) / 1000.0 * 0.95844;
        }
        return ;
    }

    /* actuator low speed feedback */
    if ( msg->can_id >= 0x261 && msg->can_id <= 0x266 )
    {
        uint16_t actuator_id = msg->can_id - 0x261;
        uint16_t voltage = (msg->data[0] << 8) | msg->data[1];
        actuator_voltage_[actuator_id] = static_cast<float>(voltage);
        actuator_temperature_[actuator_id] = (msg->data[2] << 8) | msg->data[3];
        motor_temperature_[actuator_id] = msg->data[4];
        actuator_status_[actuator_id] = msg->data[5];
        return ;
    }
}

void ArmInterface::sendControlMessage(
    const uint8_t actuator_id,
    const int16_t position,
    const int16_t velocity,
    const int16_t kp,
    const int16_t kd,
    const int8_t torque) const
{
    struct can_frame frame;
    uint8_t crc = 0;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x15A + actuator_id;
    frame.can_dlc = 8;
    frame.data[0] = (position >> 8);
    frame.data[1] = (position & 0xFF);
    frame.data[2] = (velocity >> 4);
    frame.data[3] = (((velocity & 0x0F) << 4)|kp >> 8);
    frame.data[4] = (kp & 0xFF);
    frame.data[5] = (kd >> 4);
    frame.data[6] = (((kd & 0x0F) << 4)|(torque >> 4));
    crc = (frame.data[0]^frame.data[1]^frame.data[2]^frame.data[3]^frame.data[4]^frame.data[5]^frame.data[6]) & 0x0F;
    frame.data[7] = ((torque << 4) | crc);
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::enableActuators() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x471;
    frame.can_dlc = 8;
    frame.data[0] = 7; // Select all the actuators.
    frame.data[1] = 0x02; // Enable
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::disableActuators() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x471;
    frame.can_dlc = 8;
    frame.data[0] = 7; // Select all the actuators.
    frame.data[1] = 0x01; // Disable
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setActuatorControl(
    const Eigen::Vector<double,6>& joint_pos,
    const Eigen::Vector<double,6>& joint_vel,
    const Eigen::Vector<double,6>& joint_torque) const
{
    auto double2int16 = [](double x, double min, double max, uint8_t bits) -> int16_t
    {
        return static_cast<int16_t>((x-min)*(static_cast<double>((1<<bits)-1)/(max-min))); 
    };
    auto double2int8 = [](double x, double min, double max, uint8_t bits) -> int8_t
    {
        return static_cast<int8_t>((x-min)*(static_cast<double>((1<<bits)-1)/(max-min))); 
    };
    setMITMode();
    for ( int i=0 ; i<6 ; i++ )
    {
        sendControlMessage(i,
            double2int16(joint_pos(i),-12.5,12.5,16),
            double2int16(joint_vel(i),-45.0,45.0,12),
            double2int16(2.0,0.0,500.0,12),
            double2int16(0.15,-5,5.0,12),
            double2int8(joint_torque(i),-18.0,18.0,8)
       );
    }
}

void ArmInterface::setActuatorTorque(const Eigen::Vector<double,6>& joint_torque) const
{
    auto double2int8 = [](double x, double min, double max, uint8_t bits) -> int8_t
    {
        return static_cast<int8_t>((x-min)*(static_cast<double>((1<<bits)-1)/(max-min))); 
    };
    setMITMode();
    for ( int i=0 ; i<6 ; i++ )
    {
        sendControlMessage(i,0,0,0,0,double2int8(joint_torque(i),-18.0,18.0,8)); // Warn: torque scaler unconfirmed.
    }
}

void ArmInterface::setActuatorPosition(const Eigen::Vector<double,6>& joint_pos) const
{
    struct can_frame frame;
    int32_t joint_pos_scaled;
    
    /* Set default joint position control mode. */
    setJointPositionMode();

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x155;
    frame.can_dlc = 8;
    joint_pos_scaled = static_cast<int32_t>(joint_pos(0) * 180.0 / M_PI * 1000.0);
    frame.data[0] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[1] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[2] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[3] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    joint_pos_scaled = static_cast<int32_t>(joint_pos(1) * 180.0 / M_PI * 1000.0);
    frame.data[4] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[5] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[6] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[7] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x156;
    frame.can_dlc = 8;
    joint_pos_scaled = static_cast<int32_t>(joint_pos(2) * 180.0 / M_PI * 1000.0);
    frame.data[0] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[1] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[2] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[3] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    joint_pos_scaled = static_cast<int32_t>(joint_pos(3) * 180.0 / M_PI * 1000.0);
    frame.data[4] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[5] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[6] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[7] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x157;
    frame.can_dlc = 8;
    joint_pos_scaled = static_cast<int32_t>(joint_pos(4) * 180.0 / M_PI * 1000.0);
    frame.data[0] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[1] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[2] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[3] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    joint_pos_scaled = static_cast<int32_t>(joint_pos(5) * 180.0 / M_PI * 1000.0);
    frame.data[4] = static_cast<uint8_t>(joint_pos_scaled >> 24);
    frame.data[5] = static_cast<uint8_t>(joint_pos_scaled >> 16);
    frame.data[6] = static_cast<uint8_t>(joint_pos_scaled >> 8);
    frame.data[7] = static_cast<uint8_t>(joint_pos_scaled & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setEndEffectorPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation) const
{
    struct can_frame frame;
    int32_t x = static_cast<int32_t>(pos(0) * 1000000.0);
    int32_t y = static_cast<int32_t>(pos(1) * 1000000.0);
    int32_t z = static_cast<int32_t>(pos(2) * 1000000.0);
    Eigen::Matrix3d rotation_matrix = orientation.toRotationMatrix();
    Eigen::Vector3d rpy = rotation_matrix.eulerAngles(2,1,0);
    int32_t roll = static_cast<int32_t>(rpy(2) * 180 / M_PI * 1000.0);
    int32_t pitch = static_cast<int32_t>(rpy(1) * 180 / M_PI * 1000.0);
    int32_t yaw = static_cast<int32_t>(rpy(0) * 180 / M_PI * 1000.0);
    // std::cout << "rotation matrix" << std::endl << rotation_matrix << std::endl;
    // std::cout << "roll:" << rpy(2) << " pitch:" << rpy(1) << " yaw:" << rpy(0) << std::endl;
    
    setEndEffectorPoseMode();

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x152;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(x >> 24);
    frame.data[1] = static_cast<uint8_t>(x >> 16);
    frame.data[2] = static_cast<uint8_t>(x >> 8);
    frame.data[3] = static_cast<uint8_t>(x & 0xFF);
    frame.data[4] = static_cast<uint8_t>(y >> 24);
    frame.data[5] = static_cast<uint8_t>(y >> 16);
    frame.data[6] = static_cast<uint8_t>(y >> 8);
    frame.data[7] = static_cast<uint8_t>(y & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x153;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(z >> 24);
    frame.data[1] = static_cast<uint8_t>(z >> 16);
    frame.data[2] = static_cast<uint8_t>(z >> 8);
    frame.data[3] = static_cast<uint8_t>(z & 0xFF);
    frame.data[4] = static_cast<uint8_t>(roll >> 24);
    frame.data[5] = static_cast<uint8_t>(roll >> 16);
    frame.data[6] = static_cast<uint8_t>(roll >> 8);
    frame.data[7] = static_cast<uint8_t>(roll & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x154;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(pitch >> 24);
    frame.data[1] = static_cast<uint8_t>(pitch >> 16);
    frame.data[2] = static_cast<uint8_t>(pitch >> 8);
    frame.data[3] = static_cast<uint8_t>(pitch & 0xFF);
    frame.data[4] = static_cast<uint8_t>(yaw >> 24);
    frame.data[5] = static_cast<uint8_t>(yaw >> 16);
    frame.data[6] = static_cast<uint8_t>(yaw >> 8);
    frame.data[7] = static_cast<uint8_t>(yaw & 0xFF);
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setGripperCurrentAsZero() const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x159;
    frame.can_dlc = 8;
    frame.data[7] = 0xAE;
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ArmInterface::setGripperControl(const double position, const double torque) const
{
    struct can_frame frame;
    int32_t position_scaled = static_cast<int32_t>(position * 1000000.0);
    int16_t torque_scaled = static_cast<int16_t>(torque * 1000.0);
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x159;
    frame.can_dlc = 8;
    frame.data[0] = (uint8_t)(position_scaled >> 24);
    frame.data[1] = (uint8_t)(position_scaled >> 16);
    frame.data[2] = (uint8_t)(position_scaled >> 8);
    frame.data[3] = (uint8_t)(position_scaled & 0xFF);
    frame.data[4] = (uint8_t)(torque_scaled >> 8);
    frame.data[5] = (uint8_t)(torque_scaled & 0xFF);
    frame.data[6] = 0x01; // enable gripper by setting bit0 to 1.
    write(can_socket_, &frame, sizeof(struct can_frame));
}
