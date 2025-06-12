#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <memory>
#include <asio.hpp>
#include <arm_interface.h>
#include <arm_control.h>
#include <whole_body_state_msg.hpp>
#include <dual_arm_state_msg.hpp>
#include <udp_channel.hpp>
#include <log.h>

volatile sig_atomic_t terminate = 0;

void signal_handler(int sig)
{
    if (sig == SIGINT)
    {
        terminate = 1;
    }
}

void increase_time_spec(struct timespec* time, const struct timespec* increasement)
{
    time->tv_sec += increasement->tv_sec;
    time->tv_nsec += increasement->tv_nsec;

    if (time->tv_nsec >= 1000000000)
    {
        time->tv_sec++;
        time->tv_nsec -= 1000000000;
    }
}

int main(int argv, char** argc)
{
    // Setup signal handler for SIGINT (Ctrl+C)
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);

    Eigen::Quaterniond base_orientation;
    static const Eigen::Vector3d head_position(0.325024,0,0.80246);
    static const Eigen::Quaterniond left_hand_orientation_offset = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    static const Eigen::Quaterniond right_hand_orientation_offset(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));

    static const Eigen::Matrix4d left_arm_base_transform = (Eigen::Matrix4d() << 
        0, -0.707106781 ,  0.707106781 , 0.23805,
        0,  0.707106781 ,  0.707106781 , 0.19675,
        -1, 0           ,  0           , 0.74065,
        0,  0           ,  0           , 1       ).finished();
    Eigen::Vector3d left_hand_target_pos(0.542092536439244, 0.500792536439244, 0.398868333963670);
    Eigen::Quaterniond left_hand_target_orientation(0.000044497177102,0.382683431921232,-0.923879531439719,0.000018431334243);
    Eigen::Matrix4d left_hand_target_pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond left_hand_actual_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_target_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> left_arm_actual_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    double left_gripper_control = 0;

    static const Eigen::Matrix4d right_arm_base_transform = (Eigen::Matrix4d() <<
         0, 0.707106781, 0.707106781, 0.23805,
         0, 0.707106781,-0.707106781,-0.19675,
        -1, 0,           0,           0.74065,
         0, 0,           0,           1).finished();
    Eigen::Vector3d right_hand_target_pos(0.542092536439244,-0.500792536439244,0.398868333963670);
    Eigen::Quaterniond right_hand_target_orientation(0.000044497177102,-0.382683431921232,-0.923879531439719,-0.000018431334243);
    Eigen::Matrix4d right_hand_target_pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond right_hand_actual_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_target_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_vel = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    Eigen::Vector<double,ArmModel::num_dof_> right_arm_actual_joint_torque = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
    double right_gripper_control = 0;

    std::array<Eigen::Matrix4d,ArmModel::num_link_> link_transform;
    std::array<Eigen::Matrix4d,ArmModel::num_link_> link_com_transform;
    std::array<Eigen::Matrix<double,6,ArmModel::num_dof_>,ArmModel::num_link_> link_com_jacobian;
    std::array<Eigen::Matrix<double,6,ArmModel::num_dof_>,ArmModel::num_link_> link_com_jacobian_dot;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_lin_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_ang_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_com_lin_vel;
    std::array<Eigen::Vector3d,ArmModel::num_link_> link_com_ang_vel;
    Eigen::Matrix<double,ArmModel::num_dof_,ArmModel::num_dof_> generalized_mass_matrix;
    Eigen::Matrix<double,ArmModel::num_dof_,ArmModel::num_dof_> centrifugal_coriolis_matrix;
    Eigen::Vector<double,ArmModel::num_dof_> gravity_compensate;
    
    /* Initialize logger */
    std::filesystem::path logpath(PROJECT_PATH"/logs");
    initLogger(logpath, "main");

    std::unique_ptr<ArmInterface> left_arm_interface = std::make_unique<ArmInterface>("can0");
    std::unique_ptr<ArmInterface> right_arm_interface = std::make_unique<ArmInterface>("can1");
    std::unique_ptr<ArmModel> left_arm = std::make_unique<ArmModel>(left_arm_base_transform);
    std::unique_ptr<ArmModel> right_arm = std::make_unique<ArmModel>(right_arm_base_transform);

    asio::io_context whole_body_state_io_context;
    UdpChannelReceiver<WholeBodyStateMsg> whole_body_state_channel(whole_body_state_io_context, "192.168.31.16", 12345, "192.168.31.202", 54321);
    auto whole_body_state_receive_buffer = whole_body_state_channel.get_receiver_buffer();
    whole_body_state_channel.enable_receiver();
    std::thread whole_body_state_asio_thread([&whole_body_state_io_context]()
    {
        try
        {
            whole_body_state_io_context.run();
        }
        catch (asio::system_error& e)
        {
            LOG_ERROR("whole body state asio context error: {}",e.what());
        }
    });

    asio::io_context dual_arm_state_io_context;
    UdpChannelSender<DualArmStateMsg> dual_arm_state_channel(dual_arm_state_io_context, "192.168.31.202", 54321);
    std::shared_ptr<RingBuffer<std::shared_ptr<DualArmStateMsg::Definition>>> dual_arm_state_buffer = std::make_shared<RingBuffer<std::shared_ptr<DualArmStateMsg::Definition>>>(128);
    dual_arm_state_channel.register_sender_buffer(dual_arm_state_buffer);
    dual_arm_state_channel.enable_sender();
    std::thread dual_arm_state_asio_thread([&dual_arm_state_io_context]()
    {
        try
        {
            dual_arm_state_io_context.run();
        }
        catch (asio::system_error& e)
        {
            LOG_ERROR("dual arm state asio context error: {}",e.what());
        }
    });

    // Set thread as maximum priority.
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    struct timespec wakeup_time = {0,0};
    static const struct timespec cycletime = {0, 5000000};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while (!terminate)
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        // Get high-resolution timestamp
        // auto now = std::chrono::high_resolution_clock::now();
        // auto duration = now.time_since_epoch();
        // auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        // std::cout << "Timestamp: " << microseconds << " us" << std::endl;

        left_arm_actual_joint_pos = left_arm_interface->getActuatorsPosition();
        left_arm_actual_joint_vel = left_arm_interface->getActuatorsVelocity();
        left_arm_actual_joint_torque = left_arm_interface->getActuatorsTorque();

        right_arm_actual_joint_pos = right_arm_interface->getActuatorsPosition();
        right_arm_actual_joint_vel = right_arm_interface->getActuatorsVelocity();
        right_arm_actual_joint_torque = right_arm_interface->getActuatorsTorque();

        LOG_DEBUG("left arm actual joint pos:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            left_arm_actual_joint_pos(0), left_arm_actual_joint_pos(1), left_arm_actual_joint_pos(2), left_arm_actual_joint_pos(3), left_arm_actual_joint_pos(4), left_arm_actual_joint_pos(5));
        LOG_DEBUG("left arm actual joint vel:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            left_arm_actual_joint_vel(0), left_arm_actual_joint_vel(1), left_arm_actual_joint_vel(2), left_arm_actual_joint_vel(3), left_arm_actual_joint_vel(4), left_arm_actual_joint_vel(5));
        LOG_DEBUG("left arm actual joint torque:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            left_arm_actual_joint_torque(0), left_arm_actual_joint_torque(1), left_arm_actual_joint_torque(2), left_arm_actual_joint_torque(3), left_arm_actual_joint_torque(4), left_arm_actual_joint_torque(5));

        LOG_DEBUG("right arm actual joint pos:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            right_arm_actual_joint_pos(0), right_arm_actual_joint_pos(1), right_arm_actual_joint_pos(2), right_arm_actual_joint_pos(3), right_arm_actual_joint_pos(4), right_arm_actual_joint_pos(5));
        LOG_DEBUG("right arm actual joint vel:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            right_arm_actual_joint_vel(0), right_arm_actual_joint_vel(1), right_arm_actual_joint_vel(2), right_arm_actual_joint_vel(3), right_arm_actual_joint_vel(4), right_arm_actual_joint_vel(5));
        LOG_DEBUG("right arm actual joint torque:{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f},{:7.4f}",
            right_arm_actual_joint_torque(0), right_arm_actual_joint_torque(1), right_arm_actual_joint_torque(2), right_arm_actual_joint_torque(3), right_arm_actual_joint_torque(4), right_arm_actual_joint_torque(5));

        while ( whole_body_state_receive_buffer->size() > 0 )
        {
            std::shared_ptr<WholeBodyStateMsg::Definition> cmd = whole_body_state_receive_buffer->pop();

            base_orientation = cmd->base_quat.cast<double>();
            left_hand_target_pos = cmd->left_hand_pos;
            right_hand_target_pos = cmd->right_hand_pos;
            left_hand_target_orientation = cmd->left_hand_quat.cast<double>() * left_hand_orientation_offset ;
            right_hand_target_orientation = cmd->right_hand_quat.cast<double>() * right_hand_orientation_offset ;

            left_hand_target_pos = left_hand_target_pos + head_position;
            right_hand_target_pos = right_hand_target_pos + head_position;

            left_gripper_control = cmd->left_gripper_ctrl;
            right_gripper_control = cmd->right_gripper_ctrl;
        }

        left_hand_target_pose.setIdentity();
        left_hand_target_pose.block<3,3>(0,0) = left_hand_target_orientation.toRotationMatrix();
        left_hand_target_pose.block<3,1>(0,3) = left_hand_target_pos;
        LOG_DEBUG("left arm target end effector pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
            left_hand_target_pos(0), left_hand_target_pos(1), left_hand_target_pos(2),
            left_hand_target_orientation.w(), left_hand_target_orientation.x(), left_hand_target_orientation.y(), left_hand_target_orientation.z());

        right_hand_target_pose.setIdentity();
        right_hand_target_pose.block<3,3>(0,0) = right_hand_target_orientation.toRotationMatrix();
        right_hand_target_pose.block<3,1>(0,3) = right_hand_target_pos;
        LOG_DEBUG("right arm target end effector pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
            right_hand_target_pos(0), right_hand_target_pos(1), right_hand_target_pos(2),
            right_hand_target_orientation.w(), right_hand_target_orientation.x(), right_hand_target_orientation.y(), right_hand_target_orientation.z());

        try
        {
            Eigen::Vector<double,ArmModel::num_dof_> temp_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
            // temp_joint_pos.block<3,1>(0,0) = left_arm->getShoulderJointPos(left_hand_target_pose, left_arm_actual_joint_pos.block<3,1>(0,0));
            // temp_joint_pos = left_arm->getInverseKinematics(left_hand_target_pose,left_arm_actual_joint_pos);
            temp_joint_pos = left_arm->getDampedLeastSquareInverseKinematics(
                0.1, Eigen::Vector<double,6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, left_hand_target_pose, left_arm_actual_joint_pos);
            if ( temp_joint_pos(0) > M_PI/4 )
            {
                throw std::runtime_error("left joint 1 position unreasonable");
            }
            left_arm_target_joint_pos = temp_joint_pos;
        }
        catch(const std::exception& e)
        {
            LOG_WARN("no inverse kinematics solution for left arm pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
                left_hand_target_pos(0), left_hand_target_pos(1), left_hand_target_pos(2),
                left_hand_target_orientation.w(), left_hand_target_orientation.x(), left_hand_target_orientation.y(), left_hand_target_orientation.z());
        }

        left_arm->getTransform(link_transform, link_com_transform, left_arm_actual_joint_pos);
        left_arm->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, left_arm_actual_joint_vel);
        left_arm->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        left_arm->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        centrifugal_coriolis_matrix = left_arm->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, left_arm_actual_joint_vel);
        gravity_compensate = left_arm->getJointSpaceGravityCompensate(link_com_jacobian);
        
        left_arm_target_joint_torque = centrifugal_coriolis_matrix * left_arm_actual_joint_vel + gravity_compensate;
        left_hand_actual_orientation = Eigen::Quaterniond(link_transform[5].block<3,3>(0,0));

        LOG_DEBUG("left arm target joint pos:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            left_arm_target_joint_pos(0), left_arm_target_joint_pos(1), left_arm_target_joint_pos(2),
            left_arm_target_joint_pos(3), left_arm_target_joint_pos(4), left_arm_target_joint_pos(5));
        LOG_DEBUG("left arm actual end effector pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
            link_transform[5](0,3), link_transform[5](1,3), link_transform[5](2,3),
            left_hand_actual_orientation.w(), left_hand_actual_orientation.x(), left_hand_actual_orientation.y(), left_hand_actual_orientation.z());

        try
        {
            Eigen::Vector<double,ArmModel::num_dof_> temp_joint_pos = Eigen::Vector<double,ArmModel::num_dof_>::Zero();
            // temp_joint_pos.block<3,1>(0,0) = right_arm->getShoulderJointPos(right_hand_target_pose, right_arm_actual_joint_pos.block<3,1>(0,0));
            // temp_joint_pos = right_arm->getInverseKinematics(right_hand_target_pose,right_arm_actual_joint_pos);
            temp_joint_pos = right_arm->getDampedLeastSquareInverseKinematics(
                0.1, Eigen::Vector<double,6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, right_hand_target_pose, right_arm_actual_joint_pos);
            if ( temp_joint_pos(0) < -M_PI/4 )
            {
                throw std::runtime_error("right joint 1 position unreasonable");
            }
            right_arm_target_joint_pos = temp_joint_pos;
        }
        catch(const std::exception& e)
        {
            LOG_WARN("no inverse kinematics solution for right arm pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
                right_hand_target_pos(0), right_hand_target_pos(1), right_hand_target_pos(2),
                right_hand_target_orientation.w(), right_hand_target_orientation.x(), right_hand_target_orientation.y(), right_hand_target_orientation.z());
        }

        right_arm->getTransform(link_transform, link_com_transform, right_arm_actual_joint_pos);
        right_arm->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, right_arm_actual_joint_vel);
        right_arm->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        right_arm->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        centrifugal_coriolis_matrix = right_arm->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, right_arm_actual_joint_vel);
        gravity_compensate = right_arm->getJointSpaceGravityCompensate(link_com_jacobian);

        right_arm_target_joint_torque = centrifugal_coriolis_matrix * right_arm_actual_joint_vel + gravity_compensate;
        right_hand_actual_orientation = Eigen::Quaterniond(link_transform[5].block<3,3>(0,0));
        LOG_DEBUG("right arm target joint pos:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            right_arm_target_joint_pos(0), right_arm_target_joint_pos(1), right_arm_target_joint_pos(2),
            right_arm_target_joint_pos(3), right_arm_target_joint_pos(4), right_arm_target_joint_pos(5));
        LOG_DEBUG("right arm actual end effector pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
            link_transform[5](0,3), link_transform[5](1,3), link_transform[5](2,3),
            right_hand_actual_orientation.w(), right_hand_actual_orientation.x(), right_hand_actual_orientation.y(), right_hand_actual_orientation.z());

        left_arm_interface->setGripperControl(0.07 - 0.07 * left_gripper_control, 0);
        right_arm_interface->setGripperControl(0.07 - 0.07 * right_gripper_control, 0);
        left_arm_interface->setActuatorControl(left_arm_target_joint_pos, left_arm_target_joint_vel, left_arm_target_joint_torque);
        right_arm_interface->setActuatorControl(right_arm_target_joint_pos, right_arm_target_joint_vel, right_arm_target_joint_torque);
    }
    whole_body_state_io_context.stop();
    dual_arm_state_io_context.stop();
    whole_body_state_asio_thread.join();
    dual_arm_state_asio_thread.join();

    // Destroy unique pointers explicitly
    left_arm_interface.reset();
    right_arm_interface.reset();

    LOG_INFO("Program terminated.");
    spdlog::drop_all();

    return 0;
}