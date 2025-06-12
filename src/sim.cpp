#include <iostream>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <random>
#include <time.h>
#include <pthread.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <asio.hpp>
#include <comm_channel.hpp>
#include <whole_body_msg/whole_body_msg.h>
#include <whole_body_msg/whole_body_receiver.hpp>
#include <whole_body_msg/whole_body_sender.hpp>
#include <log.h>
#include <arm_control.h>


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

volatile sig_atomic_t terminate = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
    {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else
    {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
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

void render_loop()
{
    // init GLFW
    if (!glfwInit())
    {
        LOG_ERROR("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "pathfinder", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    cam.type = mjCAMERA_FREE;  // typically use FREE for manual setup

    // Position the camera:
    cam.lookat[0] = -0.3;   // x-coordinate of lookat point
    cam.lookat[1] = 0.0;   // y-coordinate
    cam.lookat[2] = 1.2;   // z-coordinate

    // Distance from the lookat point:
    cam.distance = 0.0;  // meters

    // Orientation:
    cam.azimuth = 0.0;   // horizontal angle in degrees
    cam.elevation = -38.0; // vertical angle in degrees

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    struct timespec time = {0};
    /* time interval of 1/60 second. */
    static const struct timespec cycletime = {0,16666666};
    clock_gettime(CLOCK_MONOTONIC, &time);

    while (!glfwWindowShouldClose(window) && !terminate)
    {
        increase_time_spec(&time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, NULL);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }
    terminate = 1;
}

int main()
{
    /* Create logger file path */
    std::filesystem::path logpath(PROJECT_PATH"/logs");
    /* Initialize logger */
    initLogger(logpath, "sim");

    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kp(100,100,100,100,100,100);
    static const Eigen::Vector<double,ArmModel::num_dof_> joint_kd(20,20,20,20,20,20);
    static const Eigen::Vector3d head_position(0.325024,0,0.80246);
    static const Eigen::Quaterniond left_hand_orientation_offset = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    static const Eigen::Quaterniond right_hand_orientation_offset(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));

    Eigen::Vector3d base_position;
    Eigen::Quaterniond base_orientation;

    static const Eigen::Matrix4d left_arm_base_transform = (Eigen::Matrix4d() << 
        0, -0.707106781 ,  0.707106781 , 0.23805,
        0,  0.707106781 ,  0.707106781 , 0.19675,
        -1, 0           ,  0           , 0.74065,
        0,  0           ,  0           , 1).finished();
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

    std::unique_ptr<ArmModel> left_arm = std::make_unique<ArmModel>(left_arm_base_transform);
    std::unique_ptr<ArmModel> right_arm = std::make_unique<ArmModel>(right_arm_base_transform);

    m = mj_loadXML(PROJECT_PATH"/assets/mujoco_model/pathfinder.xml", 0, NULL, 0);
    if ( m == NULL )
    {
        LOG_ERROR("Load Mujoco XML error.");
        return -1;
    }

    d = mj_makeData(m);

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

    std::thread render_thread(render_loop);

    struct timespec wakeup_time = {0,0};
    static const struct timespec cycletime = {0, 5000000};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    
    while (!terminate)
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        while ( whole_body_state_receive_buffer->size() > 0 )
        {
            std::shared_ptr<WholeBodyStateMsg::Definition> cmd = whole_body_state_receive_buffer->pop();

            base_orientation = cmd->base_quat.cast<double>();
            left_hand_target_pos = cmd->left_hand_pos;
            right_hand_target_pos = cmd->right_hand_pos;
            left_hand_target_orientation = cmd->left_hand_quat.cast<double>() * left_hand_orientation_offset;
            right_hand_target_orientation = cmd->right_hand_quat.cast<double>() * right_hand_orientation_offset;

            left_hand_target_pos = left_hand_target_pos + head_position;
            right_hand_target_pos = right_hand_target_pos + head_position;

            left_gripper_control = cmd->left_gripper_ctrl;
            right_gripper_control = cmd->right_gripper_ctrl;

        }
        /* left hand position limit */
        if ( left_hand_target_pos(0) < 0.3 )
        {
            LOG_ERROR("left hand position hit body.x={:.4f},y={:.4f},z={:.4f}", left_hand_target_pos(0), left_hand_target_pos(1), left_hand_target_pos(2));
            left_hand_target_pos(0) = 0.3;
        }
        /* right hand position limit */
        if ( right_hand_target_pos(0) < 0.3 )
        {
            LOG_ERROR("right hand position hit body.x={:.4f},y={:.4f},z={:.4f}", right_hand_target_pos(0), right_hand_target_pos(1), right_hand_target_pos(2));
            right_hand_target_pos(0) = 0.3;
        }

        d->mocap_pos[0] = left_hand_target_pos(0);
        d->mocap_pos[1] = left_hand_target_pos(1);
        d->mocap_pos[2] = left_hand_target_pos(2);

        d->mocap_pos[3] = right_hand_target_pos(0);
        d->mocap_pos[4] = right_hand_target_pos(1);
        d->mocap_pos[5] = right_hand_target_pos(2);

        d->mocap_quat[0] = left_hand_target_orientation.w();
        d->mocap_quat[1] = left_hand_target_orientation.x();
        d->mocap_quat[2] = left_hand_target_orientation.y();
        d->mocap_quat[3] = left_hand_target_orientation.z();

        d->mocap_quat[4] = right_hand_target_orientation.w();
        d->mocap_quat[5] = right_hand_target_orientation.x();
        d->mocap_quat[6] = right_hand_target_orientation.y();
        d->mocap_quat[7] = right_hand_target_orientation.z();

        mj_step1(m, d);

        left_arm_actual_joint_pos << d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5];

        LOG_DEBUG("left arm actual joint pos:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5]);

        right_arm_actual_joint_pos << d->qpos[8], d->qpos[9], d->qpos[10], d->qpos[11], d->qpos[12], d->qpos[13];

        LOG_DEBUG("right arm actual joint pos:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            d->qpos[8], d->qpos[9], d->qpos[10], d->qpos[11], d->qpos[12], d->qpos[13]);

        left_arm_actual_joint_vel << d->qvel[0], d->qvel[1], d->qvel[2], d->qvel[3], d->qvel[4], d->qvel[5];

        // LOG_DEBUG("left arm actual joint vel:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
        //     d->qvel[0], d->qvel[1], d->qvel[2], d->qvel[3], d->qvel[4], d->qvel[5]);

        right_arm_actual_joint_vel << d->qvel[8], d->qvel[9], d->qvel[10], d->qvel[11], d->qvel[12], d->qvel[13];
        
        // LOG_DEBUG("right arm actual joint vel:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
        //     d->qvel[8], d->qvel[9], d->qvel[10], d->qvel[11], d->qvel[12], d->qvel[13]);

        left_arm_actual_joint_torque << d->qfrc_actuator[0], d->qfrc_actuator[1], d->qfrc_actuator[2], d->qfrc_actuator[3], d->qfrc_actuator[4], d->qfrc_actuator[5];
        
        // LOG_DEBUG("left arm actual joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
        //     d->qfrc_actuator[0], d->qfrc_actuator[1], d->qfrc_actuator[2], d->qfrc_actuator[3], d->qfrc_actuator[4], d->qfrc_actuator[5]);

        right_arm_actual_joint_torque << d->qfrc_actuator[8], d->qfrc_actuator[9], d->qfrc_actuator[10], d->qfrc_actuator[11], d->qfrc_actuator[12], d->qfrc_actuator[13];
        
        // LOG_DEBUG("right arm actual joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
        //     d->qfrc_actuator[8], d->qfrc_actuator[9], d->qfrc_actuator[10], d->qfrc_actuator[11], d->qfrc_actuator[12], d->qfrc_actuator[13]);

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
            LOG_WARN("left arm IK error:{}.x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
                e.what(),
                left_hand_target_pos(0), left_hand_target_pos(1), left_hand_target_pos(2),
                left_hand_target_orientation.w(), left_hand_target_orientation.x(), left_hand_target_orientation.y(), left_hand_target_orientation.z());
        }

        left_arm->getTransform(link_transform, link_com_transform, left_arm_actual_joint_pos);
        left_arm->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, left_arm_actual_joint_vel);
        left_arm->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        left_arm->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = left_arm->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = left_arm->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, left_arm_actual_joint_vel);
        gravity_compensate = left_arm->getJointSpaceGravityCompensate(link_com_jacobian);

        left_arm_target_joint_torque = generalized_mass_matrix * (joint_kp.cwiseProduct( left_arm_target_joint_pos - left_arm_actual_joint_pos ) + joint_kd.cwiseProduct( left_arm_target_joint_vel - left_arm_actual_joint_vel )) + centrifugal_coriolis_matrix * left_arm_actual_joint_vel + gravity_compensate;
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
            LOG_WARN("right arm IK error:{}.x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
                e.what(),
                right_hand_target_pos(0), right_hand_target_pos(1), right_hand_target_pos(2),
                right_hand_target_orientation.w(), right_hand_target_orientation.x(), right_hand_target_orientation.y(), right_hand_target_orientation.z());
        }
        right_arm->getTransform(link_transform, link_com_transform, right_arm_actual_joint_pos);
        right_arm->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, right_arm_actual_joint_vel);
        right_arm->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        right_arm->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = right_arm->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = right_arm->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, right_arm_actual_joint_vel);
        gravity_compensate = right_arm->getJointSpaceGravityCompensate(link_com_jacobian);
        
        right_arm_target_joint_torque = generalized_mass_matrix * (joint_kp.cwiseProduct( right_arm_target_joint_pos - right_arm_actual_joint_pos ) + joint_kd.cwiseProduct( right_arm_target_joint_vel - right_arm_actual_joint_vel )) + centrifugal_coriolis_matrix * right_arm_actual_joint_vel + gravity_compensate;
        right_hand_actual_orientation = Eigen::Quaterniond(link_transform[5].block<3,3>(0,0));
        LOG_DEBUG("right arm target joint pos:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
            right_arm_target_joint_pos(0), right_arm_target_joint_pos(1), right_arm_target_joint_pos(2),
            right_arm_target_joint_pos(3), right_arm_target_joint_pos(4), right_arm_target_joint_pos(5));
        LOG_DEBUG("right arm actual end effector pose:x={:.4f},y={:.4f},z={:.4f},qw={:.4f},qx={:.4f},qy={:.4f},qz={:.4f}",
            link_transform[5](0,3), link_transform[5](1,3), link_transform[5](2,3),
            right_hand_actual_orientation.w(), right_hand_actual_orientation.x(), right_hand_actual_orientation.y(), right_hand_actual_orientation.z());

        d->ctrl[6] = 0.035 - 0.035 * left_gripper_control;
        d->ctrl[7] = -0.035 + 0.035 * left_gripper_control;
        d->ctrl[14] = 0.035 - 0.035 * right_gripper_control;
        d->ctrl[15] = -0.035 + 0.035 * right_gripper_control;

        for ( int i = 0; i < 6; i++ )
        {
            d->ctrl[i] = left_arm_target_joint_torque[i];
        }
        for ( int i = 8 ; i < 14 ; i++ )
        {
            d->ctrl[i] = right_arm_target_joint_torque[i-8];
        }
        mj_step2(m, d);
    }
    whole_body_state_io_context.stop();
    dual_arm_state_io_context.stop();
    whole_body_state_asio_thread.join();
    dual_arm_state_asio_thread.join();
    render_thread.join();
    spdlog::drop_all();
    
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
