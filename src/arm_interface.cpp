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

ArmSimulationInterface::ArmSimulationInterface()
{

}

ArmSimulationInterface::~ArmSimulationInterface()
{
    this->render_thread_.join();
    this->physics_thread_.join();
}

void ArmSimulationInterface::initializeImpl()
{
    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    this->sim_ = std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false
    );

    // start physics thread
    std::thread physicsthreadhandle([this](){this->physicsLoop();});

    // start simulation UI loop (blocking call)
    std::thread([this](){this->sim_->RenderLoop();});
}

bool ArmSimulationInterface::isReadyImpl()
{
    /* todo: Add check ready logic */
    return true;
}

void ArmSimulationInterface::setLeftJointControl(
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
{
    this->left_target_joint_pos_ = joint_pos;
    this->left_target_joint_vel_ = joint_vel;
    this->left_target_joint_fd_torq_ = joint_feedforward_torque;
}

void ArmSimulationInterface::setRightJointControl(
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
{
    this->right_target_joint_pos_ = joint_pos;
    this->right_target_joint_vel_ = joint_vel;
    this->right_target_joint_fd_torq_ = joint_feedforward_torque;
}

void ArmSimulationInterface::setLeftGripperControlImpl(const double& position, const double& torque)
{
    /* @todo Add position/torque control logic */
    this->d->ctrl[6] = position;
}

void ArmSimulationInterface::setRightGripperControlImpl(const double& position, const double& torque)
{
    /* @todo Add position/torque control logic */
    this->d->ctrl[11] = position;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getLeftJointPositionImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_pos_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getLeftJointVelocityImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_vel_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getLeftJointTorqueImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_torq_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getRightJointPositionImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_pos_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getRightJointVelocityImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_vel_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmSimulationInterface::getRightJointTorqueImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_torq_;
}

void ArmSimulationInterface::setJointPDControl()
{
    Eigen::Vector<double, ArmModel::num_dof_> left_control =
        this->joint_kp_ * (this->left_target_joint_pos_ - this->left_actual_joint_pos_) + this->joint_kd_ * (left_target_joint_vel_ - left_actual_joint_vel_) + left_target_joint_fd_torq_;
    Eigen::Vector<double, ArmModel::num_dof_> right_control =
        this->joint_kp_ * (this->right_target_joint_pos_ - this->right_actual_joint_pos_) + this->joint_kd_ * (right_target_joint_vel_ - right_actual_joint_vel_) + right_target_joint_fd_torq_;

    for ( int i=0 ; i<ArmModel::num_dof_ ; i++ )
    {
        this->d->ctrl[i] = left_control[i];
        this->d->ctrl[i+ArmModel::num_dof_+1] = right_control[i]; // +1 for left gripper joint
    }
}

const char* ArmSimulationInterface::diverged(int disableflags, const mjData* d) {
    if (disableflags & mjDSBL_AUTORESET) {
        for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
            if (d->warning[w].number > 0) {
                return mju_warningText(w, d->warning[w].lastinfo);
            }
        }
    }
    return nullptr;
}

mjModel* ArmSimulationInterface::loadModel(const char* file)
{
    // this copy is needed so that the mujoco::sample_util::strlen call below compiles
    char filename[mujoco::Simulate::kMaxFilenameLength];
    mujoco::sample_util::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0]) {
        return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel* mnew = 0;
    auto load_start = mujoco::Simulate::Clock::now();
    if (mujoco::sample_util::strlen_arr(filename)>4 &&
        !std::strncmp(filename + mujoco::sample_util::strlen_arr(filename) - 4, ".mjb", mujoco::sample_util::sizeof_arr(filename) - mujoco::sample_util::strlen_arr(filename)+4)) {
        mnew = mj_loadModel(filename, nullptr);
        if (!mnew) {
            mujoco::sample_util::strcpy_arr(loadError, "could not load binary model");
        }
    } else {
        mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

        // remove trailing newline character from loadError
        if (loadError[0]) {
            int error_length = mujoco::sample_util::strlen_arr(loadError);
            if (loadError[error_length-1] == '\n') {
                loadError[error_length-1] = '\0';
            }
        }
    }
    auto load_interval = mujoco::Simulate::Clock::now() - load_start;
    double load_seconds = std::chrono::duration<double>(load_interval).count();

    if (!mnew) {
        LOG_ERROR(loadError);
        mujoco::sample_util::strcpy_arr(this->sim_->load_error, loadError);
        return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0]) {
        // mj_forward() below will print the warning message
        LOG_ERROR("Model compiled, but simulation warning (paused):{}", loadError);
        this->sim_->run = 0;
    }

    // if no error and load took more than 1/4 seconds, report load time
    else if (load_seconds > 0.25) {
        mujoco::sample_util::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
    }

    mujoco::sample_util::strcpy_arr(this->sim_->load_error, loadError);

    return mnew;
}

// simulate in background thread (while rendering in main thread)
void ArmSimulationInterface::physicsLoop()
{
    // cpu-sim syncronization point
    std::chrono::time_point<mujoco::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // run until asked to exit
    while (!this->sim_->exitrequest.load())
    {
        if (this->sim_->droploadrequest.load()) {
            this->sim_->LoadMessage(this->sim_->dropfilename);
            mjModel* mnew = loadModel(this->sim_->dropfilename);
            this->sim_->droploadrequest.store(false);

            mjData* dnew = nullptr;
            if (mnew) dnew = mj_makeData(mnew);
            if (dnew) {
                this->sim_->Load(mnew, dnew, this->sim_->dropfilename);

                // lock the sim mutex
                const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

                mj_deleteData(d);
                mj_deleteModel(m);

                m = mnew;
                d = dnew;
                mj_forward(m, d);

            } else {
                this->sim_->LoadMessageClear();
            }
        }

        if (this->sim_->uiloadrequest.load()) {
            this->sim_->uiloadrequest.fetch_sub(1);
            this->sim_->LoadMessage(this->sim_->filename);
            mjModel* mnew = loadModel(this->sim_->filename);
            mjData* dnew = nullptr;
            if (mnew) dnew = mj_makeData(mnew);
            if (dnew) {
                this->sim_->Load(mnew, dnew, this->sim_->filename);

                // lock the sim mutex
                const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

                mj_deleteData(d);
                mj_deleteModel(m);

                m = mnew;
                d = dnew;
                mj_forward(m, d);

            } else {
                this->sim_->LoadMessageClear();
            }
        }

        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if (this->sim_->run && this->sim_->busywait) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        {
            // lock the sim mutex
            const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

            // run only if model is present
            if (m) {
                // running
                if (this->sim_->run) {
                    bool stepped = false;

                    // record cpu time at start of iteration
                    const auto startCPU = mujoco::Simulate::Clock::now();

                    // elapsed CPU and simulation time since last sync
                    const auto elapsedCPU = startCPU - syncCPU;
                    double elapsedSim = d->time - syncSim;

                    // requested slow-down factor
                    double slowdown = 100 / this->sim_->percentRealTime[this->sim_->real_time_index];

                    // misalignment condition: distance from target sim time is bigger than syncmisalign
                    bool misaligned =
                        std::abs(std::chrono::duration<double>(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

                    // out-of-sync (for any reason): reset sync times, step
                    if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned || this->sim_->speed_changed) {
                        // re-sync
                        syncCPU = startCPU;
                        syncSim = d->time;
                        this->sim_->speed_changed = false;

                        // run single step, let next iteration deal with timing
                        for ( int i=0 ; i<ArmModel::num_dof_ ; i++ )
                        {
                            left_actual_joint_pos_(i) = this->d->qpos[i];
                            right_actual_joint_pos_(i) = this->d->qpos[i+ArmModel::num_dof_+1];  // +1 to skip left gripper joint
                            left_actual_joint_vel_(i) = this->d->qvel[i];
                            right_actual_joint_vel_(i) = this->d->qvel[i+ArmModel::num_dof_+1];  // +1 to skip left gripper joint
                            left_actual_joint_torq_(i) = this->d->qfrc_actuator[i];
                            right_actual_joint_torq_(i) = this->d->qfrc_actuator[i+ArmModel::num_dof_+1]; // +1 to skip left gripper joint
                        }
                        mj_step1(m, d);
                        this->setJointPDControl();
                        mj_step2(m, d);
                        const char* message = diverged(m->opt.disableflags, d);
                        if (message) {
                            this->sim_->run = 0;
                            mujoco::sample_util::strcpy_arr(this->sim_->load_error, message);
                        } else {
                            stepped = true;
                        }
                    }

                    // in-sync: step until ahead of cpu
                    else {
                        bool measured = false;
                        mjtNum prevSim = d->time;

                        double refreshTime = simRefreshFraction/this->sim_->refresh_rate;

                        // step while sim lags behind cpu and within refreshTime
                        while (std::chrono::duration<double>((d->time - syncSim)*slowdown) < mujoco::Simulate::Clock::now() - syncCPU &&
                            mujoco::Simulate::Clock::now() - startCPU < std::chrono::duration<double>(refreshTime)) {
                        // measure slowdown before first step
                        if (!measured && elapsedSim) {
                            this->sim_->measured_slowdown =
                                std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                            measured = true;
                        }

                        // inject noise
                        this->sim_->InjectNoise();

                        // call mj_step
                        for ( int i=0 ; i<ArmModel::num_dof_ ; i++ )
                        {
                            left_actual_joint_pos_(i) = this->d->qpos[i];
                            right_actual_joint_pos_(i) = this->d->qpos[i+ArmModel::num_dof_+1];  // +1 to skip left gripper joint
                            left_actual_joint_vel_(i) = this->d->qvel[i];
                            right_actual_joint_vel_(i) = this->d->qvel[i+ArmModel::num_dof_+1];  // +1 to skip left gripper joint
                            left_actual_joint_torq_(i) = this->d->qfrc_actuator[i];
                            right_actual_joint_torq_(i) = this->d->qfrc_actuator[i+ArmModel::num_dof_+1]; // +1 to skip left gripper joint
                        }
                        mj_step1(m, d);
                        this->setJointPDControl();
                        mj_step2(m, d);
                        const char* message = diverged(m->opt.disableflags, d);
                        if (message) {
                            this->sim_->run = 0;
                            mujoco::sample_util::strcpy_arr(this->sim_->load_error, message);
                        } else {
                            stepped = true;
                        }

                        // break if reset
                        if (d->time < prevSim) {
                            break;
                        }
                        }
                    }

                    // save current state to history buffer
                    if (stepped) {
                        this->sim_->AddToHistory();
                    }
                }

                // paused
                else {
                    // run mj_forward, to update rendering and joint sliders
                    mj_forward(m, d);
                    this->sim_->speed_changed = true;
                }
            }
        }  // release std::lock_guard<std::mutex>
    }
}

void ArmSimulationInterface::threadPhysics() {
    this->sim_->LoadMessage(this->mujoco_file_);
    m = this->loadModel(this->mujoco_file_);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      this->sim_->Load(m, d, this->mujoco_file_);

      // lock the this->sim_ mutex
      const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

      mj_forward(m, d);

    } else {
      this->sim_->LoadMessageClear();
    }

    this->physicsLoop();

    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
}

/**
 * @brief Initialize SocketCAN
 * 
 */
ArmHardwareInterface::ArmHardwareInterface(const char* left_arm_can_name, const char* right_arm_can_name)
{
    this->left_can_socket_ = -1;
    this->right_can_socket_ = -1;

    this->left_actual_joint_pos_.setZero();
    this->left_actual_joint_vel_.setZero();
    this->left_actual_joint_torq_.setZero();

    this->right_actual_joint_pos_.setZero();
    this->right_actual_joint_vel_.setZero();
    this->right_actual_joint_torq_.setZero();

    struct sockaddr_can can_addr_ = {0};
    struct ifreq can_ifr_ = {0};

    this->left_can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_ifr_.ifr_name, left_arm_can_name);
    ioctl(this->left_can_socket_, SIOCGIFINDEX, &can_ifr_);
    can_addr_.can_family = AF_CAN;
    can_addr_.can_ifindex = can_ifr_.ifr_ifindex;
    bind(this->left_can_socket_, (struct sockaddr *)&can_addr_, sizeof(can_addr_));

    this->right_can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(can_ifr_.ifr_name, right_arm_can_name);
    ioctl(this->right_can_socket_, SIOCGIFINDEX, &can_ifr_);
    can_addr_.can_family = AF_CAN;
    can_addr_.can_ifindex = can_ifr_.ifr_ifindex;
    bind(this->right_can_socket_, (struct sockaddr *)&can_addr_, sizeof(can_addr_));

    /* Set interface running flag. */
    running_ = true;

    /* Start listening on actuator feedback information */
    read_thread_ = std::thread(&ArmHardwareInterface::threadCanReceive, this);

    /* Enable all actuators */
    bool actuator_enabled = false;
    for(int i=0 ; i<20 ; i++)
    {
        enableActuators(this->left_can_socket_);
        enableActuators(this->right_can_socket_);
        sleep(0.1);
        if (!(left_arm_actuator_status_[0] & ACTUATOR_STATUS_DISABLED) &&
            !(left_arm_actuator_status_[1] & ACTUATOR_STATUS_DISABLED) &&
            !(left_arm_actuator_status_[2] & ACTUATOR_STATUS_DISABLED) &&
            !(left_arm_actuator_status_[3] & ACTUATOR_STATUS_DISABLED) &&
            !(left_arm_actuator_status_[4] & ACTUATOR_STATUS_DISABLED) &&
            !(left_arm_actuator_status_[5] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[0] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[1] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[2] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[3] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[4] & ACTUATOR_STATUS_DISABLED) &&
            !(right_arm_actuator_status_[5] & ACTUATOR_STATUS_DISABLED) )
        {
            actuator_enabled = true;
            LOG_INFO("Piper actuators enabled.");
            break;
        }
    }
    if ( !actuator_enabled )
    {
        LOG_ERROR("Failed to enable actuators. Exiting...");
        exit(EXIT_FAILURE);
    }
}

ArmHardwareInterface::~ArmHardwareInterface()
{
    setJointPositionMode(this->left_can_socket_);
    setJointPositionMode(this->right_can_socket_);
    /* Comment the following lines to avoid sudden fall */
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
    if ( this->left_can_socket_ != -1 )
    {
        close(this->left_can_socket_);
    }
    if ( this->right_can_socket_ != -1 )
    {
        close(this->right_can_socket_);
    }
    running_ = false;
    read_thread_.join();
    LOG_INFO("Arm Interface is terminated.");
}

void ArmHardwareInterface::setLeftJointControl(
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
{
    std::lock_guard(this->left_joints_mutex_);
    setActuatorControl(this->left_can_socket_, joint_pos, joint_vel, joint_feedforward_torque);
}

void ArmHardwareInterface::setRightJointControl(
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
{
    std::lock_guard(this->right_joints_mutex_);
    setActuatorControl(this->right_can_socket_, joint_pos, joint_vel, joint_feedforward_torque);
}

void ArmHardwareInterface::setLeftGripperControlImpl(const double& position, const double& torque)
{
    setGripperControl(this->left_can_socket_, position, torque);
}

void ArmHardwareInterface::setRightGripperControlImpl(const double& position, const double& torque)
{
    setGripperControl(this->right_can_socket_, position, torque);
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getLeftJointPositionImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_pos_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getLeftJointVelocityImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_vel_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getLeftJointTorqueImpl() const
{
    std::lock_guard(this->left_joints_mutex_);
    return this->left_actual_joint_torq_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getRightJointPositionImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_pos_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getRightJointVelocityImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_vel_;
}

const Eigen::Vector<double,ArmModel::num_dof_>& ArmHardwareInterface::getRightJointTorqueImpl() const
{
    std::lock_guard(this->right_joints_mutex_);
    return this->right_actual_joint_torq_;
}

void ArmHardwareInterface::threadCanReceive(void)
{
    struct can_frame frame = {0};
    size_t recvbytes = 0;

    while (this->running_)
    {
        recvbytes = read(this->left_can_socket_, &frame, sizeof(struct can_frame));
        if (recvbytes > 0)
        {
            std::lock_guard<std::mutex> lock(this->left_arm_can_mtx_);
            /* parse feedback frame data */
            parseCanFrame(this->left_can_socket_, &frame);
        }
        recvbytes = read(this->right_can_socket_, &frame, sizeof(struct can_frame));
        if (recvbytes > 0)
        {
            std::lock_guard<std::mutex> lock(this->right_arm_can_mtx_);
            /* parse feedback frame data */
            parseCanFrame(this->right_can_socket_, &frame);
        }
    }
}

// void ArmHardwareInterface::threadCanTransmit(void)
// {
//     auto increase_time_spec = [](struct timespec* time, const struct timespec* increasement)
//     {
//         time->tv_sec += increasement->tv_sec;
//         time->tv_nsec += increasement->tv_nsec;

//         if (time->tv_nsec >= 1000000000)
//         {
//             time->tv_sec++;
//             time->tv_nsec -= 1000000000;
//         }
//     };

//     struct timespec wakeup_time = {0,0};
//     static const struct timespec cycletime = {0, 5000000};
//     clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

//     while(this->running_)
//     {
//         increase_time_spec(&wakeup_time, &cycletime);
//         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

//         std::lock_guard(this->left_joints_mutex_);
//         setActuatorControl(this->left_can_socket_, this->left_target_joint_pos_, this->left_target_joint_vel_, this->left_target_joint_fd_torq_);
//         std::lock_guard(this->right_joints_mutex_);
//         setActuatorControl(this->right_can_socket_, this->right_target_joint_pos_, this->right_target_joint_vel_, this->right_target_joint_fd_torq_);
//     }
// }

void ArmHardwareInterface::setMITMode(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_MIT;   // MIT move mode.
    frame.data[2] = 50;     // Move rate.
    frame.data[3] = 0xAD;   // MIT mode.
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setJointPositionMode(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_JOINT; // Joint move mode.
    frame.data[2] = 100;    // Move rate.
    frame.data[3] = 0x00;   // Joint position control.
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setEndEffectorPoseMode(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x151;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;   // CAN command mode.
    frame.data[1] = MOVE_MODE_POSITION; // End effector move mode.
    frame.data[2] = 100;    // Move rate.
    frame.data[3] = 0x00;   // Joint position control.
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::parseCanFrame(const int can_socket, struct can_frame* msg)
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
        case STATUS_SINGULAR:
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
        if ( can_socket == this->left_can_socket_ )
        {
            this->left_actual_joint_pos_(actuator_id) = static_cast<double>(position) / 1000.0;
            this->left_actual_joint_vel_(actuator_id) = static_cast<double>(velocity) / 1000.0; // Warn: haven't checked this scaler.
            if ( actuator_id == 0 || actuator_id == 1 || actuator_id == 2 )
            {
                this->left_actual_joint_torq_(actuator_id) = static_cast<double>(current) / 1000.0 * 1.18125;
            }
            else if ( actuator_id == 3 || actuator_id == 4 || actuator_id == 5 )
            {
                this->left_actual_joint_torq_(actuator_id) = static_cast<double>(current) / 1000.0 * 0.95844;
            }
        }
        else if ( can_socket == this->right_can_socket_ )
        {
            this->right_actual_joint_pos_(actuator_id) = static_cast<double>(position) / 1000.0;
            this->right_actual_joint_vel_(actuator_id) = static_cast<double>(velocity) / 1000.0; // Warn: haven't checked this scaler.
            if ( actuator_id == 0 || actuator_id == 1 || actuator_id == 2 )
            {
                this->right_actual_joint_torq_(actuator_id) = static_cast<double>(current) / 1000.0 * 1.18125;
            }
            else if ( actuator_id == 3 || actuator_id == 4 || actuator_id == 5 )
            {
                this->right_actual_joint_torq_(actuator_id) = static_cast<double>(current) / 1000.0 * 0.95844;
            }
        }

        return ;
    }

    /* actuator low speed feedback */
    if ( msg->can_id >= 0x261 && msg->can_id <= 0x266 )
    {
        uint16_t actuator_id = msg->can_id - 0x261;
        uint16_t voltage = (msg->data[0] << 8) | msg->data[1];
        if ( can_socket == this->left_can_socket_ )
        {
            left_arm_actuator_voltage_[actuator_id] = static_cast<float>(voltage);
            left_arm_actuator_temperature_[actuator_id] = (msg->data[2] << 8) | msg->data[3];
            left_arm_motor_temperature_[actuator_id] = msg->data[4];
            left_arm_actuator_status_[actuator_id] = msg->data[5];
        }
        else if ( can_socket == this->right_can_socket_ )
        {
            right_arm_actuator_voltage_[actuator_id] = static_cast<float>(voltage);
            right_arm_actuator_temperature_[actuator_id] = (msg->data[2] << 8) | msg->data[3];
            right_arm_motor_temperature_[actuator_id] = msg->data[4];
            right_arm_actuator_status_[actuator_id] = msg->data[5];
        }
        return ;
    }
}

void ArmHardwareInterface::sendControlMessage(
    const int can_socket,
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
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::enableActuators(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x471;
    frame.can_dlc = 8;
    frame.data[0] = 7; // Select all the actuators.
    frame.data[1] = 0x02; // Enable
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::disableActuators(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x471;
    frame.can_dlc = 8;
    frame.data[0] = 7; // Select all the actuators.
    frame.data[1] = 0x01; // Disable
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setActuatorControl(
    const int can_socket,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
    const Eigen::Vector<double,ArmModel::num_dof_>& joint_torque) const
{
    auto double2int16 = [](double x, double min, double max, uint8_t bits) -> int16_t
    {
        return static_cast<int16_t>((x-min)*(static_cast<double>((1<<bits)-1)/(max-min))); 
    };
    auto double2int8 = [](double x, double min, double max, uint8_t bits) -> int8_t
    {
        return static_cast<int8_t>((x-min)*(static_cast<double>((1<<bits)-1)/(max-min))); 
    };
    setMITMode(can_socket);
    for ( int i=0 ; i<6 ; i++ )
    {
        sendControlMessage(can_socket, i,
            double2int16(joint_pos(i),-12.5,12.5,16),
            double2int16(joint_vel(i),-45.0,45.0,12),
            double2int16(2.0,0.0,500.0,12),
            double2int16(0.15,-5,5.0,12),
            double2int8(joint_torque(i),-18.0,18.0,8)
       );
    }
}

void ArmHardwareInterface::setActuatorPosition(const int can_socket, const Eigen::Vector<double,6>& joint_pos) const
{
    struct can_frame frame;
    int32_t joint_pos_scaled;
    
    /* Set default joint position control mode. */
    setJointPositionMode(can_socket);

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
    write(can_socket, &frame, sizeof(struct can_frame));

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
    write(can_socket, &frame, sizeof(struct can_frame));

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
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setEndEffectorPose(const int can_socket, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation) const
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
    
    setEndEffectorPoseMode(can_socket);

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
    write(can_socket, &frame, sizeof(struct can_frame));

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
    write(can_socket, &frame, sizeof(struct can_frame));

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
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setGripperCurrentAsZero(const int can_socket) const
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x159;
    frame.can_dlc = 8;
    frame.data[7] = 0xAE;
    write(can_socket, &frame, sizeof(struct can_frame));
}

void ArmHardwareInterface::setGripperControl(const int can_socket, const double position, const double torque) const
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
    write(can_socket, &frame, sizeof(struct can_frame));
}
