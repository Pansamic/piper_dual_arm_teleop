
#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <glfw_adapter.h>
#include <simulate.h>
#include <array_safety.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <joint_state.h>

template<typename T, std::size_t NumDof>
class MujocoInterface
{
public:
    MujocoInterface() = default;
    ~MujocoInterface() = default;

    void start(const char* mujoco_file_path)
    {
        this->render_thread_ = std::thread([this]() {
            mjvCamera cam;
            mjv_defaultCamera(&cam);

            mjvOption opt;
            mjv_defaultOption(&opt);

            mjvPerturb pert;
            mjv_defaultPerturb(&pert);

            {
                std::lock_guard<std::mutex> lock(this->sim_mutex_);
                this->sim_ = std::make_unique<mujoco::Simulate>(
                    std::make_unique<mujoco::GlfwAdapter>(),
                    &cam, &opt, &pert, false
                );
                this->sim_ready_ = true;
            }

            this->sim_ready_cv_.notify_one();

            this->sim_->RenderLoop();
        });

        // Wait until sim_ is ready
        {
            std::unique_lock<std::mutex> lock(this->sim_mutex_);
            this->sim_ready_cv_.wait(lock, [this] { return this->sim_ready_; });
        }

        this->physics_thread_ = std::thread(&MujocoInterface::threadPhysics, this, mujoco_file_path);
        
        // Wait until mujoco model and mujoco data is ready
        {
            std::unique_lock<std::mutex> lock(this->model_mutex_);
            this->model_ready_cv_.wait(lock, [this] { return this->model_ready_; });
        }
    }

    void stop()
    {
        this->sim_->exitrequest.store(1);
        this->render_thread_.join();
        this->physics_thread_.join();
    }
    void setLeftArmJointControl(const Eigen::Vector<T, NumDof>& q)
    {
        std::lock_guard(this->left_arm_mutex_);
        this->left_arm_joint_ctrl = q;
    }
    void setLeftArmGripperControl(const T& position, const T& torque)
    {
        /* @todo Add position/torque control logic */
        this->d->ctrl[6] = position/2.0;
        this->d->ctrl[7] = -position/2.0;
    }
    void setRightArmJointControl(const Eigen::Vector<T, NumDof>& q)
    {
        std::lock_guard(this->right_arm_mutex_);
        this->right_arm_joint_ctrl = q;
    }
    void setRightArmGripperControl(const T& position, const T& torque)
    {
        /* @todo Add position/torque control logic */
        this->d->ctrl[14] = position/2.0;
        this->d->ctrl[15] = -position/2.0;
    }
    Eigen::Vector<T, NumDof> getLeftArmJointPosition()
    {
        std::lock_guard<std::mutex> lock(this->left_arm_mutex_);
        return this->left_arm_actual_state_.joint_pos;
    }
    Eigen::Vector<T, NumDof> getLeftArmJointVelocity()
    {
        std::lock_guard<std::mutex> lock(this->left_arm_mutex_);
        return this->left_arm_actual_state_.joint_vel;
    }

    Eigen::Vector<T, NumDof> getLeftArmJointAcceleration()
    {
        std::lock_guard<std::mutex> lock(this->left_arm_mutex_);
        return this->left_arm_actual_state_.joint_acc;
    }

    Eigen::Vector<T, NumDof> getLeftArmJointTorque()
    {
        std::lock_guard<std::mutex> lock(this->left_arm_mutex_);
        return this->left_arm_actual_state_.joint_torq;
    }

    Eigen::Vector<T, NumDof> getRightArmJointPosition()
    {
        std::lock_guard<std::mutex> lock(this->right_arm_mutex_);
        return this->right_arm_actual_state_.joint_pos;
    }

    Eigen::Vector<T, NumDof> getRightArmJointVelocity()
    {
        std::lock_guard<std::mutex> lock(this->right_arm_mutex_);
        return this->right_arm_actual_state_.joint_vel;
    }

    Eigen::Vector<T, NumDof> getRightArmJointAcceleration()
    {
        std::lock_guard<std::mutex> lock(this->right_arm_mutex_);
        return this->right_arm_actual_state_.joint_acc;
    }

    Eigen::Vector<T, NumDof> getRightArmJointTorque()
    {
        std::lock_guard<std::mutex> lock(this->right_arm_mutex_);
        return this->right_arm_actual_state_.joint_torq;
    }
    Eigen::Vector<T, 3> getLeftHandSitePosition() const
    {
        std::lock_guard<std::mutex>(this->left_arm_mutex_);
        Eigen::Vector<T, 3> pos;
        pos(0) = this->d->site_xpos[0];
        pos(1) = this->d->site_xpos[1];
        pos(2) = this->d->site_xpos[2];
        return pos;
    }
    Eigen::Vector<T, 3> getRightHandSitePosition() const
    {
        std::lock_guard<std::mutex>(this->right_arm_mutex_);
        Eigen::Vector<T, 3> pos;
        pos(0) = this->d->site_xpos[3];
        pos(1) = this->d->site_xpos[4];
        pos(2) = this->d->site_xpos[5];
        return pos;
    }
    Eigen::Matrix<T, 3, 3> getLeftHandSiteOrientation() const
    {
        std::lock_guard<std::mutex>(this->left_arm_mutex_);
        Eigen::Matrix<T, 3, 3> R;
        R(0, 0) = this->d->site_xmat[0];
        R(0, 1) = this->d->site_xmat[1];
        R(0, 2) = this->d->site_xmat[2];
        R(1, 0) = this->d->site_xmat[3];
        R(1, 1) = this->d->site_xmat[4];
        R(1, 2) = this->d->site_xmat[5];
        R(2, 0) = this->d->site_xmat[6];
        R(2, 1) = this->d->site_xmat[7];
        R(2, 2) = this->d->site_xmat[8];
        return R;
    }
    Eigen::Matrix<T, 3, 3> getRightHandSiteOrientation() const
    {
        std::lock_guard<std::mutex>(this->right_arm_mutex_);
        Eigen::Matrix<T, 3, 3> R;
        R(0, 0) = this->d->site_xmat[9];
        R(0, 1) = this->d->site_xmat[10];
        R(0, 2) = this->d->site_xmat[11];
        R(1, 0) = this->d->site_xmat[12];
        R(1, 1) = this->d->site_xmat[13];
        R(1, 2) = this->d->site_xmat[14];
        R(2, 0) = this->d->site_xmat[15];
        R(2, 1) = this->d->site_xmat[16];
        R(2, 2) = this->d->site_xmat[17];
        return R;
    }
private:
    static constexpr T syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
    static constexpr T simRefreshFraction = 0.7;  // fraction of refresh available for simulation
    static constexpr int kErrorLength = 1024;          // load error string length
    static constexpr T joint_kp_ = 3;
    static constexpr T joint_kd_ = 0.25;

    std::mutex sim_mutex_;
    std::condition_variable sim_ready_cv_;
    bool sim_ready_ = false;

    std::mutex model_mutex_;
    std::condition_variable model_ready_cv_;
    bool model_ready_ = false;

    mjModel* m; // MuJoCo model
    mjData* d; // MuJoCo data

    std::atomic<bool> running_;

    std::unique_ptr<mujoco::Simulate> sim_;
    std::thread physics_thread_;
    std::thread render_thread_;
    
    Eigen::Vector<T, NumDof> left_arm_joint_ctrl;
    Eigen::Vector<T, NumDof> right_arm_joint_ctrl;
    JointState<T, NumDof> left_arm_actual_state_;
    JointState<T, NumDof> right_arm_actual_state_;

    mutable std::mutex left_arm_mutex_;
    mutable std::mutex right_arm_mutex_;

    const char*  diverged(int disableflags, const mjData* d) {
        if (disableflags & mjDSBL_AUTORESET) {
            for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
                if (d->warning[w].number > 0) {
                    return mju_warningText(w, d->warning[w].lastinfo);
                }
            }
        }
        return nullptr;
    }

    mjModel*  loadModel(const char* file)
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
        T load_seconds = std::chrono::duration<T>(load_interval).count();

        if (!mnew) {
            LOG_ERROR("{}", loadError);
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
    void  physicsLoop()
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
                if (m)
                {
                    // running
                    if (this->sim_->run)
                    {
                        bool stepped = false;

                        // record cpu time at start of iteration
                        const auto startCPU = mujoco::Simulate::Clock::now();

                        // elapsed CPU and simulation time since last sync
                        const auto elapsedCPU = startCPU - syncCPU;
                        T elapsedSim = d->time - syncSim;

                        // requested slow-down factor
                        T slowdown = 100 / this->sim_->percentRealTime[this->sim_->real_time_index];

                        // misalignment condition: distance from target sim time is bigger than syncmisalign
                        bool misaligned =
                            std::abs(std::chrono::duration<T>(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

                        // out-of-sync (for any reason): reset sync times, step
                        if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned || this->sim_->speed_changed)
                        {
                            // re-sync
                            syncCPU = startCPU;
                            syncSim = d->time;
                            this->sim_->speed_changed = false;

                            // run single step, let next iteration deal with timing
                            for ( int i=0 ; i< NumDof ; i++ )
                            {
                                this->left_arm_actual_state_.joint_pos(i) = this->d->qpos[i];
                                this->right_arm_actual_state_.joint_pos(i) = this->d->qpos[i + NumDof + 2];  // +2 to skip left gripper joint
                                this->left_arm_actual_state_.joint_vel(i) = this->d->qvel[i];
                                this->right_arm_actual_state_.joint_vel(i) = this->d->qvel[i + NumDof + 2];  // +2 to skip left gripper joint
                                this->left_arm_actual_state_.joint_acc(i) = this->d->qacc[i];
                                this->right_arm_actual_state_.joint_acc(i) = this->d->qacc[i + NumDof + 2];  // +2 to skip left gripper joint
                                this->left_arm_actual_state_.joint_torq(i) = this->d->qfrc_actuator[i];
                                this->right_arm_actual_state_.joint_torq(i) = this->d->qfrc_actuator[i + NumDof + 2]; // +2 to skip left gripper joint
                            }
                            mj_step1(m, d);
                            for ( std::size_t i=0 ; i< NumDof ; i++ )
                            {
                                this->d->ctrl[i] = this->left_arm_joint_ctrl(i);
                                this->d->ctrl[i + 2 + NumDof] = this->right_arm_joint_ctrl(i);
                            }
                            mj_step2(m, d);
                            const char* message = diverged(m->opt.disableflags, d);
                            if (message)
                            {
                                this->sim_->run = 0;
                                mujoco::sample_util::strcpy_arr(this->sim_->load_error, message);
                            }
                            else
                            {
                                stepped = true;
                            }
                        }

                        // in-sync: step until ahead of cpu
                        else
                        {
                            bool measured = false;
                            mjtNum prevSim = d->time;

                            T refreshTime = simRefreshFraction/this->sim_->refresh_rate;

                            // step while sim lags behind cpu and within refreshTime
                            while (std::chrono::duration<T>((d->time - syncSim)*slowdown) < mujoco::Simulate::Clock::now() - syncCPU &&
                                mujoco::Simulate::Clock::now() - startCPU < std::chrono::duration<T>(refreshTime))
                            {
                                // measure slowdown before first step
                                if (!measured && elapsedSim)
                                {
                                    this->sim_->measured_slowdown =
                                        std::chrono::duration<T>(elapsedCPU).count() / elapsedSim;
                                    measured = true;
                                }

                                // inject noise
                                this->sim_->InjectNoise();

                                // call mj_step
                                for ( int i=0 ; i< NumDof ; i++ )
                                {
                                    this->left_arm_actual_state_.joint_pos(i) = this->d->qpos[i];
                                    this->right_arm_actual_state_.joint_pos(i) = this->d->qpos[i + NumDof + 2];  // +2 to skip left gripper joint
                                    this->left_arm_actual_state_.joint_vel(i) = this->d->qvel[i];
                                    this->right_arm_actual_state_.joint_vel(i) = this->d->qvel[i + NumDof + 2];  // +2 to skip left gripper joint
                                    this->left_arm_actual_state_.joint_acc(i) = this->d->qacc[i];
                                    this->right_arm_actual_state_.joint_acc(i) = this->d->qacc[i + NumDof + 2];  // +2 to skip left gripper joint
                                    this->left_arm_actual_state_.joint_torq(i) = this->d->qfrc_actuator[i];
                                    this->right_arm_actual_state_.joint_torq(i) = this->d->qfrc_actuator[i + NumDof + 2]; // +2 to skip left gripper joint
                                }
                                mj_step1(m, d);
                                for ( std::size_t i=0 ; i< NumDof ; i++ )
                                {
                                    this->d->ctrl[i] = this->left_arm_joint_ctrl(i);
                                    this->d->ctrl[i + 2 + NumDof] = this->right_arm_joint_ctrl(i);
                                }
                                mj_step2(m, d);
                                const char* message = diverged(m->opt.disableflags, d);
                                if (message)
                                {
                                    this->sim_->run = 0;
                                    mujoco::sample_util::strcpy_arr(this->sim_->load_error, message);
                                }
                                else
                                {
                                    stepped = true;
                                }

                                // break if reset
                                if (d->time < prevSim)
                                {
                                    break;
                                }
                            }
                        }

                        // save current state to history buffer
                        if (stepped)
                        {
                            this->sim_->AddToHistory();
                        }
                    }

                    // paused
                    else
                    {
                        // run mj_forward, to update rendering and joint sliders
                        mj_forward(m, d);
                        this->sim_->speed_changed = true;
                    }
                }
            }  // release std::lock_guard<std::mutex>
        }
    }

    void  threadPhysics(const char* mujoco_file_path)
    {
        this->sim_->LoadMessage(mujoco_file_path);

        std::unique_lock<std::mutex> lock(this->model_mutex_);

        m = this->loadModel(mujoco_file_path);
        if (m)
        {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

        d = mj_makeData(m);
        }
        if (d)
        {
        this->sim_->Load(m, d, mujoco_file_path);

        // lock the this->sim_ mutex
        const std::unique_lock<std::recursive_mutex> lock(this->sim_->mtx);

        mj_forward(m, d);

        }
        else
        {
        this->sim_->LoadMessageClear();
        }

        lock.unlock();

        this->model_ready_cv_.notify_one();
        this->model_ready_ = true;

        this->physicsLoop();

        // delete everything we allocated
        mj_deleteData(d);
        mj_deleteModel(m);
    }
};

