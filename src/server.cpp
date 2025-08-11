/**
 * @file server.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief MuJoCo GUI server to simulate robotics manipulator and send control messages to client.
 * @version 0.1
 * @date 2025-08-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <time.h>
#include <iostream>
#include <log.hpp>
#include <config.h>
#include <termination.h>
#include <utils.hpp>
#include <mujoco_frontend.hpp>
#include <mujoco_backend.hpp>
#include <messenger.hpp>
#include <piper_model.hpp>
#include <trajectory_buffer.hpp>
#include <arm_planner.hpp>
#include <arm_controller.hpp>
#include <mujoco_interface.hpp>

// #define MUJOCO_PLUGIN_DIR "mujoco_plugin"

// extern "C" {
// #if defined(_WIN32) || defined(__CYGWIN__)
//   #include <windows.h>
// #else
//   #if defined(__APPLE__)
//     #include <mach-o/dyld.h>
//   #endif
//   #include <sys/errno.h>
//   #include <unistd.h>
// #endif
// }

// namespace {
// namespace mj = ::mujoco;
// namespace mju = ::mujoco::sample_util;

// // constants
// const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
// const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
// const int kErrorLength = 1024;          // load error string length

// // model and data
// mjModel* m = nullptr;
// mjData* d = nullptr;

// using Seconds = std::chrono::duration<double>;


// //---------------------------------------- plugin handling -----------------------------------------

// // return the path to the directory containing the current executable
// // used to determine the location of auto-loaded plugin libraries
// std::string getExecutableDir() {
// #if defined(_WIN32) || defined(__CYGWIN__)
//   constexpr char kPathSep = '\\';
//   std::string realpath = [&]() -> std::string {
//     std::unique_ptr<char[]> realpath(nullptr);
//     DWORD buf_size = 128;
//     bool success = false;
//     while (!success) {
//       realpath.reset(new(std::nothrow) char[buf_size]);
//       if (!realpath) {
//         std::cerr << "cannot allocate memory to store executable path\n";
//         return "";
//       }

//       DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
//       if (written < buf_size) {
//         success = true;
//       } else if (written == buf_size) {
//         // realpath is too small, grow and retry
//         buf_size *=2;
//       } else {
//         std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
//         return "";
//       }
//     }
//     return realpath.get();
//   }();
// #else
//   constexpr char kPathSep = '/';
// #if defined(__APPLE__)
//   std::unique_ptr<char[]> buf(nullptr);
//   {
//     std::uint32_t buf_size = 0;
//     _NSGetExecutablePath(nullptr, &buf_size);
//     buf.reset(new char[buf_size]);
//     if (!buf) {
//       std::cerr << "cannot allocate memory to store executable path\n";
//       return "";
//     }
//     if (_NSGetExecutablePath(buf.get(), &buf_size)) {
//       std::cerr << "unexpected error from _NSGetExecutablePath\n";
//     }
//   }
//   const char* path = buf.get();
// #else
//   const char* path = "/proc/self/exe";
// #endif
//   std::string realpath = [&]() -> std::string {
//     std::unique_ptr<char[]> realpath(nullptr);
//     std::uint32_t buf_size = 128;
//     bool success = false;
//     while (!success) {
//       realpath.reset(new(std::nothrow) char[buf_size]);
//       if (!realpath) {
//         std::cerr << "cannot allocate memory to store executable path\n";
//         return "";
//       }

//       std::size_t written = readlink(path, realpath.get(), buf_size);
//       if (written < buf_size) {
//         realpath.get()[written] = '\0';
//         success = true;
//       } else if (written == -1) {
//         if (errno == EINVAL) {
//           // path is already not a symlink, just use it
//           return path;
//         }

//         std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
//         return "";
//       } else {
//         // realpath is too small, grow and retry
//         buf_size *= 2;
//       }
//     }
//     return realpath.get();
//   }();
// #endif

//   if (realpath.empty()) {
//     return "";
//   }

//   for (std::size_t i = realpath.size() - 1; i > 0; --i) {
//     if (realpath.c_str()[i] == kPathSep) {
//       return realpath.substr(0, i);
//     }
//   }

//   // don't scan through the entire file system's root
//   return "";
// }



// // scan for libraries in the plugin directory to load additional plugins
// void scanPluginLibraries() {
//   // check and print plugins that are linked directly into the executable
//   int nplugin = mjp_pluginCount();
//   if (nplugin) {
//     std::printf("Built-in plugins:\n");
//     for (int i = 0; i < nplugin; ++i) {
//       std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
//     }
//   }

//   // define platform-specific strings
// #if defined(_WIN32) || defined(__CYGWIN__)
//   const std::string sep = "\\";
// #else
//   const std::string sep = "/";
// #endif


//   // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
//   // ${EXECDIR} is the directory containing the simulate binary itself
//   // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
//   const std::string executable_dir = getExecutableDir();
//   if (executable_dir.empty()) {
//     return;
//   }

//   const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
//   mj_loadAllPluginLibraries(
//       plugin_dir.c_str(), +[](const char* filename, int first, int count) {
//         std::printf("Plugins registered by library '%s':\n", filename);
//         for (int i = first; i < first + count; ++i) {
//           std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
//         }
//       });
// }


// //------------------------------------------- simulation -------------------------------------------

// const char* Diverged(int disableflags, const mjData* d) {
//   if (disableflags & mjDSBL_AUTORESET) {
//     for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
//       if (d->warning[w].number > 0) {
//         return mju_warningText(w, d->warning[w].lastinfo);
//       }
//     }
//   }
//   return nullptr;
// }

// mjModel* LoadModel(const char* file, mj::Simulate& sim) {
//   // this copy is needed so that the mju::strlen call below compiles
//   char filename[mj::Simulate::kMaxFilenameLength];
//   mju::strcpy_arr(filename, file);

//   // make sure filename is not empty
//   if (!filename[0]) {
//     return nullptr;
//   }

//   // load and compile
//   char loadError[kErrorLength] = "";
//   mjModel* mnew = 0;
//   auto load_start = mj::Simulate::Clock::now();
//   if (mju::strlen_arr(filename)>4 &&
//       !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
//                     mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
//     mnew = mj_loadModel(filename, nullptr);
//     if (!mnew) {
//       mju::strcpy_arr(loadError, "could not load binary model");
//     }
//   } else {
//     mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

//     // remove trailing newline character from loadError
//     if (loadError[0]) {
//       int error_length = mju::strlen_arr(loadError);
//       if (loadError[error_length-1] == '\n') {
//         loadError[error_length-1] = '\0';
//       }
//     }
//   }
//   auto load_interval = mj::Simulate::Clock::now() - load_start;
//   double load_seconds = Seconds(load_interval).count();

//   if (!mnew) {
//     std::printf("%s\n", loadError);
//     mju::strcpy_arr(sim.load_error, loadError);
//     return nullptr;
//   }

//   // compiler warning: print and pause
//   if (loadError[0]) {
//     // mj_forward() below will print the warning message
//     std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
//     sim.run = 0;
//   }

//   // if no error and load took more than 1/4 seconds, report load time
//   else if (load_seconds > 0.25) {
//     mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
//   }

//   mju::strcpy_arr(sim.load_error, loadError);

//   return mnew;
// }

// // simulate in background thread (while rendering in main thread)
// void PhysicsLoop(mj::Simulate& sim) {
//   // cpu-sim syncronization point
//   std::chrono::time_point<mj::Simulate::Clock> syncCPU;
//   mjtNum syncSim = 0;

//   // run until asked to exit
//   while (!sim.exitrequest.load()) {
//     if (sim.droploadrequest.load()) {
//       sim.LoadMessage(sim.dropfilename);
//       mjModel* mnew = LoadModel(sim.dropfilename, sim);
//       sim.droploadrequest.store(false);

//       mjData* dnew = nullptr;
//       if (mnew) dnew = mj_makeData(mnew);
//       if (dnew) {
//         sim.Load(mnew, dnew, sim.dropfilename);

//         // lock the sim mutex
//         const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

//         mj_deleteData(d);
//         mj_deleteModel(m);

//         m = mnew;
//         d = dnew;
//         mj_forward(m, d);

//       } else {
//         sim.LoadMessageClear();
//       }
//     }

//     if (sim.uiloadrequest.load()) {
//       sim.uiloadrequest.fetch_sub(1);
//       sim.LoadMessage(sim.filename);
//       mjModel* mnew = LoadModel(sim.filename, sim);
//       mjData* dnew = nullptr;
//       if (mnew) dnew = mj_makeData(mnew);
//       if (dnew) {
//         sim.Load(mnew, dnew, sim.filename);

//         // lock the sim mutex
//         const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

//         mj_deleteData(d);
//         mj_deleteModel(m);

//         m = mnew;
//         d = dnew;
//         mj_forward(m, d);

//       } else {
//         sim.LoadMessageClear();
//       }
//     }

//     // sleep for 1 ms or yield, to let main thread run
//     //  yield results in busy wait - which has better timing but kills battery life
//     if (sim.run && sim.busywait) {
//       std::this_thread::yield();
//     } else {
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }

//     {
//       // lock the sim mutex
//       const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

//       // run only if model is present
//       if (m) {
//         // running
//         if (sim.run) {
//           bool stepped = false;

//           // record cpu time at start of iteration
//           const auto startCPU = mj::Simulate::Clock::now();

//           // elapsed CPU and simulation time since last sync
//           const auto elapsedCPU = startCPU - syncCPU;
//           double elapsedSim = d->time - syncSim;

//           // requested slow-down factor
//           double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

//           // misalignment condition: distance from target sim time is bigger than syncmisalign
//           bool misaligned =
//               std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

//           // out-of-sync (for any reason): reset sync times, step
//           if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
//               misaligned || sim.speed_changed) {
//             // re-sync
//             syncCPU = startCPU;
//             syncSim = d->time;
//             sim.speed_changed = false;

//             // run single step, let next iteration deal with timing
//             mj_step(m, d);
//             const char* message = Diverged(m->opt.disableflags, d);
//             if (message) {
//               sim.run = 0;
//               mju::strcpy_arr(sim.load_error, message);
//             } else {
//               stepped = true;
//             }
//           }

//           // in-sync: step until ahead of cpu
//           else {
//             bool measured = false;
//             mjtNum prevSim = d->time;

//             double refreshTime = simRefreshFraction/sim.refresh_rate;

//             // step while sim lags behind cpu and within refreshTime
//             while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
//                    mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
//               // measure slowdown before first step
//               if (!measured && elapsedSim) {
//                 sim.measured_slowdown =
//                     std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
//                 measured = true;
//               }

//               // inject noise
//               sim.InjectNoise();

//               // call mj_step
//               mj_step(m, d);
//               const char* message = Diverged(m->opt.disableflags, d);
//               if (message) {
//                 sim.run = 0;
//                 mju::strcpy_arr(sim.load_error, message);
//               } else {
//                 stepped = true;
//               }

//               // break if reset
//               if (d->time < prevSim) {
//                 break;
//               }
//             }
//           }

//           // save current state to history buffer
//           if (stepped) {
//             sim.AddToHistory();
//           }
//         }

//         // paused
//         else {
//           // run mj_forward, to update rendering and joint sliders
//           mj_forward(m, d);
//           sim.speed_changed = true;
//         }
//       }
//     }  // release std::lock_guard<std::mutex>
//   }
// }
// }
// //-------------------------------------- physics_thread --------------------------------------------

// void PhysicsThread(mj::Simulate* sim, const char* filename) {
//     // request loadmodel if file given (otherwise drag-and-drop)
//     if (filename != nullptr) {
//         sim->LoadMessage(filename);
//         m = LoadModel(filename, *sim);
//         if (m) {
//             // lock the sim mutex
//             const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

//             d = mj_makeData(m);
//         }
//         if (d) {
//             sim->Load(m, d, filename);

//             // lock the sim mutex
//             const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

//             mj_forward(m, d);

//         } else {
//             sim->LoadMessageClear();
//         }
//     }

//   PhysicsLoop(*sim);

//   // delete everything we allocated
//   mj_deleteData(d);
//   mj_deleteModel(m);
// }
// // machinery for replacing command line error by a macOS dialog box when running under Rosetta
// #if defined(__APPLE__) && defined(__AVX__)
// extern void DisplayErrorDialogBox(const char* title, const char* msg);
// static const char* rosetta_error_msg = nullptr;
// __attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
//   rosetta_error_msg = msg;
// }
// #endif
// class TaskRunner
// {
// public:
//     static constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;
//     static constexpr std::size_t NumLink = PiperArmModel<double>::NumLink;

//     explicit TaskRunner(mjModel* m, mjData* d) :
//         mujoco_model_(m), mujoco_data_(d),
//         msg_sender_(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT),
//         left_arm_model_(
//             CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
//             CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW),
//         right_arm_model_(
//             CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
//             CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW),
//         controller(
//             1.0/CONFIG_CONTROLLER_FREQUENCY,
//             {100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
//             {0, 0, 0, 0, 0, 0},
//             {10.0, 10.0, 10.0, 10.0, 10.0}),
//         termination_(false)
//     {
//         // Initialize the messenger
//         msg_sender_.start(true, false);
//     }
//     ~TaskRunner() = default;

//     void start()
//     {
//         // Set termination flag to false
//         termination_ = false;
        
//         execution_thread_ = std::thread([this](){
//             struct timespec wakeup_time = {0, 0};
//             struct timespec cycletime = {0, 0}; // Initialize to zero
            
//             // Convert frequency (Hz) to timespec
//             double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
//             cycletime.tv_sec = static_cast<time_t>(period_seconds);
//             cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

//             clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
//             std::size_t count = 0;
//             while ( !termination_ )
//             {
//                 increaseTimeSpec(&wakeup_time, &cycletime);
//                 clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

//                 updateControl();
                
//                 count++;

//                 if ( count == 10 )
//                 {
//                     updatePlan();

//                     count = 0;
//                 }
//             }
//         });
//     }

//     void stop()
//     {
//         // Set termination flag to true
//         termination_ = true;
        
//         // Join threads if they are joinable
//         if (execution_thread_.joinable())
//         {
//             execution_thread_.join();
//         }
//         // Stop components
//         msg_sender_.stop();
//     }

// private:
//     void updatePlan()
//     {
//         {
//             // Eigen::Vector<double, 3> left_hand_mocap_pos = mj_backend.getLeftHandMocapPosition();
//             // Eigen::Quaternion<double> left_hand_mocap_ori = mj_backend.getLeftHandMocapOrientation();
//             // Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             left_hand_mocap_pose.block<3, 3>(0, 0) = Eigen::Quaternion<double>(0.71, 0, 0.71, 0).toRotationMatrix();
//             left_hand_mocap_pose(0, 3) = 0.6;
//             left_hand_mocap_pose(1, 3) = 0.2;
//             left_hand_mocap_pose(2, 3) = 0.8;
//             // left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori.toRotationMatrix();
//             // left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
//             Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = getLeftArmJointPosition();
//             Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
//             // ErrorCode err = left_arm_model.getDampedLeastSquareInverseKinematics(
//             //     ik_result, left_arm_model, 0.1, Eigen::Vector<double, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, 
//             //     left_hand_mocap_pose, actual_left_arm_joint_pos);
//             ErrorCode err = left_arm_model_.getInverseKinematics(ik_result, left_hand_mocap_pose, actual_left_arm_joint_pos);
//             if ( err == ErrorCode::NoResult )
//             {
//                 LOG_ERROR("left arm inverse kinematics fails.");
//             }
//             auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//             auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
//                 std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
//                 target_left_arm_joint_pos, ik_result);
//             left_arm_trajectory_buffer_.write(time_point, trajectory);
//         }
//         {
//             // Eigen::Vector<double, 3> right_hand_mocap_pos = mj_backend.getLeftHandMocapPosition();
//             // Eigen::Quaternion<double> right_hand_mocap_ori = mj_backend.getLeftHandMocapOrientation();
//             // Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
//             right_hand_mocap_pose.block<3, 3>(0, 0) = Eigen::Quaternion<double>(0.71, 0, 0.71, 0).toRotationMatrix();
//             right_hand_mocap_pose(0, 3) = 0.6;
//             right_hand_mocap_pose(1, 3) = -0.2;
//             right_hand_mocap_pose(2, 3) = 0.8;
//             // right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori.toRotationMatrix();
//             // right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
//             Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = getRightArmJointPosition();
//             Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
//             // ErrorCode err = right_arm_model.getDampedLeastSquareInverseKinematics(
//             //     ik_result, right_arm_model, 0.1, Eigen::Vector<double, 6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, 
//             //     right_hand_mocap_pose, actual_right_arm_joint_pos);
//             ErrorCode err = right_arm_model_.getInverseKinematics(ik_result, right_hand_mocap_pose, actual_right_arm_joint_pos);
//             if ( err == ErrorCode::NoResult )
//             {
//                 LOG_ERROR("right arm inverse kinematics fails.");
//             }
//             auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//             auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
//                 std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
//                 target_right_arm_joint_pos, ik_result);
//             right_arm_trajectory_buffer_.write(time_point, trajectory);
//         }
//     }

//     void updateControl()
//     {
//         {
//             auto [target_joint_pos, target_joint_vel, target_joint_acc] = left_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//             JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
//             JointState<double, NumDof> current_joint_state = {getLeftArmJointPosition(), getLeftArmJointVelocity(), getLeftArmJointAcceleration()};
//             auto target_joint_torque = controller.computeTorqueControlOutput(left_arm_model_, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
//             setLeftArmJointControl(target_joint_torque);
//             LOG_TRACE("left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
//             LOG_TRACE("left arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
//             LOG_TRACE("left arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
//             LOG_TRACE("left arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
//             LOG_TRACE("left arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
//                 current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
//             LOG_TRACE("left arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
//                 current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));
//         }
//         {
//             auto [target_joint_pos, target_joint_vel, target_joint_acc] = right_arm_trajectory_buffer_.interpolate(std::chrono::steady_clock::now());
//             JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
//             JointState<double, NumDof> current_joint_state = {getRightArmJointPosition(), getRightArmJointVelocity(), getRightArmJointAcceleration()};
//             auto target_joint_torque = controller.computeTorqueControlOutput(right_arm_model_, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
//             setRightArmJointControl(target_joint_torque);
//             LOG_TRACE("right arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
//             LOG_TRACE("right arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
//             LOG_TRACE("right arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
//             LOG_TRACE("right arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
//             LOG_TRACE("right arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
//                 current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
//             LOG_TRACE("right arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
//                 current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
//                 current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));            
//         }

//     }

//     // Components
//     Messenger<ChannelMode::UDP> msg_sender_;
//     // MujocoBackend<double, NumDof> mj_backend_;
//     // MujocoFrontend mj_frontend_;
//     mjModel* mujoco_model_;
//     mjData* mujoco_data_;
//     PiperArmModel<double> left_arm_model_;
//     PiperArmModel<double> right_arm_model_;
//     ArmController<double, NumLink, NumDof> controller;
//     BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer_;
//     BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer_;

//     // Threads
//     std::thread execution_thread_;

//     // Termination flag
//     bool termination_;
//     void setLeftArmJointControl(Eigen::Vector<double, NumDof> q)
//     {
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[i] = q(i);
//         }
//     }

//     void setLeftArmJointControl(std::array<double, NumDof> q)
//     {
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[i] = q[i];
//         }
//     }

//     void setLeftArmJointControl(std::vector<double> q)
//     {
//         if ( q.size() != NumDof )
//         {
//             return ;
//         }
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[i] = q[i];
//         }
//     }

//     void setRightArmJointControl(Eigen::Vector<double, NumDof> q)
//     {
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[NumDof + 2 + i] = q(i); // 2 for two finger joints.
//         }
//     }

//     void setRightArmJointControl(std::array<double, NumDof> q)
//     {
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[NumDof + 2 + i] = q[i]; // 2 for two finger joints.
//         }
//     }

//     void setRightArmJointControl(std::vector<double> q)
//     {
//         if ( q.size() != NumDof )
//         {
//             return ;
//         }
//         for ( int i=0 ; i<NumDof ; ++i )
//         {
//             this->mujoco_data_->ctrl[NumDof + 2 + i] = q[i]; // 2 for two finger joints.
//         }
//     }

//     Eigen::Vector<double, NumDof> getLeftArmJointPosition() const
//     {
//         Eigen::Vector<double, NumDof> joint_pos;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_pos(i) = this->mujoco_data_->qpos[i];
//         }
//         return joint_pos;
//     }

//     Eigen::Vector<double, NumDof> getLeftArmJointVelocity() const
//     {
//         Eigen::Vector<double, NumDof> joint_vel;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_vel(i) = this->mujoco_data_->qvel[i];
//         }
//         return joint_vel;
//     }

//     Eigen::Vector<double, NumDof> getLeftArmJointAcceleration() const
//     {
//         Eigen::Vector<double, NumDof> joint_acc;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_acc(i) = this->mujoco_data_->qacc[i];
//         }
//         return joint_acc;
//     }

//     Eigen::Vector<double, NumDof> getRightArmJointPosition() const
//     {
//         Eigen::Vector<double, NumDof> joint_pos;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_pos(i) = this->mujoco_data_->qpos[NumDof + 2 + i];
//         }
//         return joint_pos;
//     }

//     Eigen::Vector<double, NumDof> getRightArmJointVelocity() const
//     {
//         Eigen::Vector<double, NumDof> joint_vel;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_vel(i) = this->mujoco_data_->qvel[NumDof + 2 + i];
//         }
//         return joint_vel;
//     }

//     Eigen::Vector<double, NumDof> getRightArmJointAcceleration() const
//     {
//         Eigen::Vector<double, NumDof> joint_acc;
//         for ( std::size_t i=0 ; i<NumDof ; ++i )
//         {
//             joint_acc(i) = this->mujoco_data_->qacc[NumDof + 2 + i];
//         }
//         return joint_acc;
//     }

//     Eigen::Vector<double, 3> getLeftHandMocapPosition() const
//     {
//         Eigen::Vector<double, 3> pos;
//         pos(0) = this->mujoco_data_->mocap_pos[0];
//         pos(1) = this->mujoco_data_->mocap_pos[1];
//         pos(2) = this->mujoco_data_->mocap_pos[2];
//         return pos;
//     }
//     Eigen::Vector<double, 3> getRightHandMocapPosition() const
//     {
//         Eigen::Vector<double, 3> pos;
//         pos(0) = this->mujoco_data_->mocap_pos[3];
//         pos(1) = this->mujoco_data_->mocap_pos[4];
//         pos(2) = this->mujoco_data_->mocap_pos[5];
//         return pos;
//     }
//     Eigen::Quaternion<double> getLeftHandMocapOrientation() const
//     {
//         Eigen::Quaternion<double> quat;
//         quat.w() = this->mujoco_data_->mocap_quat[0];
//         quat.x() = this->mujoco_data_->mocap_quat[1];
//         quat.y() = this->mujoco_data_->mocap_quat[2];
//         quat.z() = this->mujoco_data_->mocap_quat[3];
//         return quat;
//     }
//     Eigen::Quaternion<double> getRightHandMocapOrientation() const
//     {
//         Eigen::Quaternion<double> quat;
//         quat.w() = this->mujoco_data_->mocap_quat[4];
//         quat.x() = this->mujoco_data_->mocap_quat[5];
//         quat.y() = this->mujoco_data_->mocap_quat[6];
//         quat.z() = this->mujoco_data_->mocap_quat[7];
//         return quat;
//     }
// };
int main(void)
{
    constexpr std::size_t NumDof = PiperArmModel<double>::NumDof;

    /* Create log file. */
    initLogger(PROJECT_PATH"/logs", "server");
    /* Initialize program stop flag as false. */
    TerminationHandler::stop_requested = false;
    /* Register SIGINT handler. */
    TerminationHandler::setup();

    Messenger<ChannelMode::UDP> msg_sender(CONFIG_SERVER_IPV4_ADDRESS, CONFIG_SERVER_PORT, CONFIG_CLIENT_IPV4_ADDRESS, CONFIG_CLIENT_PORT);
    msg_sender.start(true, false);

    // MujocoBackend<double, NumDof> mj_backend;
    // MujocoFrontend mj_frontend(mj_backend.getMujocoModel(), mj_backend.getMujocoData());
    // mj_backend.start();
    // mj_frontend.start();
    ArmSimulationInterface<double, NumDof> interface;
    interface.start(PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque_full.xml");

    PiperArmModel<double> left_arm_model(
        CONFIG_LEFT_ARM_BASE_X, CONFIG_LEFT_ARM_BASE_Y, CONFIG_LEFT_ARM_BASE_Z,
        CONFIG_LEFT_ARM_BASE_ROLL, CONFIG_LEFT_ARM_BASE_PITCH, CONFIG_LEFT_ARM_BASE_YAW);

    PiperArmModel<double> right_arm_model(
        CONFIG_RIGHT_ARM_BASE_X, CONFIG_RIGHT_ARM_BASE_Y, CONFIG_RIGHT_ARM_BASE_Z,
        CONFIG_RIGHT_ARM_BASE_ROLL, CONFIG_RIGHT_ARM_BASE_PITCH, CONFIG_RIGHT_ARM_BASE_YAW);
    
    ArmController<double, PiperArmModel<double>::NumLink, NumDof> controller(
        1.0/CONFIG_CONTROLLER_FREQUENCY,
        {100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
        {0, 0, 0, 0, 0, 0},
        {10.0, 10.0, 10.0, 10.0, 10.0});
    BsplineTrajectoryBuffer<double, NumDof> left_arm_trajectory_buffer;
    BsplineTrajectoryBuffer<double, NumDof> right_arm_trajectory_buffer;

    auto update_plan_left_arm = [&]()
    {
        Eigen::Vector<double, 3> left_hand_mocap_pos = interface.getLeftHandSitePosition();
        Eigen::Matrix<double, 3, 3> left_hand_mocap_ori = interface.getLeftHandSiteOrientation();
        Eigen::Matrix<double, 4, 4> left_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
        left_hand_mocap_pose.block<3, 3>(0, 0) = left_hand_mocap_ori;
        left_hand_mocap_pose.block<3, 1>(0, 3) = left_hand_mocap_pos;
        Eigen::Vector<double, NumDof> actual_left_arm_joint_pos = interface.getLeftArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( left_arm_model.getInverseKinematics(ik_result, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult)
        {
            if ( left_arm_model.getDampedLeastSquareInverseKinematics(ik_result, left_arm_model, left_hand_mocap_pose, actual_left_arm_joint_pos) == ErrorCode::NoResult )
            {
                LOG_WARN("left arm inverse kinematics fails.");
                return ;
            }
        }
        auto [target_left_arm_joint_pos, left_arm_joint_vel, left_arm_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_left_arm_joint_pos, ik_result);
        left_arm_trajectory_buffer.write(time_point, trajectory);
    };

    auto update_plan_right_arm = [&]()
    {
        Eigen::Vector<double, 3> right_hand_mocap_pos = interface.getRightHandSitePosition();
        Eigen::Matrix<double, 3, 3> right_hand_mocap_ori = interface.getRightHandSiteOrientation();
        Eigen::Matrix<double, 4, 4> right_hand_mocap_pose = Eigen::Matrix<double, 4, 4>::Identity();
        right_hand_mocap_pose.block<3, 3>(0, 0) = right_hand_mocap_ori;
        right_hand_mocap_pose.block<3, 1>(0, 3) = right_hand_mocap_pos;
        Eigen::Vector<double, NumDof> actual_right_arm_joint_pos = interface.getRightArmJointPosition();
        Eigen::Vector<double, NumDof> ik_result = Eigen::Vector<double, NumDof>::Zero();
        if ( right_arm_model.getInverseKinematics(ik_result, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
        {
            if ( right_arm_model.getDampedLeastSquareInverseKinematics(ik_result, right_arm_model, right_hand_mocap_pose, actual_right_arm_joint_pos) == ErrorCode::NoResult )
            {
                LOG_WARN("right arm inverse kinematics fails.");
                return ;
            }
        }
        auto [target_right_arm_joint_pos, right_arm_joint_vel, right_arm_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
        auto [time_point, trajectory] = LinearArmPlanner::plan<double, NumDof, CONFIG_WAYPOINT_AMOUNT>(
            std::chrono::steady_clock::now(), 1.0/CONFIG_PLANNER_FREQUENCY,
            target_right_arm_joint_pos, ik_result);
        right_arm_trajectory_buffer.write(time_point, trajectory);                
    };

    auto update_control = [&]()
    {
        std::array<double, NumDof> left_arm_target_joint_pos, right_arm_target_joint_pos;
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = left_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            for ( int i=0 ; i<NumDof ; i++ )
                left_arm_target_joint_pos[i] = target_joint_pos(i);
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {interface.getLeftArmJointPosition(), interface.getLeftArmJointVelocity(), interface.getLeftArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(left_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            interface.setLeftArmJointControl(target_joint_torque);
            LOG_TRACE("left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
            LOG_TRACE("left arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
            LOG_TRACE("left arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
            LOG_TRACE("left arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
            LOG_TRACE("left arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
                current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
            LOG_TRACE("left arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
                current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));
        }
        {
            auto [target_joint_pos, target_joint_vel, target_joint_acc] = right_arm_trajectory_buffer.interpolate(std::chrono::steady_clock::now());
            for ( int i=0 ; i<NumDof ; i++ )
                right_arm_target_joint_pos[i] = target_joint_pos(i);
            JointState<double, NumDof> target_joint_state = {target_joint_pos, target_joint_vel, target_joint_acc};
            JointState<double, NumDof> current_joint_state = {interface.getRightArmJointPosition(), interface.getRightArmJointVelocity(), interface.getRightArmJointAcceleration()};
            auto target_joint_torque = controller.computeTorqueControlOutput(right_arm_model, Eigen::Vector<double, 3>::Zero(), target_joint_state, current_joint_state);
            interface.setRightArmJointControl(target_joint_torque);
            LOG_TRACE("right arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_pos(0), target_joint_pos(1), target_joint_pos(2), target_joint_pos(3), target_joint_pos(4), target_joint_pos(5));
            LOG_TRACE("right arm target joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_vel(0), target_joint_vel(1), target_joint_vel(2), target_joint_vel(3), target_joint_vel(4), target_joint_vel(5));
            LOG_TRACE("right arm target joint acceleration:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_acc(0), target_joint_acc(1), target_joint_acc(2), target_joint_acc(3), target_joint_acc(4), target_joint_acc(5));
            LOG_TRACE("right arm target joint torque:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                target_joint_torque(0), target_joint_torque(1), target_joint_torque(2), target_joint_torque(3), target_joint_torque(4), target_joint_torque(5));
            LOG_TRACE("right arm current joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_pos(0), current_joint_state.joint_pos(1), current_joint_state.joint_pos(2),
                current_joint_state.joint_pos(3), current_joint_state.joint_pos(4), current_joint_state.joint_pos(5));
            LOG_TRACE("right arm current joint velocity:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                current_joint_state.joint_vel(0), current_joint_state.joint_vel(1), current_joint_state.joint_vel(2),
                current_joint_state.joint_vel(3), current_joint_state.joint_vel(4), current_joint_state.joint_vel(5));            
        }
        msg_sender.send<double>(true, left_arm_target_joint_pos, right_arm_target_joint_pos);
    };

    struct timespec wakeup_time = {0, 0};
    struct timespec cycletime = {0, 0}; // Initialize to zero
    
    // Convert frequency (Hz) to timespec
    double period_seconds = 1.0 / static_cast<double>(CONFIG_CONTROLLER_FREQUENCY);
    cycletime.tv_sec = static_cast<time_t>(period_seconds);
    cycletime.tv_nsec = static_cast<long>((period_seconds - cycletime.tv_sec) * 1e9);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    std::size_t count = 0;
    while ( !TerminationHandler::stop_requested )
    {
        increaseTimeSpec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        update_control();
        
        count++;

        if ( count == 10 )
        {
            update_plan_left_arm();
            update_plan_right_arm();

            count = 0;
        }
    }

    msg_sender.stop();
    interface.stop();
    // mj_backend.stop();
    // mj_frontend.stop();

    // // display an error if running on macOS under Rosetta 2
    // #if defined(__APPLE__) && defined(__AVX__)
    // if (rosetta_error_msg) {
    //     DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    //     std::exit(1);
    // }
    // #endif

    // // print version, check compatibility
    // std::printf("MuJoCo version %s\n", mj_versionString());
    // if (mjVERSION_HEADER!=mj_version()) {
    //     mju_error("Headers and library have different versions");
    // }

    // // scan for libraries in the plugin directory to load additional plugins
    // scanPluginLibraries();

    // mjvCamera cam;
    // mjv_defaultCamera(&cam);

    // mjvOption opt;
    // mjv_defaultOption(&opt);

    // mjvPerturb pert;
    // mjv_defaultPerturb(&pert);

    // // simulate object encapsulates the UI
    // auto sim = std::make_unique<mj::Simulate>(
    //     std::make_unique<mj::GlfwAdapter>(),
    //     &cam, &opt, &pert, /* is_passive = */ false
    // );

    // const char* filename = PROJECT_PATH"/assets/mujoco_model/piper_dual_arm_torque_full.xml";

    // // start physics thread
    // std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

    // // start simulation UI loop (blocking call)
    // sim->RenderLoop();
    // physicsthreadhandle.join();

    spdlog::drop_all();

    return 0;
}
