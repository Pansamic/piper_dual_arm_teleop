/**
 * @file arm_simulation_interface.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm communication and packet parser.
 * @version 0.1
 * @date 2025-05-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ARM_SIMULATION_INTERFACE_H__
#define __ARM_SIMULATION_INTERFACE_H__

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
#include <arm_model.h>
#include <arm_interface.h>

class ArmSimulationInterface final : public ArmInterface
{
public:
    ArmSimulationInterface(const char* mujoco_file_path);
    ~ArmSimulationInterface();

    void setLeftJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque) override;
    void setLeftGripperControl(const double& position, const double& torque) override;
    void setRightJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque) override;
    void setRightGripperControl(const double& position, const double& torque) override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointPosition() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointVelocity() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointAcceleration() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointTorque() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointPosition() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointVelocity() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointAcceleration() const override;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointTorque() const override;
private:
    const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
    const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
    const int kErrorLength = 1024;          // load error string length
    const double joint_kp_ = 10.0;
    const double joint_kd_ = 0.8;

    mjModel* m = nullptr; // MuJoCo model
    mjData* d = nullptr; // MuJoCo data

    std::atomic<bool> running_;

    std::unique_ptr<mujoco::Simulate> sim_;
    std::thread physics_thread_;
    std::thread render_thread_;
    
    JointState left_arm_target_state_;
    JointState left_arm_actual_state_;
    JointState right_arm_target_state_;
    JointState right_arm_actual_state_;

    std::mutex left_arm_mutex_;
    std::mutex right_arm_mutex_;

    void setJointPDControl();
    mjModel* loadModel(const char* file);
    const char* diverged(int disableflags, const mjData* d);
    void physicsLoop();
    void threadPhysics(const char* mujoco_file_path);
};

#endif // __ARM_SIMULATION_INTERFACE_H__