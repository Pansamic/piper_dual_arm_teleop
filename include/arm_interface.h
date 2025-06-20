/**
 * @file arm_interface.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm communication and packet parser.
 * @version 0.1
 * @date 2025-05-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ARM_INTERFACE_H__
#define __ARM_INTERFACE_H__

#include <thread>
#include <mutex>
#include <atomic>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <glfw_adapter.h>
#include <simulate.h>
#include <array_safety.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arm_model.h>

template<typename Derived>
class ArmInterface
{
public:
    void initialize()
    {
        static_cast<Derived*>(this)->initializeImpl();
    }

    bool isReady() const
    {
        return static_cast<const Derived*>(this)->isReadyImpl();
    }

    void setLeftJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
    {
        static_cast<Derived*>(this)->setLeftJointControl(joint_pos, joint_vel, joint_feedforward_torque);
    }

    void setLeftGripperControl(const double& position, const double& torque)
    {
        static_cast<Derived*>(this)->setLeftGripperControlImpl(position, torque);
    }

    void setRightJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque)
    {
        static_cast<Derived*>(this)->setRightJointControl(joint_pos, joint_vel, joint_feedforward_torque);
    }

    void setRightGripperControl(const double& position, const double& torque)
    {
        static_cast<Derived*>(this)->setRightGripperControlImpl(position, torque);
    }

    Eigen::Vector<double,ArmModel::num_dof_> getLeftJointPosition() const
    {
        return static_cast<const Derived*>(this)->getLeftJointPositionImpl();
    }

    Eigen::Vector<double,ArmModel::num_dof_> getLeftJointVelocity() const
    {
        return static_cast<const Derived*>(this)->getLeftJointVelocityImpl();
    }

    Eigen::Vector<double,ArmModel::num_dof_> getRightJointPosition() const
    {
        return static_cast<const Derived*>(this)->getRightJointPositionImpl();
    }

    Eigen::Vector<double,ArmModel::num_dof_> getRightJointVelocity() const
    {
        return static_cast<const Derived*>(this)->getRightJointVelocityImpl();
    }
};

class ArmSimulationInterface : public ArmInterface<ArmSimulationInterface>
{
public:
    ArmSimulationInterface();
    ~ArmSimulationInterface();

    void initializeImpl();
    bool isReadyImpl();
    void setLeftJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque);
    void setLeftGripperControlImpl(const double& position, const double& torque);
    void setRightJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque);
    void setRightGripperControlImpl(const double& position, const double& torque);
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointPositionImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointVelocityImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointTorqueImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointPositionImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointVelocityImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointTorqueImpl() const;
private:

    const char* mujoco_file_ = PROJECT_PATH"/assets/mujoco_model/pathfinder.xml";
    const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
    const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
    const int kErrorLength = 1024;          // load error string length
    const double joint_kp_ = 10.0;
    const double joint_kd_ = 0.8;

    mjModel* m = nullptr; // MuJoCo model
    mjData* d = nullptr; // MuJoCo data

    bool running_;

    std::unique_ptr<mujoco::Simulate> sim_;
    std::thread physics_thread_;
    std::thread render_thread_;

    Eigen::Vector<double, ArmModel::num_dof_> left_target_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> left_target_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> left_target_joint_fd_torq_;
    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_torq_;

    Eigen::Vector<double, ArmModel::num_dof_> right_target_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> right_target_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> right_target_joint_fd_torq_;
    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_torq_;

    std::mutex left_joints_mutex_;
    std::mutex right_joints_mutex_;

    void setJointPDControl();
    mjModel* loadModel(const char* file);
    const char* diverged(int disableflags, const mjData* d);
    void physicsLoop();
    void threadPhysics();
};

class ArmHardwareInterface : public ArmInterface<ArmHardwareInterface>
{
public:
    ArmHardwareInterface(const char* left_arm_can_name, const char* right_arm_can_name);
    ~ArmHardwareInterface();

    void initializeImpl();
    bool isReadyImpl();
    void setLeftJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque);
    void setLeftGripperControlImpl(const double& position, const double& torque);
    void setRightJointControl(
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_feedforward_torque);
    void setRightGripperControlImpl(const double& position, const double& torque);
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointPositionImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointVelocityImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getLeftJointTorqueImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointPositionImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointVelocityImpl() const;
    const Eigen::Vector<double,ArmModel::num_dof_>& getRightJointTorqueImpl() const;

private:
    enum ArmControlMode
    {
        MODE_STANDBY = 0,
        MODE_CAN,
        MODE_TEACH,
        MODE_ETHERNET,
        MODE_WIFI,
        MODE_REMOTE,
        MODE_DUAL_TEACH,
        MODE_OFFLINE_TRAJECTORY
    };

    enum ArmStatus
    {
        STATUS_NORMAL,
        STATUS_EMERGENCY_STOP,
        STATUS_NO_SOLUTION,
        STATUS_SINGULAR,
        STATUS_OVER_POSITION_LIMIT,
        STATUS_JOINT_COMMUNICATION_ERROR,
        STATUS_JOINT_BRAKE_UNRELEASED,
        STATUS_SELF_COLLISION,
        STATUS_TEACH_OVERSPEED,
        STATUS_JOINT_EXCEPTION,
        STATUS_OTHER_EXCEPTION,
        STATUS_TEACH_RECORD,
        STATUS_TEACH_EXECUTE,
        STATUS_TEACH_SUSPEND,
        STATUS_NTC_OVER_TEMPERATURE,
        STATUS_RELEASE_RES_OVER_TEMPERATURE
    };

    enum MoveMode
    {
        MOVE_MODE_POSITION = 0,
        MOVE_MODE_JOINT,
        MOVE_MODE_LINEAR,
        MOVE_MODE_CIRCULAR,
        MOVE_MODE_MIT
    };

    enum TeachStatus
    {
        TEACH_STATE_OFF = 0,
        TEACH_STATE_START,
        TEACH_STATE_END,
        TEACH_STATE_EXECUTE,
        TEACH_STATE_SUSPEND,
        TEACH_STATE_RESUME,
        TEACH_STATE_TERMINATE,
        TEACH_STATE_TRAJ_BEGIN
    };

    enum MotionStatus
    {
        MOTION_REACH = 0,
        MOTION_NOT_REACH
    };

    enum ActuatorStatus
    {
        ACTUATOR_STATUS_LOW_VOLTAGE = 1<<0,
        ACTUATOR_STATUS_MOTOR_OVER_TEMPERATURE = 1<<1,
        ACTUATOR_STATUS_OVER_CURRENT = 1<<2,
        ACTUATOR_STATUS_OVER_TEMPERATURE = 1<<3,
        ACTUATOR_STATUS_COLLISION_PROTECT = 1<<4,
        ACTUATOR_STATUS_ERROR = 1<<5,
        ACTUATOR_STATUS_DISABLED = 1<<6,
        ACTUATOR_STATUS_LOCKED_ROTOR = 1<<7
    };

    /* runing flag */
    bool running_;

    /* CAN socket id */
    int left_can_socket_;
    int right_can_socket_;

    /* thread for CAN polling receiption */
    std::thread read_thread_;

    /* CAN socket mutex */
    std::mutex left_arm_can_mtx_;
    std::mutex right_arm_can_mtx_;
    
    std::mutex left_joints_mutex_;
    std::mutex right_joints_mutex_;

    std::array<float,ArmModel::num_dof_> left_arm_actuator_voltage_;
    std::array<int16_t,ArmModel::num_dof_> left_arm_actuator_temperature_;
    std::array<int8_t,ArmModel::num_dof_> left_arm_motor_temperature_;
    std::array<uint8_t,ArmModel::num_dof_> left_arm_actuator_status_;

    std::array<float,ArmModel::num_dof_> right_arm_actuator_voltage_;
    std::array<int16_t,ArmModel::num_dof_> right_arm_actuator_temperature_;
    std::array<int8_t,ArmModel::num_dof_> right_arm_motor_temperature_;
    std::array<uint8_t,ArmModel::num_dof_> right_arm_actuator_status_;

    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> left_actual_joint_torq_;

    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_pos_;
    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_vel_;
    Eigen::Vector<double, ArmModel::num_dof_> right_actual_joint_torq_;

    void parseCanFrame(const int can_socket, struct can_frame* can_frame);

    /**
     * @brief Send MIT protocol command to piper manipulator.
     * 
     * @param actuator_id from 0 to 5.
     * @param position position without joint limit bound. unit: rad.
     * @param velocity velocity without joint velocity bound. unit: rad/s.
     * @param kp 
     * @param kd 
     * @param torque 
     */
    void sendControlMessage(
        const int can_socket,
        const uint8_t actuator_id,
        const int16_t position,
        const int16_t velocity,
        const int16_t kp,
        const int16_t kd,
        const int8_t torque) const;
    void setMITMode(const int can_socket) const;
    void setJointPositionMode(const int can_socket) const;
    void setEndEffectorPoseMode(const int can_socket) const;
    void enableActuators(const int can_socket) const;
    void disableActuators(const int can_socket) const;
    void setActuatorControl(
        const int can_socket,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel,
        const Eigen::Vector<double,ArmModel::num_dof_>& joint_torque) const;
    void setActuatorPosition(const int can_socket, const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos) const;
    void setEndEffectorPose(const int can_socket, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation) const;
    void setGripperCurrentAsZero(const int can_socket) const;
    void setGripperControl(const int can_socket, const double position, const double torque) const;
    void threadCanReceive();
    // void threadCanTransmit();
};

#endif // __ARM_INTERFACE_H__