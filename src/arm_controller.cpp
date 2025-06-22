/**
 * @file arm_controller.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Teleoperation controller.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <Eigen/Core>
#include <arm_model.h>
#include <arm_controller.h>

const Eigen::Vector<double,ArmModel::num_dof_> ArmController::joint_kp_ = Eigen::Vector<double,ArmModel::num_dof_>(100,100,100,100,100,100);

const Eigen::Vector<double,ArmModel::num_dof_> ArmController::joint_kd_ = Eigen::Vector<double,ArmModel::num_dof_>(20,20,20,20,20,20);

ArmController::ArmController(std::unique_ptr<ArmModel> model, std::unique_ptr<ArmInterface> interface, const unsigned int control_interval):
model_(std::move(model)), interface_(std::move(interface)), running_(true), control_thread_(&ArmController::control_loop_, this)
{

}

ArmController::~ArmController()
{
    this->running_ = false;
    if ( this->control_thread_.joinable() )
    {
        control_thread_.join();  // Ensure thread finishes before destruction
    }
}


void ArmController::setLeftTargetJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_pos_ = joint_pos;
}

void ArmController::setLeftTargetJointPosition(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_pos_(0) = j1;
    this->left_target_joint_pos_(1) = j2;
    this->left_target_joint_pos_(2) = j3;
    this->left_target_joint_pos_(3) = j4;
    this->left_target_joint_pos_(4) = j5;
    this->left_target_joint_pos_(5) = j6;
}

void ArmController::setLeftTargetJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_vel_ = joint_vel;
}

void ArmController::setLeftTargetJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_vel_(0) = j1;
    this->left_target_joint_vel_(1) = j2;
    this->left_target_joint_vel_(2) = j3;
    this->left_target_joint_vel_(3) = j4;
    this->left_target_joint_vel_(4) = j5;
    this->left_target_joint_vel_(5) = j6;
}

void ArmController::setLeftTargetJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_acc_ = joint_vel;
}

void ArmController::setLeftTargetJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_acc_(0) = j1;
    this->left_target_joint_acc_(1) = j2;
    this->left_target_joint_acc_(2) = j3;
    this->left_target_joint_acc_(3) = j4;
    this->left_target_joint_acc_(4) = j5;
    this->left_target_joint_acc_(5) = j6;
}

void ArmController::setLeftTargetJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_torq_ = joint_torq;
}

void ArmController::setLeftTargetJointTorque(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_target_state_mtx_);
    this->left_target_joint_torq_(0) = j1;
    this->left_target_joint_torq_(1) = j2;
    this->left_target_joint_torq_(2) = j3;
    this->left_target_joint_torq_(3) = j4;
    this->left_target_joint_torq_(4) = j5;
    this->left_target_joint_torq_(5) = j6;
}

void ArmController::setRightTargetJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    this->right_target_joint_pos_ = joint_pos;
}

void ArmController::setRightTargetJointPosition(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_pos_(0) = j1;
    this->right_target_joint_pos_(1) = j2;
    this->right_target_joint_pos_(2) = j3;
    this->right_target_joint_pos_(3) = j4;
    this->right_target_joint_pos_(4) = j5;
    this->right_target_joint_pos_(5) = j6;
}

void ArmController::setRightTargetJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_vel_ = joint_vel;
}

void ArmController::setRightTargetJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_vel_(0) = j1;
    this->right_target_joint_vel_(1) = j2;
    this->right_target_joint_vel_(2) = j3;
    this->right_target_joint_vel_(3) = j4;
    this->right_target_joint_vel_(4) = j5;
    this->right_target_joint_vel_(5) = j6;
}

void ArmController::setRightTargetJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_acc_ = joint_vel;
}

void ArmController::setRightTargetJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_acc_(0) = j1;
    this->right_target_joint_acc_(1) = j2;
    this->right_target_joint_acc_(2) = j3;
    this->right_target_joint_acc_(3) = j4;
    this->right_target_joint_acc_(4) = j5;
    this->right_target_joint_acc_(5) = j6;
}

void ArmController::setRightTargetJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_torq_ = joint_torq;
}

void ArmController::setRightTargetJointTorque(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_target_state_mtx_);
    this->right_target_joint_torq_(0) = j1;
    this->right_target_joint_torq_(1) = j2;
    this->right_target_joint_torq_(2) = j3;
    this->right_target_joint_torq_(3) = j4;
    this->right_target_joint_torq_(4) = j5;
    this->right_target_joint_torq_(5) = j6;
}

void ArmController::updateLeftActualJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_pos_ = joint_pos;
}

void ArmController::updateLeftActualJointPosition(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_pos_(0) = j1;
    this->left_actual_joint_pos_(1) = j2;
    this->left_actual_joint_pos_(2) = j3;
    this->left_actual_joint_pos_(3) = j4;
    this->left_actual_joint_pos_(4) = j5;
    this->left_actual_joint_pos_(5) = j6;
}

void ArmController::updateLeftActualJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_vel_ = joint_vel;
}

void ArmController::updateLeftActualJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_vel_(0) = j1;
    this->left_actual_joint_vel_(1) = j2;
    this->left_actual_joint_vel_(2) = j3;
    this->left_actual_joint_vel_(3) = j4;
    this->left_actual_joint_vel_(4) = j5;
    this->left_actual_joint_vel_(5) = j6;
}

void ArmController::updateLeftActualJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_acc_ = joint_vel;
}

void ArmController::updateLeftActualJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_acc_(0) = j1;
    this->left_actual_joint_acc_(1) = j2;
    this->left_actual_joint_acc_(2) = j3;
    this->left_actual_joint_acc_(3) = j4;
    this->left_actual_joint_acc_(4) = j5;
    this->left_actual_joint_acc_(5) = j6;
}

void ArmController::updateLeftActualJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_torq_ = joint_torq;
}

void ArmController::updateLeftActualJointTorque(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->left_actual_state_mtx_);
    this->left_actual_joint_torq_(0) = j1;
    this->left_actual_joint_torq_(1) = j2;
    this->left_actual_joint_torq_(2) = j3;
    this->left_actual_joint_torq_(3) = j4;
    this->left_actual_joint_torq_(4) = j5;
    this->left_actual_joint_torq_(5) = j6;
}

void ArmController::updateRightActualJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_pos_ = joint_pos;
}

void ArmController::updateRightActualJointPosition(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_pos_(0) = j1;
    this->right_actual_joint_pos_(1) = j2;
    this->right_actual_joint_pos_(2) = j3;
    this->right_actual_joint_pos_(3) = j4;
    this->right_actual_joint_pos_(4) = j5;
    this->right_actual_joint_pos_(5) = j6;
}

void ArmController::updateRightActualJointVelocity(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_vel_ = joint_vel;
}

void ArmController::updateRightActualJointVelocity(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_vel_(0) = j1;
    this->right_actual_joint_vel_(1) = j2;
    this->right_actual_joint_vel_(2) = j3;
    this->right_actual_joint_vel_(3) = j4;
    this->right_actual_joint_vel_(4) = j5;
    this->right_actual_joint_vel_(5) = j6;
}

void ArmController::updateRightActualJointAcceleration(const Eigen::Vector<double,ArmModel::num_dof_>& joint_vel)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_acc_ = joint_vel;
}

void ArmController::updateRightActualJointAcceleration(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_acc_(0) = j1;
    this->right_actual_joint_acc_(1) = j2;
    this->right_actual_joint_acc_(2) = j3;
    this->right_actual_joint_acc_(3) = j4;
    this->right_actual_joint_acc_(4) = j5;
    this->right_actual_joint_acc_(5) = j6;
}

void ArmController::updateRightActualJointTorque(const Eigen::Vector<double,ArmModel::num_dof_>& joint_torq)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_torq_ = joint_torq;
}

void ArmController::updateRightActualJointTorque(double j1, double j2, double j3, double j4, double j5, double j6)
{
    std::lock_guard(this->right_actual_state_mtx_);
    this->right_actual_joint_torq_(0) = j1;
    this->right_actual_joint_torq_(1) = j2;
    this->right_actual_joint_torq_(2) = j3;
    this->right_actual_joint_torq_(3) = j4;
    this->right_actual_joint_torq_(4) = j5;
    this->right_actual_joint_torq_(5) = j6;
}

void ArmController::control_loop_()
{
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
    Eigen::Vector<double,ArmModel::num_dof_> feedforward_torque;

    auto increase_time_spec = [](struct timespec* time, const struct timespec* increasement)
    {
        time->tv_sec += increasement->tv_sec;
        time->tv_nsec += increasement->tv_nsec;

        if (time->tv_nsec >= 1000000000)
        {
            time->tv_sec++;
            time->tv_nsec -= 1000000000;
        }
    };

    struct timespec wakeup_time = {0,0};
    static const struct timespec cycletime = {0, 5000000};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while(this->running_)
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        std::lock_guard(this->left_actual_state_mtx_);
        this->model_->getTransform(link_transform, link_com_transform, this->left_actual_joint_pos_);
        this->model_->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, this->left_actual_joint_vel_);
        this->model_->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        this->model_->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = this->model_->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = this->model_->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, this->left_actual_joint_vel_);
        gravity_compensate = this->model_->getJointSpaceGravityCompensate(link_com_jacobian);

        feedforward_torque = generalized_mass_matrix * this->left_actual_joint_acc_ + centrifugal_coriolis_matrix * this->left_actual_joint_vel_ + gravity_compensate;

        std::lock_guard(this->left_target_state_mtx_);
        this->interface_->setLeftJointControl(this->left_target_joint_pos_, this->left_target_joint_vel_, feedforward_torque);

        std::lock_guard(this->right_actual_state_mtx_);
        this->model_->getTransform(link_transform, link_com_transform, this->right_actual_joint_pos_);
        this->model_->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, this->right_actual_joint_vel_);
        this->model_->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        this->model_->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = this->model_->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = this->model_->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, this->right_actual_joint_vel_);
        gravity_compensate = this->model_->getJointSpaceGravityCompensate(link_com_jacobian);

        feedforward_torque = generalized_mass_matrix * this->right_actual_joint_acc_ + centrifugal_coriolis_matrix * this->right_actual_joint_vel_ + gravity_compensate;

        std::lock_guard(this->right_target_state_mtx_);
        this->interface_->setRightJointControl(this->right_target_joint_pos_, this->right_target_joint_vel_, feedforward_torque);
    }
}
