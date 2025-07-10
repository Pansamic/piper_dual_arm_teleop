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

ArmController::ArmController(
    std::shared_ptr<ArmModel> left_arm_model,
    std::shared_ptr<ArmModel> right_arm_model,
    std::shared_ptr<ArmInterface> interface,
    TrajectoryBuffer<ArmPlanner::num_plan_waypoint_>& left_arm_trajectory_buffer,
    TrajectoryBuffer<ArmPlanner::num_plan_waypoint_>& right_arm_trajectory_buffer,
    const size_t freq_ctrl):
    left_arm_model_(left_arm_model), right_arm_model_(right_arm_model),
    interface_(interface), freq_ctrl_(freq_ctrl),
    left_arm_trajectory_buffer_(left_arm_trajectory_buffer),
    right_arm_trajectory_buffer_(right_arm_trajectory_buffer),
    control_thread_(&ArmController::threadControl, this), running_(true)
{

}

void ArmController::stop()
{
    this->running_.store(false, std::memory_order_release);
    if ( this->control_thread_.joinable() )
    {
        control_thread_.join();  // Ensure thread finishes before destruction
    }
}

void ArmController::threadControl()
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
    JointState target_joint_state;
    JointState actual_joint_state;
    ErrorCode err = OK;

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
    static struct timespec cycletime = {0, static_cast<long int>(1000000000L/this->freq_ctrl_)};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while( this->running_.load(std::memory_order_acquire) )
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        actual_joint_state.joint_pos = this->interface_->getLeftJointPosition();
        actual_joint_state.joint_vel = this->interface_->getLeftJointVelocity();

        this->left_arm_model_->getTransform(link_transform, link_com_transform, actual_joint_state.joint_pos);
        this->left_arm_model_->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, actual_joint_state.joint_vel);
        this->left_arm_model_->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        this->left_arm_model_->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = this->left_arm_model_->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = this->left_arm_model_->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, actual_joint_state.joint_vel);
        gravity_compensate = this->left_arm_model_->getJointSpaceGravityCompensate(link_com_jacobian);

        feedforward_torque = generalized_mass_matrix * actual_joint_state.joint_acc + centrifugal_coriolis_matrix * actual_joint_state.joint_vel + gravity_compensate;

        target_joint_state.joint_pos.setZero();
        target_joint_state.joint_vel.setZero();
        target_joint_state.joint_acc.setZero();
        target_joint_state.joint_torq.setZero();
        err = this->left_arm_trajectory_buffer_.interpolate(target_joint_state, std::chrono::steady_clock::now());
        switch ( err )
        {
        case NotImplemented:
            LOG_ERROR("Failed to interpolate: Method hasn't been implemented.");
            break;
        case InvalidData:
            LOG_WARN("Failed to interpolate: Not enough trajectory points in trajectory buffer.");
            break;
        case InvalidArgument:
            LOG_ERROR("Failed to interpolate: Method doesn't exist.");
            break;
        default:
            break;
        }
        this->interface_->setLeftJointControl(target_joint_state.joint_pos, target_joint_state.joint_vel, feedforward_torque);

        actual_joint_state.joint_pos = this->interface_->getRightJointPosition();
        actual_joint_state.joint_vel = this->interface_->getRightJointVelocity();

        this->right_arm_model_->getTransform(link_transform, link_com_transform, actual_joint_state.joint_pos);
        this->right_arm_model_->getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel, link_transform, link_com_transform, actual_joint_state.joint_vel);
        this->right_arm_model_->getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        this->right_arm_model_->getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform, link_lin_vel, link_ang_vel, link_com_lin_vel);
        generalized_mass_matrix = this->right_arm_model_->getJointSpaceMassMatrix(link_com_transform,link_com_jacobian);
        centrifugal_coriolis_matrix = this->right_arm_model_->getJointSpaceCoriolisMatrix(link_com_transform,link_com_jacobian,link_com_jacobian_dot, actual_joint_state.joint_vel);
        gravity_compensate = this->right_arm_model_->getJointSpaceGravityCompensate(link_com_jacobian);

        feedforward_torque = generalized_mass_matrix * actual_joint_state.joint_acc + centrifugal_coriolis_matrix * actual_joint_state.joint_vel + gravity_compensate;

        target_joint_state.joint_pos.setZero();
        target_joint_state.joint_vel.setZero();
        target_joint_state.joint_acc.setZero();
        target_joint_state.joint_torq.setZero();
        err = this->right_arm_trajectory_buffer_.interpolate(target_joint_state, std::chrono::steady_clock::now());
        switch ( err )
        {
        case NotImplemented:
            LOG_ERROR("Failed to interpolate: Method hasn't been implemented.");
            break;
        case InvalidData:
            LOG_WARN("Failed to interpolate: Not enough trajectory points in trajectory buffer.");
            break;
        case InvalidArgument:
            LOG_ERROR("Failed to interpolate: Method doesn't exist.");
            break;
        default:
            break;
        }
        this->interface_->setRightJointControl(target_joint_state.joint_pos, target_joint_state.joint_vel, feedforward_torque);
    }
}
