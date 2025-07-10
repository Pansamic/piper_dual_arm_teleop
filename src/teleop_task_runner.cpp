/**
 * @file teleop_task_runner.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Task runner for dual arm teleoperation.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <termination.h>
#include <teleop_task_runner.h>

const Eigen::Vector3d TeleopTaskRunner::head_position_ = Eigen::Vector3d(0.325024,0,0.80246);
const Eigen::Quaterniond TeleopTaskRunner::left_hand_orientation_offset_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
const Eigen::Quaterniond TeleopTaskRunner::right_hand_orientation_offset_(
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));
const Eigen::Matrix4d TeleopTaskRunner::left_arm_base_transform_ = (Eigen::Matrix4d() << 
    0,  0,  1, 0.23805,
   -1,  0,  0, 0.19675,
    0, -1,  0, 0.74065,
    0,  0,  0, 1).finished();
const Eigen::Matrix4d TeleopTaskRunner::right_arm_base_transform_ = (Eigen::Matrix4d() <<
    0, 0, 1, 0.23805,
    1, 0, 0,-0.19675,
    0, 1, 0, 0.74065,
    0, 0, 0, 1).finished();

TeleopTaskRunner::TeleopTaskRunner(std::shared_ptr<ArmInterface> interface, size_t freq_plan, size_t freq_ctrl):
    freq_plan_(freq_plan), freq_ctrl_(freq_ctrl),
    interface_(interface),
    left_hand_target_pos_(Eigen::Vector3d::Zero()),
    left_hand_target_orientation_(Eigen::Quaterniond::Identity()),
    left_hand_target_pose_(Eigen::Matrix4d::Identity()),
    left_hand_actual_orientation_(Eigen::Quaterniond::Identity()),
    left_gripper_control_(0),
    right_hand_target_pos_(Eigen::Vector3d::Zero()),
    right_hand_target_orientation_(Eigen::Quaterniond::Identity()),
    right_hand_target_pose_(Eigen::Matrix4d::Identity()),
    right_hand_actual_orientation_(Eigen::Quaterniond::Identity()),
    right_gripper_control_(0),
    left_arm_joint_state_history_(32),
    right_arm_joint_state_history_(32),
    left_arm_trajectory_buffer_(),
    right_arm_trajectory_buffer_(),
    channel_(this->io_context_, "192.168.1.108", 54321, "192.168.1.105", 12345),
    send_mq_(RingBuffer<whole_body_msg>{32}),
    recv_mq_(RingBuffer<whole_body_msg>{32})
{
}

void TeleopTaskRunner::stop()
{
    // this->running_.store(false, std::memory_order_release);
    this->running_ = false;
    this->io_context_.stop();
    this->io_context_thread_.join();
    this->planner_->stop();
    this->controller_->stop();
}

void TeleopTaskRunner::run()
{
    whole_body_msg msg;
    ErrorCode err;

    this->channel_.bind_message_queue("whole_body_sender", ParserType::Sender, this->send_mq_);
    this->channel_.bind_message_queue("whole_body_receiver", ParserType::Receiver, this->recv_mq_);
    this->channel_.enable_sender();
    this->channel_.enable_receiver();

    this->io_context_thread_ = std::thread([&]() { this->io_context_.run(); });

    this->left_arm_model_ = std::make_shared<ArmModel>(left_arm_base_transform_);
    this->right_arm_model_ = std::make_shared<ArmModel>(right_arm_base_transform_);

    this->planner_ = std::make_unique<ArmPlanner>(
        this->left_arm_trajectory_buffer_,
        this->right_arm_trajectory_buffer_,
        this->freq_plan_);

    this->controller_ = std::make_unique<ArmController>(
        this->left_arm_model_,
        this->right_arm_model_,
        this->interface_,
        this->left_arm_trajectory_buffer_,
        this->right_arm_trajectory_buffer_,
        this->freq_ctrl_);

    this->running_ = true;

    while ( this->running_ )
    {
        if ( TerminationHandler::stop_requested.load() )
        {
            return;
        }
        if ( this->recv_mq_.empty() )
        {
            continue;
        }
        this->recv_mq_.dequeue(msg);

        /* Check for system control enable signal */
        if ( !(msg.mask&(1<<15)) )
        {
            continue;
        }
        /* Check for left arm control enable signal */
        if ( msg.mask&(1<<13) )
        {
            /* Assign position to Eigen::Vector3d */
            this->left_hand_target_pos_(0) = msg.left_hand_pos[0];
            this->left_hand_target_pos_(1) = msg.left_hand_pos[1];
            this->left_hand_target_pos_(2) = msg.left_hand_pos[2];
            /* Assign quaternion to Eigen::Quaterniond */
            this->left_hand_target_orientation_.x() = msg.left_hand_quat[0];
            this->left_hand_target_orientation_.y() = msg.left_hand_quat[1];
            this->left_hand_target_orientation_.z() = msg.left_hand_quat[2];
            this->left_hand_target_orientation_.w() = msg.left_hand_quat[3];

            LOG_DEBUG("Original left hand target position:x={:.4f},y={:.4f},z={:.4f}.Orientation:x={:.4f},y={:.4f},z={:.4f},w={:.4f}.",
                this->left_hand_target_pos_(0), this->left_hand_target_pos_(1), this->left_hand_target_pos_(2),
                this->left_hand_target_orientation_.x(), this->left_hand_target_orientation_.y(),
                this->left_hand_target_orientation_.z(), this->left_hand_target_orientation_.w());

            this->scaleLeftHandPose(this->left_hand_target_pos_, this->left_hand_target_orientation_);

            /* Construct transformation matrix from orientation and position */
            this->left_hand_target_pose_.block<3,3>(0,0) = this->left_hand_target_orientation_.toRotationMatrix();
            this->left_hand_target_pose_.block<3,1>(0,3) = this->left_hand_target_pos_;
            this->left_hand_target_pose_(3,3) = 1;

            LOG_DEBUG("Scaled left hand target position:x={:.4f},y={:.4f},z={:.4f}.Orientation:x={:.4f},y={:.4f},z={:.4f},w={:.4f}.",
                this->left_hand_target_pos_(0), this->left_hand_target_pos_(1), this->left_hand_target_pos_(2),
                this->left_hand_target_orientation_.x(), this->left_hand_target_orientation_.y(),
                this->left_hand_target_orientation_.z(), this->left_hand_target_orientation_.w());

            this->interface_->setLeftGripperControl(0.07 - 0.07 * msg.left_grip, 0);
            this->interface_->setRightGripperControl(0.07 - 0.07 * msg.right_grip, 0);
            /* Get least damped inverse kinematics */
            err = this->left_arm_model_->getDampedLeastSquareInverseKinematics(this->left_arm_target_joint_state_.joint_pos,
                0.1, Eigen::Vector<double,6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, this->left_hand_target_pose_, this->interface_->getLeftJointPosition());
            if ( err == NoResult )
            {
                LOG_WARN("No left hand inverse kinematics result.");
                continue;
            }

            if ( this->checkInvalidJointPosition(this->left_arm_target_joint_state_.joint_pos) )
            {
                this->left_arm_target_joint_state_.joint_torq.setZero();
                err = this->computeJointVelocityAndAcceleration(
                    this->left_arm_joint_state_history_,
                    this->left_arm_target_joint_state_.joint_pos,
                    this->left_arm_target_joint_state_.joint_vel,
                    this->left_arm_target_joint_state_.joint_acc);
                if ( err == InvalidData )
                {
                    LOG_WARN("Failed to compute joint vel and acc: history buffer size={:d}", this->left_arm_joint_state_history_.size());
                    this->planner_->setLeftArmTargetJointState(this->left_arm_target_joint_state_);
                    this->left_arm_joint_state_history_.push(this->left_arm_target_joint_state_);
                    continue;
                }
                LOG_INFO("Left arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                    this->left_arm_target_joint_state_.joint_pos(0),this->left_arm_target_joint_state_.joint_pos(1),this->left_arm_target_joint_state_.joint_pos(2),
                    this->left_arm_target_joint_state_.joint_pos(3),this->left_arm_target_joint_state_.joint_pos(4),this->left_arm_target_joint_state_.joint_pos(5));
                this->planner_->setLeftArmTargetJointState(this->left_arm_target_joint_state_);
                this->left_arm_joint_state_history_.push(this->left_arm_target_joint_state_);
            }
        }
        /* Check for right arm control enable signal */
        if ( msg.mask&(1<<12) )
        {
            /* Assign position to Eigen::Vector3d */
            this->right_hand_target_pos_(0) = msg.right_hand_pos[0];
            this->right_hand_target_pos_(1) = msg.right_hand_pos[1];
            this->right_hand_target_pos_(2) = msg.right_hand_pos[2];
            /* Assign quaternion to Eigen::Quaterniond */
            this->right_hand_target_orientation_.x() = msg.right_hand_quat[0];
            this->right_hand_target_orientation_.y() = msg.right_hand_quat[1];
            this->right_hand_target_orientation_.z() = msg.right_hand_quat[2];
            this->right_hand_target_orientation_.w() = msg.right_hand_quat[3];

            LOG_DEBUG("Original right hand target position:x={:.4f},y={:.4f},z={:.4f}.Orientation:x={:.4f},y={:.4f},z={:.4f},w={:.4f}.",
                this->right_hand_target_pos_(0), this->right_hand_target_pos_(1), this->right_hand_target_pos_(2),
                this->right_hand_target_orientation_.x(), this->right_hand_target_orientation_.y(),
                this->right_hand_target_orientation_.z(), this->right_hand_target_orientation_.w());

            this->scaleRightHandPose(this->right_hand_target_pos_, this->right_hand_target_orientation_);

            /* Construct transformation matrix from orientation and position */
            this->right_hand_target_pose_.block<3,3>(0,0) = this->right_hand_target_orientation_.toRotationMatrix();
            this->right_hand_target_pose_.block<3,1>(0,3) = this->right_hand_target_pos_;
            this->right_hand_target_pose_(3,3) = 1;

            LOG_DEBUG("Scaled right hand target position:x={:.4f},y={:.4f},z={:.4f}.Orientation:x={:.4f},y={:.4f},z={:.4f},w={:.4f}.",
                this->right_hand_target_pos_(0), this->right_hand_target_pos_(1), this->right_hand_target_pos_(2),
                this->right_hand_target_orientation_.x(), this->right_hand_target_orientation_.y(),
                this->right_hand_target_orientation_.z(), this->right_hand_target_orientation_.w());

            /* Get least damped inverse kinematics */
            err = this->right_arm_model_->getDampedLeastSquareInverseKinematics(this->right_arm_target_joint_state_.joint_pos,
                0.1, Eigen::Vector<double,6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, this->right_hand_target_pose_, this->interface_->getRightJointPosition());
            if ( err == NoResult )
            {
                LOG_WARN("No right hand inverse kinematics result.");
                continue;
            }

            if ( this->checkInvalidJointPosition(this->right_arm_target_joint_state_.joint_pos) )
            {
                this->right_arm_target_joint_state_.joint_torq.setZero();
                err = this->computeJointVelocityAndAcceleration(
                    this->right_arm_joint_state_history_,
                    this->right_arm_target_joint_state_.joint_pos,
                    this->right_arm_target_joint_state_.joint_vel,
                    this->right_arm_target_joint_state_.joint_acc);
                if ( err == InvalidData )
                {
                    LOG_WARN("Failed to compute joint vel and acc: history buffer size={:d}", this->right_arm_joint_state_history_.size());
                    this->planner_->setRightArmTargetJointState(this->right_arm_target_joint_state_);
                    this->right_arm_joint_state_history_.push(this->right_arm_target_joint_state_);
                    continue;
                }
                LOG_INFO("Right arm target joint position:{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                    this->right_arm_target_joint_state_.joint_pos(0),this->right_arm_target_joint_state_.joint_pos(1),this->right_arm_target_joint_state_.joint_pos(2),
                    this->right_arm_target_joint_state_.joint_pos(3),this->right_arm_target_joint_state_.joint_pos(4),this->right_arm_target_joint_state_.joint_pos(5));
                this->planner_->setRightArmTargetJointState(this->right_arm_target_joint_state_);
                this->right_arm_joint_state_history_.push(this->right_arm_target_joint_state_);
            }
        }
    }
}

void TeleopTaskRunner::scaleLeftHandPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    /* Scale the end effector position. */
    position *= 1.0;
    /* Add position offset because VR remote controller position is based on headset,
     * so the base position should be converted to the arm model base. */
    position += this->head_position_;
    // orientation = orientation * this->left_hand_orientation_offset_;
}

void TeleopTaskRunner::scaleRightHandPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    /* Scale the end effector position. */
    position *= 1.0;
    /* Add position offset because VR remote controller position is based on headset,
     * so the base position should be converted to the arm model base. */
    position += this->head_position_;
    // orientation = orientation * this->right_hand_orientation_offset_;
}

bool TeleopTaskRunner::checkInvalidJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    // for ( auto it=this->left_arm_joint_state_history_.crbegin() ; it!=this->left_arm_joint_state_history_.crend() ; ++it )
    // {
        
    // }
    return true;
}

ErrorCode TeleopTaskRunner::computeJointVelocityAndAcceleration(
    RingBuffer<JointState>& history,
    const Eigen::Vector<double, ArmModel::num_dof_>& joint_pos,
    Eigen::Vector<double, ArmModel::num_dof_>& joint_vel,
    Eigen::Vector<double, ArmModel::num_dof_>& joint_acc)
{
    constexpr int window_size = 5;
    constexpr int poly_deg = 2;
    constexpr int dof = ArmModel::num_dof_;
    static_assert(window_size >= poly_deg + 1, "Insufficient data points");

    ErrorCode err = OK;

    std::array<Eigen::Vector<double, dof>, window_size> pos_buffer;

    if (history.size() < window_size)
    {
        /* Insufficient history for velocity/acceleration estimation */
        err = InvalidData;
        return err;
    }

    constexpr double dt = 1.0 / 72.0;
    Eigen::Matrix<double, window_size, poly_deg + 1> A;
    Eigen::Matrix<double, window_size, dof> V;

    for (int i = 0; i < window_size; ++i)
    {
        double t = -dt * (window_size - 1 - i);
        A(i, 0) = 1.0;
        A(i, 1) = t;
        A(i, 2) = t * t;
        V.row(i) = pos_buffer[i];
    }

    // Fit polynomial coefficients: coeffs = (AᵗA)⁻¹ AᵗV
    Eigen::Matrix<double, poly_deg + 1, dof> coeffs = A.colPivHouseholderQr().solve(V);

    joint_vel = coeffs.row(1);          // First derivative at t = 0
    joint_acc = 2.0 * coeffs.row(2);    // Second derivative at t = 0
    return err;
}
