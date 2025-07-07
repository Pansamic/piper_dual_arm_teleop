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
#include <teleop_task_runner.h>

const Eigen::Vector3d TeleopTaskRunner::head_position_ = Eigen::Vector3d(0.325024,0,0.80246);
const Eigen::Quaterniond TeleopTaskRunner::left_hand_orientation_offset_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
const Eigen::Quaterniond TeleopTaskRunner::right_hand_orientation_offset_(
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())));
const Eigen::Matrix4d TeleopTaskRunner::left_arm_base_transform_ = (Eigen::Matrix4d() << 
    0, -0.707106781 ,  0.707106781 , 0.23805,
    0,  0.707106781 ,  0.707106781 , 0.19675,
    -1, 0           ,  0           , 0.74065,
    0,  0           ,  0           , 1).finished();
const Eigen::Matrix4d TeleopTaskRunner::right_arm_base_transform_ = (Eigen::Matrix4d() <<
    0,  0.707106781, 0.707106781, 0.23805,
    0,  0.707106781,-0.707106781,-0.19675,
    -1, 0,           0,           0.74065,
    0,  0,           0,           1).finished();

// left hand home position: 0.542092536439244, 0.500792536439244, 0.398868333963670
// left hand home orientation: 0.000044497177102,0.382683431921232,-0.923879531439719,0.000018431334243
// right hand home position: 0.542092536439244, -0.500792536439244, 0.398868333963670
// right hand home orientation: 0.000044497177102, -0.382683431921232, -0.923879531439719, -0.000018431334243

TeleopTaskRunner::TeleopTaskRunner(std::shared_ptr<ArmInterface> interface, size_t freq_plan, size_t freq_ctrl):
    freq_plan_(freq_plan_), freq_ctrl_(freq_ctrl_),
    interface_(interface),
    left_hand_target_pos_(Eigen::Vector3d::Zero()),
    left_hand_target_orientation_(Eigen::Quaterniond::Identity()),
    left_hand_target_pose_(Eigen::Matrix4d::Identity()),
    left_hand_actual_orientation_(Eigen::Matrix4d::Identity()),
    left_gripper_control_(0),
    right_hand_target_pos_(Eigen::Vector3d::Zero()),
    right_hand_target_orientation_(Eigen::Quaterniond::Identity()),
    right_hand_target_pose_(Eigen::Matrix4d::Identity()),
    right_hand_actual_orientation_(Eigen::Quaterniond::Identity()),
    right_gripper_control_(0),
    left_arm_trajectory_buffer_(freq_ctrl/freq_plan*this->traj_buf_size_),
    right_arm_trajectory_buffer_(freq_ctrl/freq_plan*this->traj_buf_size_),
    left_arm_target_joint_state_buffer_(32),
    right_arm_target_joint_state_buffer_(32),
    left_arm_target_joint_pos_history_(32),
    right_arm_target_joint_pos_history_(32),
    channel_(this->io_context_, "192.168.1.105", 54321, "192.168.1.5", 12345),
    send_mq_(RingBuffer<whole_body_msg>{10}),
    recv_mq_(RingBuffer<whole_body_msg>{10})
{
    this->channel_.bind_message_queue("whole_body_sender", ParserType::Sender, this->send_mq_);
    this->channel_.bind_message_queue("whole_body_receiver", ParserType::Receiver, this->recv_mq_);
    this->channel_.enable_sender();
    this->channel_.enable_receiver();

    std::thread t([&]() { this->io_context_.run(); });

    this->left_arm_model_ = std::make_shared<ArmModel>(left_arm_base_transform_);
    this->right_arm_model_ = std::make_shared<ArmModel>(right_arm_base_transform_);

    this->planner_ = std::make_unique<ArmPlanner>(
        this->left_arm_target_joint_state_buffer_,
        this->right_arm_target_joint_state_buffer_,
        this->left_arm_trajectory_buffer_,
        this->right_arm_trajectory_buffer_,
        this->freq_plan_, this->freq_ctrl_);

    this->controller_ = std::make_unique<ArmController>(
        this->left_arm_model_,
        this->right_arm_model_,
        interface,
        this->left_arm_trajectory_buffer_,
        this->right_arm_trajectory_buffer_,
        this->freq_ctrl_);
}

TeleopTaskRunner::~TeleopTaskRunner()
{
    this->running_ = false;
}

void TeleopTaskRunner::run()
{
    whole_body_msg msg;

    while ( this->running_ )
    {
        if ( !this->recv_mq_.dequeue(msg) )
        {
            /* Continue if message not received. */
            continue;
        }
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
            this->left_hand_target_orientation_.w() = msg.left_hand_quat[0];
            this->left_hand_target_orientation_.x() = msg.left_hand_quat[1];
            this->left_hand_target_orientation_.y() = msg.left_hand_quat[2];
            this->left_hand_target_orientation_.z() = msg.left_hand_quat[3];
            /* Construct transformation matrix from orientation and position */
            this->left_hand_target_pose_.block<3,3>(0,0) = this->left_hand_target_orientation_.toRotationMatrix();
            this->left_hand_target_pose_.block<3,1>(0,3) = this->left_hand_target_pos_;
            this->left_hand_target_pose_(3,3) = 1;

            this->scaleLeftHandPose(this->left_hand_target_pose_);

            /* Get least damped inverse kinematics */
            Eigen::Vector<double,ArmModel::num_dof_> joint_pos = this->left_arm_model_->getDampedLeastSquareInverseKinematics(
                0.1, Eigen::Vector<double,6>(0.05,0.05,0.05,0.1,0.1,0.1), 200, this->left_hand_target_pose_, this->interface_->getLeftJointPosition());
            
            this->checkInvalidJointPosition(joint_pos);

            this->left_arm_target_joint_pos_history_.push(joint_pos);
        }
        /* Check for right arm control enable signal */
        if ( msg.mask&(1<<12) )
        {
            /* Assign position to Eigen::Vector3d */
            this->right_hand_target_pos_(0) = msg.right_hand_pos[0];
            this->right_hand_target_pos_(1) = msg.right_hand_pos[1];
            this->right_hand_target_pos_(2) = msg.right_hand_pos[2];
            /* Assign quaternion to Eigen::Quaterniond */
            this->right_hand_target_orientation_.w() = msg.right_hand_quat[0];
            this->right_hand_target_orientation_.x() = msg.right_hand_quat[1];
            this->right_hand_target_orientation_.y() = msg.right_hand_quat[2];
            this->right_hand_target_orientation_.z() = msg.right_hand_quat[3];
            /* Construct transformation matrix from orientation and position */
            this->right_hand_target_pose_.block<3,3>(0,0) = this->right_hand_target_orientation_.toRotationMatrix();
            this->right_hand_target_pose_.block<3,1>(0,3) = this->right_hand_target_pos_;
            this->right_hand_target_pose_(3,3) = 1;
        }
    }
}

void TeleopTaskRunner::scaleLeftHandPose(Eigen::Matrix4d& pose)
{
    static const Eigen::Matrix3d left_hand_orientation_offset_rotm = this->left_hand_orientation_offset_.toRotationMatrix();
    pose.block<3,1>(0,3) += this->head_position_;
    pose.block<3,3>(0,0) *= left_hand_orientation_offset_rotm;
}

void TeleopTaskRunner::scaleRightHandPose(Eigen::Matrix4d& pose)
{
    static const Eigen::Matrix3d right_hand_orientation_offset_rotm = this->right_hand_orientation_offset_.toRotationMatrix();
    pose.block<3,1>(0,3) += this->head_position_;
    pose.block<3,3>(0,0) *= right_hand_orientation_offset_rotm;
}

bool TeleopTaskRunner::checkInvalidJointPosition(const Eigen::Vector<double,ArmModel::num_dof_>& joint_pos)
{
    for ( auto it=this->left_arm_target_joint_pos_history_.crbegin() ; it!=this->left_arm_target_joint_pos_history_.crend() ; ++it )
    {
        
    }
}
