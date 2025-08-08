#include <iostream>
#include <arm_model.hpp>

int main()
{
    Eigen::Vector3d left_hand_position(0.1075, 0.088, -0.6315);
    Eigen::Quaterniond left_hand_orientation(0.9953,-0.0076,-0.0878,0.0408);
    Eigen::Matrix4d left_hand_pose_in = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d left_hand_pose_out;
    left_hand_pose_in.block<3,1>(0,3) = left_hand_position;
    left_hand_pose_in.block<3,3>(0,0) = left_hand_orientation.toRotationMatrix();
    ArmModel left_arm(ArmType::LEFT_ARM);
    if (left_arm.getPredictedPose(left_hand_pose_out, left_hand_pose_in))
    {
        std::cout << "Left Arm Predicted Pose:\n" << left_hand_pose_out << std::endl;
    }
    else
    {
        std::cout << "Left Arm IK failed." << std::endl;
    }

    Eigen::Vector3d right_hand_position(0.0841,-0.1505,-0.6237);
    Eigen::Quaterniond right_hand_orientation(0.9914,-0.0214,-0.0947,-0.0878);
    Eigen::Matrix4d right_hand_pose_in = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d right_hand_pose_out;
    right_hand_pose_in.block<3,1>(0,3) = right_hand_position;
    right_hand_pose_in.block<3,3>(0,0) = right_hand_orientation.toRotationMatrix();
    ArmModel right_arm(ArmType::RIGHT_ARM);
    if (right_arm.getPredictedPose(right_hand_pose_out, right_hand_pose_in))
    {
        std::cout << "Right Arm Predicted Pose:\n" << right_hand_pose_out << std::endl;
    }
    else
    {
        std::cout << "Right Arm IK failed." << std::endl;
    }
    return 0;
}