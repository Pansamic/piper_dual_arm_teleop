/**
 * @file rigidbodytree.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Rigid body tree structure
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __RIGIDBODYTREE_HPP__
#define __RIGIDBODYTREE_HPP__

#include <string>
#include <array>
#include <Eigen/Core>

template<typename T, std::size_t Size>
class RigidBodyTree
{
public:
    enum JointType
    {
        JointTypeRevolute = 1,
        JointTypePrismatic,
        JointTypeFixed,
    };
    struct Joint
    {
        std::string name;
        std::size_t id;
        JointType type;
        Eigen::Vector<T, 3> joint_axis;
        Eigen::Vector<T, 2> limits;
        Eigen::Matrix<T, 4, 4> joint_to_parent_transform;
    };
    struct RigidBody
    {
        std::string name;
        std::size_t id;
        T mass;
        Eigen::Vector<T, 3> center_of_mass;
        Eigen::Matrix<T, 3, 3> intertia;
        Joint joint;
        struct RigidBody* parent;
        std::array<struct RigidBody*, 2> children;
    };
    RigidBodyTree() = delete;
    RigidBodyTree(RigidBody* root):root_(root){}
    ~RigidBodyTree() = default;
    std::size_t getTreeSize()
    {
        auto get_tree_size = [](RigidBody* root)
        {
            if ( root != nullptr )
            {
                return get_tree_size(root->children[0]) + get_tree_size(root->children[1]);
            }
            else
            {
                return 0;
            }
        }
        return get_tree_size(this->root);
    }
    /**
     * @brief Calculate transformation matrix of link named <name> in base link frame.
     * 
     * @param link_transform output link transform matrix of link named <name> in base frame.
     * @param name name of the link.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     * 
     * @note This method is slower than index search version of `getLinkTransform` because string
     * matching is more time-consuming than numerical matching.
     */
    void getLinkTransform(
        Eigen::Matrix<T, 4, 4>& link_transform,
        const std::string name,
        const Eigen::Vector<T, Size>& joint_pos) const
    {
        
    }
    /**
     * @brief Calculate transformation matrix of link named <name> in base link frame.
     * 
     * @param link_transform output link transform matrix of link named <name> in base frame.
     * @param id index of the link.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getLinkTransform(
        Eigen::Matrix<T, 4, 4>& link_transform,
        const std::size_t id,
        const Eigen::Vector<T, Size>& joint_pos) const
    {
        link_transform = Eigen::Matrix<T, 4, 4>::Identity();

        std::vector<const RigidBody*> path;
        const RigidBody* node = root_;
        std::function<const RigidBody*(const RigidBody*, std::size_t)> dfs = [&](const RigidBody* curr, std::size_t target_id) -> const RigidBody*
        {
            if (!curr) return nullptr;
            path.push_back(curr);
            if (curr->id == target_id)
                return curr;
            for (const RigidBody* child : curr->children)
            {
                if (child)
                {
                    const RigidBody* found = dfs(child, target_id);
                    if (found) return found;
                }
            }
            path.pop_back();
            return nullptr;
        };

        if (!dfs(node, id))
            return; // Link not found

        for (const RigidBody* rb : path)
        {
            const Joint& joint = rb->joint;
            Eigen::Matrix<T, 4, 4> joint_motion = Eigen::Matrix<T, 4, 4>::Identity();

            if (joint.type == JointTypeRevolute)
            {
                Eigen::AngleAxis<T> aa(joint_pos[joint.id], joint.joint_axis.normalized());
                joint_motion.template block<3, 3>(0, 0) = aa.toRotationMatrix();
            }
            else if (joint.type == JointTypePrismatic)
            {
                joint_motion.template block<3, 1>(0, 3) = joint.joint_axis.normalized() * joint_pos[joint.id];
            }
            // JointTypeFixed has identity motion

            link_transform = link_transform * joint.joint_to_parent_transform * joint_motion;
        }
    }
    /**
     * @brief Calculate transform matricies of all the links in rigid body tree.
     * 
     * @param link_transform output. array of link transform matricies.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getAllLinkTransform(
        std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform,
        const Eigen::Vector<T, Size>& joint_pos) const
    {

    }
    /**
     * @brief Calculate transformation matrix of center of mass of link named <name> in base link frame.
     * 
     * @param link_com_transform output. transform matrix of center of mass of link named <name>.
     * @param name name of link.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getLinkComTransform(
        Eigen::Matrix<T, 4, 4>& link_com_transform,
        const std::string name,
        const Eigen::Vector<T, Size>& joint_pos)
    {

    }
    /**
     * @brief Calculate transformation matrix of center of mass of link with index `id` in base link frame.
     * 
     * @param link_com_transform output. transform matrix of center of mass of link named <name>.
     * @param id index of link.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getLinkComTransform(
        std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const std::size_t id,
        const Eigen::Vector<T, Size>& joint_pos)
    {

    }
    /**
     * @brief Calculate transformation matrix of all the center of mass of links in rigid body tree in base link frame.
     * 
     * @param link_com_transform output. array of link transform matricies.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getAllLinkComTransform(
        std::array<Eigen::Matrix<T, 4, 4>,Size>& link_com_transform,
        const Eigen::Vector<T,Size>& joint_pos)
    {

    }
    /**
     * @brief Calculate transform matricies of link frame and center of mass frame of all the links in rigid body tree.
     * 
     * @param link_transform output. array of link transform matricies.
     * @param link_com_transform output. array of link center of mass transform matricies.
     * @param joint_pos all joint position in order of `id` of rigid body tree, including fixed joint.
     */
    void getAllLinkAndComTransform(
        std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform,
        std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const Eigen::Vector<T, Size>& joint_pos)
    {

    }
    /**
     * @brief Calculate the velocity of each link and center of mass.
     * 
     * @param link_lin_vel output. link linear velocity.
     * @param link_ang_vel output. link angular velocity.
     * @param link_com_lin_vel output. link center of mass linear velocity.
     * @param link_com_ang_vel output. link center of mass angular velocity.
     * @param link_transform transform matricies of link frame.
     * @param link_com_transform transform matricies of center of mass of links.
     * @param joint_vel joint velocities.
     */
    void getLinkVelocity(
        std::array<Eigen::Vector<T, 3>, Size>& link_lin_vel,
        std::array<Eigen::Vector<T, 3>, Size>& link_ang_vel,
        std::array<Eigen::Vector<T, 3>, Size>& link_com_lin_vel,
        std::array<Eigen::Vector<T, 3>, Size>& link_com_ang_vel,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const Eigen::Vector<T, Size>& joint_vel) const;
    
    void getLinkSpaceJacobian(
        std::array<Eigen::Matrix<T, 6, Size>, Size>& link_jacobian,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform) const;

    void getLinkComSpaceJacobian(
        std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform) const;

    void getLinkComSpaceJacobianDot(
        std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian_dot,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const std::array<Eigen::Vector<T, 3>, Size>& link_lin_vel,
        const std::array<Eigen::Vector<T, 3>, Size>& link_ang_vel,
        const std::array<Eigen::Vector<T, 3>, Size>& link_com_lin_vel) const;

    Eigen::Matrix<T,Size,Size> getJointSpaceMassMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian) const;

    Eigen::Matrix<T,Size,Size> getJointSpaceCoriolisMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, Size>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian,
        const std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian_dot,
        const Eigen::Vector<T,Size>& joint_vel) const;

    Eigen::Vector<T,Size> getJointSpaceGravityCompensate(
        const std::array<Eigen::Matrix<T, 6, Size>, Size>& link_com_jacobian) const;

    /**
     * @brief Get task space inverse dynamics matricies.
     * 
     * @param task_space_mass_matrix 
     * @param task_space_bias_force 
     * @param joint_pos 
     * @param twist 
     * 
     * @note From "Modern Robotics - Mechanics, Planning and Control" eq8.90 and eq8.91
     */
    void getTaskSpaceInverseDynamics(
        Eigen::Matrix<T,6,6>& task_space_mass_matrix,
        Eigen::Vector<T,6>& task_space_bias_force,
        const Eigen::Vector<T,Size>& joint_pos,
        const Eigen::Vector<T,Size>& joint_vel) const;
private:
    RigidBody* root_;
};

#endif // __RIGIDBODYTREE_HPP__