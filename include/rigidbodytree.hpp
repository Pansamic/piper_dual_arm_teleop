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
#include <vector>
#include <functional>
#include <algorithm>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD> // For pseudoInverse
#include <multi_node_tree.hpp>

enum JointType
{
    JointTypeRevolute = 1,
    JointTypePrismatic,
    JointTypeFixed,
};

template<typename T>
struct Joint
{
    /* ID starts from 1 */
    std::size_t id;
    JointType type;
    /* Axis in local frame of current link */
    Eigen::Vector<T, 3> axis;
    /* Limits in radians for revolute joints, meters for prismatic joints. 
     * For fixed joints, limits are ignored. Format: [min, max]*/
    Eigen::Vector<T, 2> limits;
    Eigen::Matrix<T, 4, 4> joint_to_parent_transform;
};

template<typename T>
struct RigidBody
{
    /* ID starts from 1 */
    std::size_t id;
    T mass;
    /* Postiion of center of mass in body frame (CoM) */
    Eigen::Vector<T, 3> center_of_mass;
    /* Inertia tensor in body frame (CoM) */
    Eigen::Matrix<T, 3, 3> inertia; // 
    Joint<T> joint;
};

template<typename T, std::size_t NumDof>
class RigidBodyTree : private MultiNodeTree<RigidBody<T>*>
{
public:
    /**
     * @brief A struct to store the data of the rigid body tree.
     * This struct is used to initialize the rigid body tree. 
     * 
     * @note These parameters corresponds to URDF parameters. 
     * So it's convenient to copy the data in URDF to this structure.
     */
    struct Builder
    {
        /* Index of root node, starting from 0. */
        std::size_t root_id;
        /* Base link position offset. */
        std::array<T, 3> base_position;
        /* Base link orientation offset. */
        std::array<T, 3> base_orientation;
        /* Mass of each link. Unit: kg */
        std::array<T, NumDof> mass;
        /* Position relative to parent joint of each joint. Unit: m */
        std::array<std::array<T, 3>, NumDof> initial_position;
        /* euler angles (RPY ZYX order) relative to parent joint of each joint expressed as a quaternion. Unit: radian */
        std::array<std::array<T, 3>, NumDof> initial_orientation;
        /* Center of mass of each link in local frame. Unit: m */
        std::array<std::array<T, 3>, NumDof> com;
        /* Inertia tensor of each link in local frame. Order: [Ixx, Ixy, Ixz, Iyy, Iyz, Izz] */
        std::array<std::array<T, 6>, NumDof> inertia;
        /* Joint type of each joint. */
        std::array<JointType, NumDof> joint_type;
        /* Axis of rotation for revolute joints. Unit: radian */
        std::array<std::array<T, 3>, NumDof> joint_axis;
        /* Limits of each joint. Unit: radian */
        std::array<std::array<T, 2>, NumDof> joint_limits;
        /* Parent index of each link, starting from 0. */
        // std::array<std::size_t, NumDof> parent_index;
        /* Children indices of each link, starting from 0. */
        std::array<std::vector<std::size_t>, NumDof> children_indices;

        RigidBody<T> buildRigidBodyById(std::size_t id)
        {
            RigidBody<T> link;
            link.id = id;
            link.mass = mass[id];
            link.center_of_mass = Eigen::Map<Eigen::Vector<T, 3>>(com[id].data());
            link.inertia = this->getInertiaMatrix(
                inertia[id][0],
                inertia[id][1],
                inertia[id][2],
                inertia[id][3],
                inertia[id][4],
                inertia[id][5]
            );
            link.joint.id = id;
            link.joint.type = joint_type[id];
            link.joint.axis = Eigen::Map<Eigen::Vector<T, 3>>(joint_axis[id].data());
            link.joint.limits = Eigen::Map<Eigen::Vector<T, 2>>(joint_limits[id].data());
            link.joint.joint_to_parent_transform = this->getTransformFromRotationAndTranslation(
                initial_position[id][0], initial_position[id][1], initial_position[id][2],
                initial_orientation[id][0], initial_orientation[id][1], initial_orientation[id][2]
            );
            return link;
        }
    };

    explicit RigidBodyTree() = default;
    ~RigidBodyTree() = default;

    bool build(const Builder& builder)
    {
        // Check if root_id is valid
        if (builder.root_id >= NumDof)
        {
            return false;
        }

        /* Build all the links */
        for ( int i=0 ; i<NumDof ; i++ )
        {
            this->links_storage_[i] = builder.buildRigidBodyById(i);
        }

        /* Compute base transform offset. */
        Eigen::Matrix<T, 4, 4> base_transform = this->getTransformFromRotationAndTranslation(
            builder.base_position[0], builder.base_position[1], builder.base_position[2],
            builder.base_orientation[0], builder.base_orientation[1], builder.base_orientation[2]);

        /* Apply base transform to first link. */
        const auto& root_joint_transform = this->links_storage_[builder.root_id].joint.joint_to_parent_transform;
        this->links_storage_[builder.root_id].joint.joint_to_parent_transform = base_transform * root_joint_transform;

        /* Set root node. */
        this->setRoot(&this->links_storage_[builder.root_id]);

        auto root_node = this->getRoot();
        if (!root_node)
        {
            return false; // Failed to set root
        }

        // Use a queue for BFS traversal to build the tree level by level
        std::queue<std::shared_ptr<TreeNode<RigidBody<T>*>>> node_queue;
        node_queue.push(root_node);

        while (!node_queue.empty())
        {
            auto current_tree_node = node_queue.front();
            node_queue.pop();

            // Get the ID of the body represented by the current tree node
            std::size_t current_id = current_tree_node->data.id; // IDs are 0-based

            // Check if the current index is valid (should always be true if tree is built correctly)
            if (current_id >= NumDof)
            {
                this->clear();
                return false;
            }

            // Iterate through all children indices of the current body
            const std::vector<std::size_t>& children_ids = builder.children_indices[current_id];

            for (std::size_t child_id : children_ids)
            {
                // Validate child_id (1-based)
                if (child_id == 0 || child_id > NumDof)
                {
                    this->clear();
                    return false; // Invalid child ID
                }

                // Add the child to the current node in the tree
                // addChild takes the data (T) and returns a shared_ptr to the newly created child TreeNode
                std::shared_ptr<TreeNode<RigidBody<T>*>> new_child_node = current_tree_node->addChild(&this->links_storage_[child_id]);

                // Add the new child node to the queue for processing its children later
                node_queue.push(new_child_node);
            }
        }

        return true;
    }

    /**
     * @brief Calculate transformation matrix of link with index `id` in base link frame.
     *
     * @param link_transform output link transform matrix of link with index `id` in base frame.
     * @param id index of the link (body id).
     * @param joint_pos all joint position in order of `joint.id` of rigid body tree, including fixed joint.
     */
    void getLinkTransform(
        Eigen::Matrix<T, 4, 4>& link_transform,
        const std::size_t id,
        const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        link_transform = Eigen::Matrix<T, 4, 4>::Identity();

        std::vector<RigidBody<T>*> path = this->getPathTo(&this->links_storage_[id]);

        // Accumulate transforms from root to the target node
        for (const RigidBody<T>* rb : path)
        {
            const Joint<T>& joint = rb->joint;
            Eigen::Matrix<T, 4, 4> joint_motion = Eigen::Matrix<T, 4, 4>::Identity();
            if (joint.type == JointTypeRevolute)
            {
                Eigen::AngleAxis<T> aa(joint_pos[joint.id], joint.axis);
                joint_motion.template block<3, 3>(0, 0) = aa.toRotationMatrix();
            }
            else if (joint.type == JointTypePrismatic)
            {
                joint_motion.template block<3, 1>(0, 3) = joint.axis * joint_pos[joint.id];
            }
            // JointTypeFixed has identity motion
            link_transform = link_transform * joint.joint_to_parent_transform * joint_motion;
        }
    }

    /**
     * @brief Calculate transform matrices of all the links in rigid body tree.
     *
     * @param link_transform output. array of link transform matrices.
     * @param joint_pos all joint position in order of `joint.id` of rigid body tree, including fixed joint.
     */
    void getAllLinkTransform(
        std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform,
        const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            if (this->links_storage_[i])
            {
                getLinkTransform(link_transform[i], i, joint_pos);
            }
            else
            {
                link_transform[i] = Eigen::Matrix<T, 4, 4>::Identity();
            }
        }
    }

    /**
     * @brief Calculate transformation matrix of center of mass of link with index `id` in base link frame.
     *
     * @param link_com_transform output. transform matrix of center of mass of link with index `id`.
     * @param id index of link (body id).
     * @param joint_pos all joint position in order of `joint.id` of rigid body tree, including fixed joint.
     */
    void getLinkComTransform(
        Eigen::Matrix<T, 4, 4>& link_com_transform, // Changed from array to single matrix
        const std::size_t id,
        const Eigen::Vector<T, NumDof>& joint_pos) const // Added const
    {
        getLinkTransform(link_com_transform, id, joint_pos); // Get the link frame transform first

        const RigidBody<T>& node = this->links_storage_[id];

        // Add the CoM offset in the link's local frame, rotated to the base frame
        Eigen::Vector<T, 3> com_offset_base = link_com_transform.template block<3,3>(0, 0) * node.center_of_mass;
        link_com_transform.template block<3, 1>(0, 3) += com_offset_base;
    }

    /**
     * @brief Calculate transformation matrix of all the center of mass of links in rigid body tree in base link frame.
     *
     * @param link_com_transform output. array of link center of mass transform matrices.
     * @param joint_pos all joint position in order of `joint.id` of rigid body tree, including fixed joint.
     */
    void getAllLinkComTransform(
        std::array<Eigen::Matrix<T, 4, 4>,NumDof>& link_com_transform,
        const Eigen::Vector<T,NumDof>& joint_pos) const // Added const
    {
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            if (this->links_storage_[i])
            {
                getLinkComTransform(link_com_transform[i], i, joint_pos);
            }
            else
            {
                link_com_transform[i] = Eigen::Matrix<T, 4, 4>::Identity();
            }
        }
    }

    /**
     * @brief Calculate transform matrices of link frame and center of mass frame of all the links in rigid body tree.
     *
     * @param link_transform output. array of link transform matrices.
     * @param link_com_transform output. array of link center of mass transform matrices.
     * @param joint_pos all joint position in order of `joint.id` of rigid body tree, including fixed joint.
     */
    void getAllLinkAndComTransform(
        std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform,
        std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform,
        const Eigen::Vector<T, NumDof>& joint_pos) const // Added const
    {
        getAllLinkTransform(link_transform, joint_pos);
        getAllLinkComTransform(link_com_transform, joint_pos);
    }

    /**
     * @brief Calculate the velocity of each link and center of mass.
     *
     * @param link_lin_vel output. link linear velocity.
     * @param link_ang_vel output. link angular velocity.
     * @param link_com_lin_vel output. link center of mass linear velocity.
     * @param link_com_ang_vel output. link center of mass angular velocity.
     * @param link_transform transform matrices of link frame.
     * @param link_com_transform transform matrices of center of mass of links.
     * @param joint_vel joint velocities (for all joints, including fixed).
     */
    void getLinkVelocity(
        std::array<Eigen::Vector<T, 3>, NumDof>& link_lin_vel,
        std::array<Eigen::Vector<T, 3>, NumDof>& link_ang_vel,
        std::array<Eigen::Vector<T, 3>, NumDof>& link_com_lin_vel,
        std::array<Eigen::Vector<T, 3>, NumDof>& link_com_ang_vel,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform,
        const Eigen::Vector<T, NumDof>& joint_vel) const
    {
        // Initialize velocities
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            link_lin_vel[i].setZero();
            link_ang_vel[i].setZero();
            link_com_lin_vel[i].setZero();
            link_com_ang_vel[i].setZero();
        }

        // Traverse the tree from root to leaves
        std::function<void(const RigidBody<T>*, const Eigen::Vector<T, 3>&, const Eigen::Vector<T, 3>&)> compute_velocities;
        compute_velocities = [&](const RigidBody<T>* body, const Eigen::Vector<T, 3>& parent_lin_vel, const Eigen::Vector<T, 3>& parent_ang_vel)
        {
            if (!body) return;

            const std::size_t i = body->id;
            const Joint<T>& joint = body->joint;
            const Eigen::Matrix<T, 3, 3> rotation = link_transform[i].template block<3,3>(0,0);
            Eigen::Vector<T, 3> joint_axis_world = rotation * joint.axis.template cast<T>();

            Eigen::Vector<T, 3> v_lin = parent_lin_vel;
            Eigen::Vector<T, 3> v_ang = parent_ang_vel;

            if (joint.type == JointTypeRevolute)
            {
                v_ang += joint_axis_world * joint_vel[joint.id];
                // v_lin += parent_ang_vel.cross(link_transform[i].template block<3,1>(0,3) - (body->parent ? link_transform[body->parent->id].template block<3,1>(0,3) : Eigen::Vector<T, 3>::Zero()));
                if (body->parent)
                {
                    v_lin += parent_ang_vel.cross(link_transform[i].template block<3,1>(0,3) - link_transform[body->parent->id].template block<3,1>(0,3));
                }
            }
            else if (joint.type == JointTypePrismatic)
            {
                v_lin += parent_ang_vel.cross(link_transform[i].template block<3,1>(0,3) - (body->parent ? link_transform[body->parent->id].template block<3,1>(0,3) : Eigen::Vector<T, 3>::Zero())) + joint_axis_world * joint_vel[joint.id];
                // Prismatic joints don't add angular velocity from the joint motion itself
            }
            else if (joint.type == JointTypeFixed)
            {
                if (body->parent)
                {
                    v_lin += parent_ang_vel.cross(link_transform[i].template block<3,1>(0,3) - link_transform[body->parent->id].template block<3,1>(0,3));
                }
                 // Fixed joints pass on parent velocity
            }

            link_lin_vel[i] = v_lin;
            link_ang_vel[i] = v_ang;
            link_com_ang_vel[i] = v_ang; // Angular velocity is the same at CoM for a rigid body

            // Linear velocity at CoM: v_com = v_link + omega x r_com/link
            Eigen::Vector<T, 3> r_com = link_com_transform[i].template block<3,1>(0,3) - link_transform[i].template block<3,1>(0,3);
            link_com_lin_vel[i] = v_lin + v_ang.cross(r_com);

            // Recurse for children
            for (const auto* child : body->children)
            {
                compute_velocities(child, v_lin, v_ang);
            }
        };

        if (root_)
        {
            compute_velocities(root_, Eigen::Vector<T, 3>::Zero(), Eigen::Vector<T, 3>::Zero());
        }
    }


    void getLinkSpaceJacobian(
        std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_jacobian,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform) const
    {
        // Initialize Jacobians
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            link_jacobian[i].setZero();
        }

        for (std::size_t i = 0; i < NumDof; ++i)
        { // For each link/body 'i'
            const RigidBody<T>* body_i = links_[i];
            if (!body_i) continue;

            Eigen::Vector<T, 3> pos_i = link_transform[i].template block<3,1>(0,3);

            // Traverse the path from body 'i' to the root to find joints affecting it
            const RigidBody<T>* current = body_i;
            while (current != nullptr)
            {
                const Joint<T>& joint = current->joint;
                const std::size_t j = joint.id; // Joint index in the joint_pos/vel vector

                if (joint.type == JointTypeFixed)
                {
                    // Fixed joints do not contribute to the Jacobian
                    current = current->parent;
                    continue;
                }

                Eigen::Matrix<T, 3, 3> rotation_j = link_transform[current->id].template block<3,3>(0,0);
                Eigen::Vector<T, 3> joint_axis_world = rotation_j * joint.axis.template cast<T>();
                Eigen::Vector<T, 3> pos_j = link_transform[current->id].template block<3,1>(0,3);

                if (joint.type == JointTypeRevolute)
                {
                    link_jacobian[i].template block<3,1>(0,j) = joint_axis_world.cross(pos_i - pos_j);
                    link_jacobian[i].template block<3,1>(3,j) = joint_axis_world;
                }
                else if (joint.type == JointTypePrismatic)
                {
                    link_jacobian[i].template block<3,1>(0,j) = joint_axis_world;
                    link_jacobian[i].template block<3,1>(3,j).setZero();
                }
                current = current->parent;
            }
        }
    }

    void getLinkComSpaceJacobian(
        std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform) const
    {
         // Initialize Jacobians
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            link_com_jacobian[i].setZero();
        }

        for (std::size_t i = 0; i < NumDof; ++i) // For each link/body 'i' (CoM point)
        {
            const RigidBody<T>* body_i = links_[i];
            if (!body_i) continue;

            Eigen::Vector<T, 3> pos_com_i = link_com_transform[i].template block<3,1>(0,3); // Position of CoM 'i'

            // Traverse the path from body 'i' to the root to find joints affecting it
            const RigidBody<T>* current = body_i;
            while (current != nullptr)
            {
                const Joint<T>& joint = current->joint;
                const std::size_t j = joint.id; // Joint index in the joint_pos/vel vector

                if (joint.type == JointTypeFixed)
                {
                    // Fixed joints do not contribute to the Jacobian
                    current = current->parent;
                    continue;
                }

                Eigen::Matrix<T, 3, 3> rotation_j = link_transform[current->id].template block<3,3>(0,0);
                Eigen::Vector<T, 3> joint_axis_world = rotation_j * joint.axis.template cast<T>();
                Eigen::Vector<T, 3> pos_j = link_transform[current->id].template block<3,1>(0,3); // Joint location

                if (joint.type == JointTypeRevolute)
                {
                    link_com_jacobian[i].template block<3,1>(0,j) = joint_axis_world.cross(pos_com_i - pos_j);
                    link_com_jacobian[i].template block<3,1>(3,j) = joint_axis_world;
                }
                else if (joint.type == JointTypePrismatic)
                {
                    link_com_jacobian[i].template block<3,1>(0,j) = joint_axis_world;
                    link_com_jacobian[i].template block<3,1>(3,j).setZero();
                }
                current = current->parent;
            }
        }
    }

    void getLinkComSpaceJacobianDot(
        std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian_dot,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform,
        const std::array<Eigen::Vector<T, 3>, NumDof>& link_lin_vel,
        const std::array<Eigen::Vector<T, 3>, NumDof>& link_ang_vel,
        const std::array<Eigen::Vector<T, 3>, NumDof>& link_com_lin_vel) const
    {
        // Initialize Jacobian Dots
        for (std::size_t i = 0; i < NumDof; ++i)
        {
            link_com_jacobian_dot[i].setZero();
        }

        for (std::size_t i = 0; i < NumDof; ++i)
        { // For each link/body 'i' (CoM point)
            const RigidBody<T>* body_i = links_[i];
            if (!body_i) continue;

            Eigen::Vector<T, 3> vel_com_i = link_com_lin_vel[i]; // Velocity of CoM 'i'

            // Traverse the path from body 'i' to the root to find joints affecting it
            const RigidBody<T>* current = body_i;
            while (current != nullptr)
            {
                const Joint<T>& joint = current->joint;
                const std::size_t j = joint.id; // Joint index in the joint_pos/vel vector

                if (joint.type == JointTypeFixed)
                {
                    // Fixed joints do not contribute to the Jacobian
                    current = current->parent;
                    continue;
                }

                Eigen::Matrix<T, 3, 3> rotation_j = link_transform[current->id].template block<3,3>(0,0);
                Eigen::Vector<T, 3> joint_axis_world = rotation_j * joint.axis.template cast<T>();
                Eigen::Vector<T, 3> pos_j = link_transform[current->id].template block<3,1>(0,3); // Joint location
                Eigen::Vector<T, 3> vel_j = link_lin_vel[current->id]; // Velocity of joint 'j' location
                Eigen::Vector<T, 3> ang_vel_j = link_ang_vel[current->id]; // Angular velocity at joint 'j'

                if (joint.type == JointTypeRevolute)
                {
                    link_com_jacobian_dot[i].template block<3,1>(0,j) =
                        (ang_vel_j.cross(joint_axis_world)).cross(link_com_transform[i].template block<3,1>(0,3) - pos_j) +
                        joint_axis_world.cross(vel_com_i - vel_j);
                    link_com_jacobian_dot[i].template block<3,1>(3,j) = ang_vel_j.cross(joint_axis_world);
                }
                else if (joint.type == JointTypePrismatic)
                {
                    // Based on the ArmModel logic, Jd_v for prismatic involves omega x axis
                    link_com_jacobian_dot[i].template block<3,1>(0,j) = ang_vel_j.cross(joint_axis_world);
                    link_com_jacobian_dot[i].template block<3,1>(3,j).setZero();
                }
                current = current->parent;
            }
        }
    }

    Eigen::Matrix<T,NumDof,NumDof> getJointSpaceMassMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian) const
    {
        Eigen::Matrix<T,NumDof,NumDof> generalized_mass_matrix = Eigen::Matrix<T,NumDof,NumDof>::Zero();
        Eigen::Matrix<T, 3, NumDof> J_v;
        Eigen::Matrix<T, 3, NumDof> J_w;
        Eigen::Matrix<T, 3, 3> inertia_g;

        for (std::size_t i = 0; i < NumDof; ++i)
        {
            const RigidBody<T>* body = links_[i];
            if (!body || body->mass <= T(0)) continue; // Skip if body doesn't exist or has no mass

            /* Extract linear and angular velocity parts of jacobian matrix for link i */
            J_v = link_com_jacobian[i].template block<3, NumDof>(0, 0);
            J_w = link_com_jacobian[i].template block<3, NumDof>(3, 0);

            /* Rotate inertia matrix to base frame */
            inertia_g = link_com_transform[i].template block<3,3>(0,0) * body->inertia.template cast<T>() * link_com_transform[i].template block<3,3>(0,0).transpose();

            generalized_mass_matrix.noalias() += J_v.transpose() * body->mass * J_v + J_w.transpose() * inertia_g * J_w;
        }
        return generalized_mass_matrix;
    }

    Eigen::Matrix<T,NumDof,NumDof> getJointSpaceCoriolisMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, NumDof>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian,
        const std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian_dot,
        const Eigen::Vector<T,NumDof>& joint_vel) const
    {
        auto getSymmetricSkew = [](const Eigen::Vector<T, 3>& vec) -> Eigen::Matrix<T, 3, 3>
        {
            Eigen::Matrix<T, 3, 3> skew_matrix;
            skew_matrix << T(0), -vec(2), vec(1),
                        vec(2), T(0), -vec(0),
                        -vec(1), vec(0), T(0);
            return skew_matrix;
        };

        Eigen::Matrix<T,NumDof,NumDof> centrifugal_coriolis_matrix = Eigen::Matrix<T,NumDof,NumDof>::Zero();
        Eigen::Matrix<T, 3, NumDof> J_v, J_w, Jd_v, Jd_w;
        Eigen::Matrix<T, 3, 3> inertia_g;

        for (std::size_t i = 0; i < NumDof; ++i)
        {
            const RigidBody<T>* body = links_[i];
            if (!body || body->mass <= T(0)) continue;

            /* Extract parts of jacobian and its time derivative */
            J_v = link_com_jacobian[i].template block<3, NumDof>(0, 0);
            J_w = link_com_jacobian[i].template block<3, NumDof>(3, 0);
            Jd_v = link_com_jacobian_dot[i].template block<3, NumDof>(0, 0);
            Jd_w = link_com_jacobian_dot[i].template block<3, NumDof>(3, 0);

            /* Rotate inertia matrix to base frame */
            inertia_g = link_com_transform[i].template block<3,3>(0,0) * body->inertia.template cast<T>() * link_com_transform[i].template block<3,3>(0,0).transpose();

            centrifugal_coriolis_matrix.noalias() +=
                J_v.transpose() * body->mass * Jd_v +
                J_w.transpose() * inertia_g * Jd_w +
                J_w.transpose() * getSymmetricSkew(J_w * joint_vel) * inertia_g * J_w; // Coriolis term
        }
        return centrifugal_coriolis_matrix;
    }

    Eigen::Vector<T, NumDof> getJointSpaceGravityCompensate(
        const std::array<Eigen::Matrix<T, 6, NumDof>, NumDof>& link_com_jacobian) const
    {
        Eigen::Vector<T, NumDof> gravity_compensate = Eigen::Vector<T, NumDof>::Zero();
        Eigen::Matrix<T, 3, NumDof> J_v;
        // Eigen::Matrix<T, 3, NumDof> J_w; // Not needed for gravity

        // Assume gravity is along negative Z base frame
        static const Eigen::Vector<T, 3> base_gravity(0, 0, -9.81);

        for (std::size_t i = 0; i < NumDof; ++i)
        {
            const RigidBody<T>* body = links_[i];
            if (!body || body->mass <= T(0)) continue;

            /* Extract linear velocity part of jacobian matrix */
            J_v = link_com_jacobian[i].template block<3, NumDof>(0, 0);
            // J_w = link_com_jacobian[i].template block<3, NumDof>(3, 0); // Not used

            gravity_compensate.noalias() += J_v.transpose() * (body->mass * base_gravity);
        }
        return gravity_compensate;
    }

    /**
     * @brief Get task space inverse dynamics matrices.
     *
     * @param task_space_mass_matrix
     * @param task_space_bias_force
     * @param joint_pos
     * @param joint_vel
     *
     * @note From "Modern Robotics - Mechanics, Planning and Control" eq8.90 and eq8.91
     * @note Assumes the last link (highest ID with a non-fixed joint) is the end-effector.
     *       This might need adjustment based on your specific tree structure.
     */
    void getTaskSpaceInverseDynamics(
        Eigen::Matrix<T,6,6>& task_space_mass_matrix,
        Eigen::Matrix<T, 6, 1>& task_space_bias_force, // Changed to Matrix for consistency
        const Eigen::Vector<T, NumDof>& joint_pos,
        const Eigen::Vector<T, NumDof>& joint_vel) const
    {
        // --- 1. Forward Kinematics and Velocities ---
        std::array<Eigen::Matrix<T, 4, 4>, NumDof> link_transform;
        std::array<Eigen::Matrix<T, 4, 4>, NumDof> link_com_transform;
        std::array<Eigen::Vector<T, 3>, NumDof> link_lin_vel;
        std::array<Eigen::Vector<T, 3>, NumDof> link_ang_vel;
        std::array<Eigen::Vector<T, 3>, NumDof> link_com_lin_vel;
        std::array<Eigen::Vector<T, 3>, NumDof> link_com_ang_vel;
        std::array<Eigen::Matrix<T, 6, NumDof>, NumDof> link_com_jacobian;
        std::array<Eigen::Matrix<T, 6, NumDof>, NumDof> link_com_jacobian_dot;

        getAllLinkAndComTransform(link_transform, link_com_transform, joint_pos);
        getLinkVelocity(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel,
                        link_transform, link_com_transform, joint_vel);
        getLinkComSpaceJacobian(link_com_jacobian, link_transform, link_com_transform);
        getLinkComSpaceJacobianDot(link_com_jacobian_dot, link_transform, link_com_transform,
                                   link_lin_vel, link_ang_vel, link_com_lin_vel);

        // --- 2. Joint Space Dynamics ---
        Eigen::Matrix<T, NumDof, NumDof> generalized_mass_matrix = getJointSpaceMassMatrix(link_com_transform, link_com_jacobian);
        Eigen::Matrix<T, NumDof, NumDof> centrifugal_coriolis_matrix = getJointSpaceCoriolisMatrix(
            link_com_transform, link_com_jacobian, link_com_jacobian_dot, joint_vel);
        Eigen::Matrix<T, NumDof, 1> gravity_compensate = getJointSpaceGravityCompensate(link_com_jacobian);

        // --- 3. Identify End-Effector ---
        // Find the last body with a non-fixed joint (assuming it's the EE)
        std::size_t ee_body_id = 0;
        for (std::size_t i = 0; i < NumDof; ++i) {
            const RigidBody<T>* body = links_[i];
            if (body && body->joint.type != JointTypeFixed) {
                ee_body_id = i; // Keep updating, the last one will be the EE candidate
            }
        }
        const std::size_t end_effector_id = ee_body_id;

        // --- 4. Task Space Calculation ---
        Eigen::Matrix<T, 6, NumDof> jacobian = link_com_jacobian[end_effector_id]; // EE CoM Jacobian

        // Compute inverse or pseudo-inverse of Jacobian
        Eigen::Matrix<T, NumDof, 6> jacobian_inv;
        double det_JJT = (jacobian * jacobian.transpose()).determinant();
        if (std::abs(det_JJT) > T(1e-6)) { // Check for numerical invertibility
             jacobian_inv = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
        } else {
             // Use SVD-based pseudo-inverse for robustness
             jacobian_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        }
        Eigen::Matrix<T, 6, NumDof> jacobian_inv_T = jacobian_inv.transpose();

        Eigen::Matrix<T, 6, 1> twist = jacobian * joint_vel; // EE twist

        // Task Space Inertia Matrix (Lambda)
        Eigen::Matrix<T, 6, 6> task_space_mass_matrix_inv = jacobian * generalized_mass_matrix.inverse() * jacobian.transpose();
        double det_Lambda_inv = task_space_mass_matrix_inv.determinant();
        if (std::abs(det_Lambda_inv) > T(1e-6)) {
            task_space_mass_matrix = task_space_mass_matrix_inv.inverse();
        } else {
            task_space_mass_matrix = task_space_mass_matrix_inv.completeOrthogonalDecomposition().pseudoInverse();
        }

        // Task Space Bias Force (mu)
        // mu = J^-T * (C * q_dot) + J^-T * G - Lambda * (J_dot * q_dot)
        task_space_bias_force =
            jacobian_inv_T * (centrifugal_coriolis_matrix * joint_vel) +
            jacobian_inv_T * gravity_compensate -
            task_space_mass_matrix * (link_com_jacobian_dot[end_effector_id] * joint_vel);
    }
private:
    std::array<RigidBody<T>, Size> links_storage_;

    // Helper to create a 4x4 transform matrix from XYZ translation and RPY rotation (Euler ZYX)
    const static Eigen::Matrix<T, 4, 4> getTransformFromRotationAndTranslation(T x, T y, T z, T roll, T pitch, T yaw)
    {
        Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
        Eigen::AngleAxis<T> angle_x(roll, Eigen::Matrix<T, 3, 1>::UnitX());
        Eigen::AngleAxis<T> angle_y(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
        Eigen::AngleAxis<T> angle_z(yaw, Eigen::Matrix<T, 3, 1>::UnitZ());
        Eigen::Quaternion<T> q = angle_z * angle_y * angle_x; // ZYX convention
        transform.template block<3, 3>(0, 0) = q.toRotationMatrix();
        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;
        return transform;
    }

    // Helper to create a 3x3 inertia matrix from principal moments and products
    const static Eigen::Matrix<T, 3, 3> getInertiaMatrix(T ixx, T ixy, T ixz, T iyy, T iyz, T izz)
    {
        Eigen::Matrix<T, 3, 3> inertia;
        inertia << ixx, ixy, ixz,
                ixy, iyy, iyz,
                ixz, iyz, izz;
        return inertia;
    }
};
#endif // __RIGIDBODYTREE_HPP__