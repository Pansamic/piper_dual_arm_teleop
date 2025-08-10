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
    std::size_t id;
    JointType type;
    Eigen::Vector<T, 3> axis;
    Eigen::Vector<T, 2> limits;
    Eigen::Matrix<T, 4, 4> joint_to_parent_transform;

    constexpr Joint() : id(0), type(JointType::JointTypeFixed), axis(Eigen::Vector<T, 3>::Zero()), limits(Eigen::Vector<T, 2>::Zero()), joint_to_parent_transform(Eigen::Matrix<T, 4, 4>::Zero()){}
    /* @todo test constexpr constant variable declaration. */
    constexpr Joint(
        std::size_t id,
        JointType type,
        Eigen::Vector<T, 3> axis,
        Eigen::Vector<T, 2> limits,
        Eigen::Matrix<T, 4, 4> joint_to_parent_transform) :
        id(id), type(type), axis(axis), limits(limits), joint_to_parent_transform(joint_to_parent_transform){}
};

template<typename T>
struct RigidBody
{
    std::size_t id;
    T mass;
    Eigen::Vector<T, 3> center_of_mass;
    Eigen::Matrix<T, 3, 3> inertia;
    Joint<T> joint;

    constexpr RigidBody() : id(0), mass(0), center_of_mass(Eigen::Vector<T, 3>::Zero()), inertia(Eigen::Matrix<T, 3, 3>::Zero()), joint() {}
    // constexpr RigidBody(
    //     std::size_t id,
    //     T mass,
    //     Eigen::Vector<T, 3> center_of_mass,
    //     Eigen::Matrix<T, 3, 3> inertia,
    //     Joint<T> joint
    // )
};

template<typename T, std::size_t NumLink, std::size_t NumDof>
class RigidBodyTree : private MultiNodeTree<RigidBody<T>*>
{
public:
    struct Builder
    {
        std::size_t root_id;
        std::array<T, 3> base_position;
        std::array<T, 3> base_orientation;
        std::array<T, NumLink> mass;
        std::array<std::array<T, 3>, NumLink> initial_position;
        std::array<std::array<T, 3>, NumLink> initial_orientation;
        std::array<std::array<T, 3>, NumLink> com;
        std::array<std::array<T, 6>, NumLink> inertia;
        std::array<JointType, NumLink> joint_type;
        std::array<std::array<T, 3>, NumLink> joint_axis;
        std::array<std::array<T, 2>, NumLink> joint_limits;
        std::array<std::vector<std::size_t>, NumLink> children_indices;

        RigidBody<T> buildRigidBodyById(std::size_t id) const
        {
            RigidBody<T> link;
            link.id = id;
            link.mass = mass[id];
            link.center_of_mass = Eigen::Map<const Eigen::Vector<T, 3>>(com[id].data());
            link.inertia = getInertiaMatrix(
                inertia[id][0], inertia[id][1], inertia[id][2],
                inertia[id][3], inertia[id][4], inertia[id][5]
            );
            link.joint.id = id;
            link.joint.type = joint_type[id];
            link.joint.axis = Eigen::Map<const Eigen::Vector<T, 3>>(joint_axis[id].data());
            link.joint.limits = Eigen::Map<const Eigen::Vector<T, 2>>(joint_limits[id].data());
            link.joint.joint_to_parent_transform = getTransformFromRotationAndTranslation(
                initial_position[id][0], initial_position[id][1], initial_position[id][2],
                initial_orientation[id][0], initial_orientation[id][1], initial_orientation[id][2]
            );
            return link;
        }
    };

    RigidBodyTree() = default;

    ~RigidBodyTree() = default;

    bool build(const Builder& builder)
    {
        if (builder.root_id >= NumLink)
        {
            return false;
        }

        std::size_t dof_in_data = 0;
        for ( const auto it : builder.joint_type )
        {
            if ( it == JointType::JointTypePrismatic || it == JointType::JointTypeRevolute )
            {
                dof_in_data++;
            }
        }
        if ( dof_in_data != NumDof )
        {
            return false;
        }

        for (std::size_t i = 0; i < NumLink; ++i)
        {
            links_storage_[i] = builder.buildRigidBodyById(i);
        }

        this->base_transform_ = getTransformFromRotationAndTranslation(
            builder.base_position[0], builder.base_position[1], builder.base_position[2],
            builder.base_orientation[0], builder.base_orientation[1], builder.base_orientation[2]
        );

        auto& root_joint_transform = links_storage_[builder.root_id].joint.joint_to_parent_transform;
        root_joint_transform = this->base_transform_ * root_joint_transform;

        this->setRoot(&links_storage_[builder.root_id]);
        auto root_node = this->getRoot();
        if (!root_node)
        {
            return false;
        }

        std::queue<std::shared_ptr<TreeNode<RigidBody<T>*>>> node_queue;
        node_queue.push(root_node);

        while (!node_queue.empty())
        {
            auto current_node = node_queue.front();
            node_queue.pop();
            std::size_t current_id = current_node->data->id;

            for (std::size_t child_id : builder.children_indices[current_id])
            {
                if (child_id >= NumLink)
                {
                    this->clear();
                    return false;
                }
                auto child_node = current_node->addChild(&links_storage_[child_id]);
                node_queue.push(child_node);
            }
        }

        return true;
    }

    const RigidBody<T>* findLinkByID(std::size_t id) const
    {
        if ( id > NumLink )
        {
            return nullptr;
        }
        return &this->links_storage_[id];
    }

    Eigen::Matrix<T, 4, 4> getBaseTransform() const
    {
        return this->base_transform_;
    }

    Eigen::Matrix<T, 4, 4> getLinkTransform(std::size_t id, const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
        auto path = this->getPathTo(&this->links_storage_[id]);
        auto joint_pos_expand = this->expand(joint_pos);

        for (const RigidBody<T>* rb : path)
        {
            const Joint<T>& joint = rb->joint;
            Eigen::Matrix<T, 4, 4> joint_motion = Eigen::Matrix<T, 4, 4>::Identity();

            if (joint.type == JointTypeRevolute)
            {
                Eigen::AngleAxis<T> aa(joint_pos_expand[joint.id], joint.axis);
                joint_motion.template block<3, 3>(0, 0) = aa.toRotationMatrix();
            }
            else if (joint.type == JointTypePrismatic)
            {
                joint_motion.template block<3, 1>(0, 3) = joint.axis * joint_pos_expand[joint.id];
            }

            transform = transform * joint.joint_to_parent_transform * joint_motion;
        }
        return transform;
    }

    std::array<Eigen::Matrix<T, 4, 4>, NumLink> getAllLinkTransform(const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        std::array<Eigen::Matrix<T, 4, 4>, NumLink> transforms;
        for (std::size_t i = 0; i < NumLink; ++i)
        {
            transforms[i] = getLinkTransform(i, joint_pos);
        }
        return transforms;
    }

    Eigen::Matrix<T, 4, 4> getLinkComTransform(std::size_t id, const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        auto link_transform = getLinkTransform(id, joint_pos);
        const auto& com = links_storage_[id].center_of_mass;
        Eigen::Vector<T, 3> com_in_base = link_transform.template block<3, 3>(0, 0) * com;
        link_transform.template block<3, 1>(0, 3) += com_in_base;
        return link_transform;
    }

    std::array<Eigen::Matrix<T, 4, 4>, NumLink> getAllLinkComTransform(const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        std::array<Eigen::Matrix<T, 4, 4>, NumLink> com_transforms;
        for (std::size_t i = 0; i < NumLink; ++i)
        {
            com_transforms[i] = getLinkComTransform(i, joint_pos);
        }
        return com_transforms;
    }

    std::tuple<std::array<Eigen::Matrix<T, 4, 4>, NumLink>, std::array<Eigen::Matrix<T, 4, 4>, NumLink>>
    getAllLinkAndComTransform(const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        auto link_transforms = getAllLinkTransform(joint_pos);
        auto com_transforms = getAllLinkComTransform(joint_pos);
        return std::make_tuple(std::move(link_transforms), std::move(com_transforms));
    }

    std::tuple<
        std::array<Eigen::Vector<T, 3>, NumLink>,
        std::array<Eigen::Vector<T, 3>, NumLink>,
        std::array<Eigen::Vector<T, 3>, NumLink>,
        std::array<Eigen::Vector<T, 3>, NumLink>>
    getLinkVelocity(
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_com_transform,
        const Eigen::Vector<T, NumDof>& joint_vel) const
    {
        std::array<Eigen::Vector<T, 3>, NumLink> link_lin_vel;
        std::array<Eigen::Vector<T, 3>, NumLink> link_ang_vel;
        std::array<Eigen::Vector<T, 3>, NumLink> link_com_lin_vel;
        std::array<Eigen::Vector<T, 3>, NumLink> link_com_ang_vel;
        auto joint_vel_expand = this->expand(joint_vel);
        for (auto& v : link_lin_vel)
        {
            v.setZero();
        }
        for (auto& v : link_ang_vel)
        {
            v.setZero();
        }
        for (auto& v : link_com_lin_vel)
        {
            v.setZero();
        }
        for (auto& v : link_com_ang_vel)
        {
            v.setZero();
        }

        std::function<void(const RigidBody<T>*, Eigen::Vector<T, 3>, Eigen::Vector<T, 3>)> compute_velocities;
        compute_velocities = [&](const RigidBody<T>* body, Eigen::Vector<T, 3> parent_lin_vel, Eigen::Vector<T, 3> parent_ang_vel)
        {
            if (!body)
            {
                return;
            }
            const std::size_t i = body->id;
            const Joint<T>& joint = body->joint;
            const Eigen::Matrix<T, 3, 3>& R = link_transform[i].template block<3,3>(0,0);
            Eigen::Vector<T, 3> joint_axis_world = R * joint.axis;

            Eigen::Vector<T, 3> v_lin = parent_lin_vel;
            Eigen::Vector<T, 3> v_ang = parent_ang_vel;

            if (joint.type == JointTypeRevolute)
            {
                v_ang += joint_axis_world * joint_vel_expand[joint.id];
                if (body->id != 0)
                {
                    auto parent_id = this->findParent(body);
                    if (parent_id)
                    {
                        v_lin += parent_ang_vel.cross(
                            link_transform[i].template block<3,1>(0,3) -
                            link_transform[parent_id->id].template block<3,1>(0,3)
                        );
                    }
                }
            }
            else if (joint.type == JointTypePrismatic)
            {
                if (body->id != 0)
                {
                    auto parent_id = this->findParent(body);
                    if (parent_id)
                    {
                        v_lin += parent_ang_vel.cross(
                            link_transform[i].template block<3,1>(0,3) -
                            link_transform[parent_id->id].template block<3,1>(0,3)
                        );
                    }
                }
                v_lin += joint_axis_world * joint_vel_expand[joint.id];
            }

            link_lin_vel[i] = v_lin;
            link_ang_vel[i] = v_ang;
            link_com_ang_vel[i] = v_ang;

            Eigen::Vector<T, 3> r_com = link_com_transform[i].template block<3,1>(0,3) - link_transform[i].template block<3,1>(0,3);
            link_com_lin_vel[i] = v_lin + v_ang.cross(r_com);

            for (const auto& child_ptr : this->getChildren(body))
            {
                compute_velocities(child_ptr->data, v_lin, v_ang);
            }
        };

        if (this->getRoot())
        {
            compute_velocities(this->getRoot()->data, Eigen::Vector<T,3>::Zero(), Eigen::Vector<T,3>::Zero());
        }

        return std::make_tuple(link_lin_vel, link_ang_vel, link_com_lin_vel, link_com_ang_vel);
    }

    Eigen::Matrix<T, 6, NumDof> getLinkSpaceJacobian(std::size_t id, const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        Eigen::Matrix<T, 6, NumDof> jacobian = Eigen::Matrix<T, 6, NumDof>::Zero();
        auto path = this->getPathTo(&links_storage_[id]);
        Eigen::Matrix<T, 4, 4> T_base_to_link = getLinkTransform(id, joint_pos);
        Eigen::Vector<T, 3> p_effector = T_base_to_link.template block<3,1>(0,3);

        for (const RigidBody<T>* rb : path)
        {
            const Joint<T>& joint = rb->joint;
            if (joint.type == JointTypeFixed)
            {
                continue;
            }

            std::size_t j = joint.id;
            Eigen::Matrix<T, 4, 4> T_base_to_joint = getLinkTransform(rb->id, joint_pos);
            Eigen::Vector<T, 3> p_joint = T_base_to_joint.template block<3,1>(0,3);
            Eigen::Matrix<T, 3, 3> R_base_to_joint = T_base_to_joint.template block<3,3>(0,0);
            Eigen::Vector<T, 3> axis_world = R_base_to_joint * joint.axis;

            if (joint.type == JointTypeRevolute)
            {
                jacobian.template block<3,1>(0,j) = axis_world.cross(p_effector - p_joint);
                jacobian.template block<3,1>(3,j) = axis_world;
            }
            else if (joint.type == JointTypePrismatic)
            {
                jacobian.template block<3,1>(0,j) = axis_world;
                jacobian.template block<3,1>(3,j).setZero();
            }
        }
        return jacobian;
    }

    std::array<Eigen::Matrix<T, 6, NumDof>, NumLink> getAllLinkSpaceJacobian(const Eigen::Vector<T, NumDof>& joint_pos) const
    {
        std::array<Eigen::Matrix<T, 6, NumDof>, NumLink> jacobians;
        for (std::size_t i = 0; i < NumLink; ++i)
        {
            jacobians[i] = this->getLinkSpaceJacobian(i, joint_pos);
        }
        return jacobians;
    }

    std::array<Eigen::Matrix<T, 6, NumLink>, NumLink> getLinkComSpaceJacobian(
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_com_transform) const
    {
        std::array<Eigen::Matrix<T, 6, NumLink>, NumLink> jacobians;
        for (auto& J : jacobians)
        {
            J.setZero();
        }

        for (std::size_t i = 0; i < NumLink; ++i)
        {
            const RigidBody<T>* body = &links_storage_[i];
            Eigen::Vector<T, 3> p_com_i = link_com_transform[i].template block<3,1>(0,3);

            const RigidBody<T>* current = body;
            while (current != nullptr)
            {
                const Joint<T>& joint = current->joint;
                if (joint.type == JointTypeFixed)
                {
                    current = (current == body) ? nullptr : findParent(current)->data;
                    continue;
                }

                std::size_t j = joint.id;
                Eigen::Matrix<T, 3, 3> R_j = link_transform[current->id].template block<3,3>(0,0);
                Eigen::Vector<T, 3> axis_world = R_j * joint.axis;
                Eigen::Vector<T, 3> p_j = link_transform[current->id].template block<3,1>(0,3);

                if (joint.type == JointTypeRevolute)
                {
                    jacobians[i].template block<3,1>(0,j) = axis_world.cross(p_com_i - p_j);
                    jacobians[i].template block<3,1>(3,j) = axis_world;
                }
                else if (joint.type == JointTypePrismatic)
                {
                    jacobians[i].template block<3,1>(0,j) = axis_world;
                    jacobians[i].template block<3,1>(3,j).setZero();
                }

                current = (current == body) ? nullptr : findParent(current)->data;
            }
        }
        return jacobians;
    }

    std::array<Eigen::Matrix<T, 6, NumLink>, NumLink> getLinkComSpaceJacobianDot(
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_transform,
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_com_transform,
        const std::array<Eigen::Vector<T, 3>, NumLink>& link_lin_vel,
        const std::array<Eigen::Vector<T, 3>, NumLink>& link_ang_vel,
        const std::array<Eigen::Vector<T, 3>, NumLink>& link_com_lin_vel) const
    {
        std::array<Eigen::Matrix<T, 6, NumLink>, NumLink> jacobian_dot;
        for (auto& Jd : jacobian_dot)
        {
            Jd.setZero();
        }

        for (std::size_t i = 0; i < NumLink; ++i)
        {
            const RigidBody<T>* body = &links_storage_[i];
            Eigen::Vector<T, 3> v_com_i = link_com_lin_vel[i];

            RigidBody<T>* current = body;
            while (current != nullptr)
            {
                const Joint<T>& joint = current->joint;
                if (joint.type == JointTypeFixed)
                {
                    current = (current == body) ? nullptr : findParent(current)->data;
                    continue;
                }

                std::size_t j = joint.id;
                Eigen::Matrix<T, 3, 3> R_j = link_transform[current->id].template block<3,3>(0,0);
                Eigen::Vector<T, 3> axis_world = R_j * joint.axis;
                Eigen::Vector<T, 3> p_j = link_transform[current->id].template block<3,1>(0,3);
                Eigen::Vector<T, 3> v_j = link_lin_vel[current->id];
                Eigen::Vector<T, 3> omega_j = link_ang_vel[current->id];

                if (joint.type == JointTypeRevolute)
                {
                    jacobian_dot[i].template block<3,1>(0,j) =
                        (omega_j.cross(axis_world)).cross(link_com_transform[i].template block<3,1>(0,3) - p_j) +
                        axis_world.cross(v_com_i - v_j);
                    jacobian_dot[i].template block<3,1>(3,j) = omega_j.cross(axis_world);
                }
                else if (joint.type == JointTypePrismatic)
                {
                    jacobian_dot[i].template block<3,1>(0,j) = omega_j.cross(axis_world);
                    jacobian_dot[i].template block<3,1>(3,j).setZero();
                }

                current = (current == body) ? nullptr : findParent(current)->data;
            }
        }
        return jacobian_dot;
    }

    Eigen::Matrix<T, NumDof, NumDof> getJointSpaceMassMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, NumDof>, NumLink>& link_com_jacobian) const
    {
        Eigen::Matrix<T, NumDof, NumDof> M = Eigen::Matrix<T, NumDof, NumDof>::Zero();
        for (std::size_t i = 0; i < NumLink; ++i)
        {
            const auto& body = links_storage_[i];
            if (body.mass <= T(0))
            {
                continue;
            }

            Eigen::Matrix<T, 3, NumDof> Jv = link_com_jacobian[i].template block<3, NumDof>(0, 0);
            Eigen::Matrix<T, 3, NumDof> Jw = link_com_jacobian[i].template block<3, NumDof>(3, 0);
            Eigen::Matrix<T, 3, 3> I_base = link_com_transform[i].template block<3,3>(0,0) * body.inertia * link_com_transform[i].template block<3,3>(0,0).transpose();

            M.noalias() += Jv.transpose() * body.mass * Jv + Jw.transpose() * I_base * Jw;
        }
        return M;
    }

    Eigen::Matrix<T, NumLink, NumLink> getJointSpaceCoriolisMatrix(
        const std::array<Eigen::Matrix<T, 4, 4>, NumLink>& link_com_transform,
        const std::array<Eigen::Matrix<T, 6, NumLink>, NumLink>& link_com_jacobian,
        const std::array<Eigen::Matrix<T, 6, NumLink>, NumLink>& link_com_jacobian_dot,
        const Eigen::Vector<T, NumDof>& joint_vel) const
    {
        auto getSkew = [](const Eigen::Vector<T, 3>& w)
        {
            Eigen::Matrix<T, 3, 3> S;
            S << 0, -w(2), w(1),
                 w(2), 0, -w(0),
                 -w(1), w(0), 0;
            return S;
        };

        Eigen::Matrix<T, NumLink, NumLink> C = Eigen::Matrix<T, NumLink, NumLink>::Zero();
        for (std::size_t i = 0; i < NumLink; ++i)
        {
            const auto& body = links_storage_[i];
            if (body.mass <= T(0))
            {
                continue;
            }

            Eigen::Matrix<T, 3, NumLink> Jv = link_com_jacobian[i].template block<3, NumLink>(0, 0);
            Eigen::Matrix<T, 3, NumLink> Jw = link_com_jacobian[i].template block<3, NumLink>(3, 0);
            Eigen::Matrix<T, 3, NumLink> Jdv = link_com_jacobian_dot[i].template block<3, NumLink>(0, 0);
            Eigen::Matrix<T, 3, NumLink> Jdw = link_com_jacobian_dot[i].template block<3, NumLink>(3, 0);
            Eigen::Matrix<T, 3, 3> I_base = link_com_transform[i].template block<3,3>(0,0) * body.inertia * link_com_transform[i].template block<3,3>(0,0).transpose();

            C.noalias() += Jv.transpose() * body.mass * Jdv
                         + Jw.transpose() * I_base * Jdw
                         + Jw.transpose() * getSkew(Jw * joint_vel) * I_base * Jw;
        }
        return C;
    }

    Eigen::Vector<T, NumDof> getJointSpaceGravityCompensate(
        const std::array<Eigen::Matrix<T, 6, NumLink>, NumLink>& link_com_jacobian) const
    {
        Eigen::Vector<T, NumLink> G = Eigen::Vector<T, NumLink>::Zero();
        const Eigen::Vector<T, 3> gravity(0, 0, -9.81);

        for (std::size_t i = 0; i < NumLink; ++i)
        {
            const auto& body = links_storage_[i];
            if (body.mass <= T(0))
            {
                continue;
            }
            Eigen::Matrix<T, 3, NumLink> Jv = link_com_jacobian[i].template block<3, NumLink>(0, 0);
            G.noalias() += Jv.transpose() * (body.mass * gravity);
        }
        return this->squeeze(G);
    }

    std::tuple<Eigen::Matrix<T, 6, 6>, Eigen::Vector<T, 6>>
    getTaskSpaceInverseDynamics(
        const Eigen::Vector<T, NumDof>& joint_pos,
        const Eigen::Vector<T, NumDof>& joint_vel) const
    {
        auto [link_tf, com_tf] = getAllLinkAndComTransform(joint_pos);
        auto [lin_vel, ang_vel, com_lin_vel, com_ang_vel] = getLinkVelocity(link_tf, com_tf, joint_vel);
        auto com_jac = getLinkComSpaceJacobian(link_tf, com_tf);
        auto com_jac_dot = getLinkComSpaceJacobianDot(link_tf, com_tf, lin_vel, ang_vel, com_lin_vel);

        Eigen::Matrix<T, NumLink, NumLink> M = getJointSpaceMassMatrix(com_tf, com_jac);
        Eigen::Matrix<T, NumLink, NumLink> C = getJointSpaceCoriolisMatrix(com_tf, com_jac, com_jac_dot, joint_vel);
        Eigen::Vector<T, NumLink> G = getJointSpaceGravityCompensate(com_jac);

        std::size_t ee_id = NumLink - 1;
        while (ee_id > 0 && links_storage_[ee_id].joint.type == JointTypeFixed)
        {
            --ee_id;
        }

        Eigen::Matrix<T, 6, NumLink> J = com_jac[ee_id];
        Eigen::Matrix<T, 6, NumLink> J_dot = com_jac_dot[ee_id];
        Eigen::Matrix<T, NumLink, 6> J_inv;
        if ((J * J.transpose()).determinant() > 1e-6)
        {
            J_inv = J.transpose() * (J * J.transpose()).inverse();
        }
        else
        {
            J_inv = J.completeOrthogonalDecomposition().pseudoInverse();
        }

        Eigen::Matrix<T, 6, 6> Lambda_inv = J * M.inverse() * J.transpose();
        Eigen::Matrix<T, 6, 6> Lambda;
        if (Lambda_inv.determinant() > 1e-6)
        {
            Lambda = Lambda_inv.inverse();
        }
        else
        {
            Lambda = Lambda_inv.completeOrthogonalDecomposition().pseudoInverse();
        }

        Eigen::Vector<T, 6> mu = J_inv.transpose() * (C * joint_vel + G) - Lambda * (J_dot * joint_vel);

        return std::make_tuple(Lambda, mu);
    }

private:
    Eigen::Matrix<T, 4, 4> base_transform_;
    std::array<RigidBody<T>, NumLink> links_storage_;

    const static Eigen::Matrix<T, 4, 4> getTransformFromRotationAndTranslation(T x, T y, T z, T roll, T pitch, T yaw)
    {
        Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
        Eigen::AngleAxis<T> rx(roll, Eigen::Vector<T,3>::UnitX());
        Eigen::AngleAxis<T> ry(pitch, Eigen::Vector<T,3>::UnitY());
        Eigen::AngleAxis<T> rz(yaw, Eigen::Vector<T,3>::UnitZ());
        Eigen::Quaternion<T> q = rz * ry * rx;
        transform.template block<3,3>(0,0) = q.toRotationMatrix();
        transform(0,3) = x;
        transform(1,3) = y;
        transform(2,3) = z;
        return transform;
    }

    const static Eigen::Matrix<T, 3, 3> getInertiaMatrix(T ixx, T ixy, T ixz, T iyy, T iyz, T izz)
    {
        Eigen::Matrix<T, 3, 3> I;
        I << ixx, ixy, ixz,
             ixy, iyy, iyz,
             ixz, iyz, izz;
        return I;
    }

    Eigen::Vector<T, NumDof> squeeze(const Eigen::Vector<T, NumLink>& vec) const
    {
        Eigen::Vector<T, NumDof> sq_vec = Eigen::Vector<T, NumDof>::Zero();

        for ( int i=0, j=0 ; i<NumLink ; i++ )
        {
            if ( this->links_storage_[i].joint.type == JointType::JointTypeRevolute ||
                 this->links_storage_[i].joint.type == JointType::JointTypePrismatic )
            {
                sq_vec(j) = vec(i);
                j++;
            }
        }

        return sq_vec;
    }

    Eigen::Vector<T, NumLink> expand(const Eigen::Vector<T, NumDof>& vec) const
    {
        Eigen::Vector<T, NumLink> ex_vec = Eigen::Vector<T, NumLink>::Zero();

        for ( int i=0, j=0 ; i<NumLink ; i++ )
        {
            if ( this->links_storage_[i].joint.type == JointType::JointTypeRevolute ||
                 this->links_storage_[i].joint.type == JointType::JointTypePrismatic )
            {
                ex_vec(i) = vec(j);
                j++;
            }
        }
    }
};

#endif // __RIGIDBODYTREE_HPP__