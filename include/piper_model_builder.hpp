/**
 * @file piper_model_builder.hpp
 * @brief Definition and builder for the Piper robotic arm model.
 * @version 0.1
 * @date 2025-08-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __PIPER_MODEL_HPP__
#define __PIPER_MODEL_HPP__

#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <cstddef>

#include <rigidbodytree.hpp>

namespace PiperArmModelBuilder
{
// --- Configuration ---
constexpr std::size_t NumLink = 7;
constexpr std::size_t NumDof = 6;

// Parent Index Definition (1-based ID of parent, 0 for root)
// Kinematic chain: Base(0) -> 1 -> 2 -> 3 -> 4 -> 5 -> 6(Gripper Base) -> 7, 8(Fingers)
// Note: This array is used internally to build the children_indices in the Builder.
// const std::array<std::size_t, NumLink> piper_parent_index =
// {{
//     0, // Link 0 (Base) has no parent
//     1, // Link 1's parent is Link 0 (ID 1)
//     2, // Link 2's parent is Link 1 (ID 2)
//     3, // Link 3's parent is Link 2 (ID 3)
//     4, // Link 4's parent is Link 3 (ID 4)
//     5, // Link 5's parent is Link 4 (ID 5)
//     6, // Gripper Base's parent is Link 5 (ID 6)
//     // 7, // Link 7's parent is Gripper Base (ID 7)
//     // 7  // Link 8's parent is also Gripper Base (ID 7)
// }};

// --- Builder Initialization Function Template ---
/**
 * @brief Creates a RigidBodyTree::Builder pre-populated with the Piper robotic arm model data.
 * 
 * @tparam T The floating-point precision (e.g., float, double).
 * @return RigidBodyTree<T, NumLink, NumDof>::Builder A builder instance for the Piper model.
 */
template<typename T>
RigidBodyTree<T, NumLink, NumDof>::Builder
getPiperArmModelBuilder(const std::array<T, 3>& base_position, const std::array<T, 3>& base_orientation)
{
    typename RigidBodyTree<T, NumLink, NumDof>::Builder builder{};

    builder.root_id = 0;

    builder.base_position = base_position;
    builder.base_orientation = base_orientation;

    builder.mass =
    {{
        0.71, 1.17, 0.5, 0.38,
        0.383, 0.007, 0.45,
        // 0.025, 0.025
    }};

    builder.initial_position =
    {{
        {{0, 0, 0.123}},
        {{0, 0, 0}},
        {{0.28503, 0, 0}},
        {{-0.021984, -0.25075, 0}},
        {{0, 0, 0}},
        {{0, -0.091, 0}},
        {{0, 0, 0}},
        // {{0, 0, 0.1358}},
        // {{0, 0, 0.1358}}
    }};

    builder.initial_orientation =
    {{
        {{0, 0, 0}},
        {{M_PI / 2.0, -0.1359, -M_PI}},
        {{0, 0, -1.7939}},
        {{M_PI / 2.0, 0, 0}},
        {{-M_PI / 2.0, 0, 0}},
        {{M_PI / 2.0, 0, 0}},
        {{0, 0, 0}},
        // {{M_PI / 2.0, 0, 0}},
        // {{M_PI / 2.0, 0, -M_PI}}
    }};

    builder.com =
    {{
        {{0.000121504734057468, 0.000104632162460536, -0.00438597309559853}},
        {{0.198666145229743, -0.010926924140076, 0.00142121714502687}},
        {{-0.0202737662122021, -0.133914995944595, -0.000458682652737356}},
        {{-9.66635791618542E-05, 0.000876064475651083, -0.00496880904640868}},
        {{-4.10554118924211E-05, -0.0566486692356075, -0.0037205791677906}},
        {{-8.82590762930069E-05, 9.0598378529832E-06, -0.002}},
        {{-0.000183807162235591, 8.05033155577911E-05, 0.0321436689908876}},
        // {{0.00065123185041968, -0.0491929869131989, 0.00972258769184025}},
        // {{0.00065123185041968, -0.0491929869131989, 0.00972258769184025}}
    }};

    builder.inertia =
    {{
        {{0.00048916, -0.00000036, -0.00000224, 0.00040472, -0.00000242, 0.00043982}}, // Link 1
        {{0.00116918, -0.00180037, 0.00025146, 0.06785384, -0.00000455, 0.06774489}},  // Link 2
        {{0.01361711, 0.00165794, -0.00000048, 0.00045024, -0.00000045, 0.01380322}},  // Link 3
        {{0.00018501, 0.00000054, 0.00000120, 0.00018965, -0.00000841, 0.00015484}},   // Link 4
        {{0.00166169, 0.00000006, -0.00000007, 0.00018510, 0.00001026, 0.00164321}},   // Link 5
        {{5.73015540542155E-07, -1.98305403089247E-22, -7.2791893904596E-23, 5.73015540542155E-07, -3.4146026640245E-24, 1.06738869138926E-06}}, // Link 6
        {{0.00092934, 0.00000034, -0.00000738, 0.00071447, 0.00000005, 0.00039442}},   // Gripper Base
        // {{0.00007371, -0.00000113, 0.00000021, 0.00000781, -0.00001372, 0.0000747}},   // Link 7
        // {{0.00007371, -0.00000113, 0.00000021, 0.00000781, -0.00001372, 0.0000747}}    // Link 8
    }};

    builder.joint_type =
    {{
        JointType::JointTypeRevolute,
        JointType::JointTypeRevolute,
        JointType::JointTypeRevolute,
        JointType::JointTypeRevolute,
        JointType::JointTypeRevolute,
        JointType::JointTypeRevolute,
        JointType::JointTypeFixed,
        // JointType::JointTypeFixed,   // Set finger joint fixed to avoid computation complexity.
        // JointType::JointTypeFixed
    }};

    builder.joint_axis =
    {{
        {{0, 0, 1}}, // Joint 1
        {{0, 0, 1}}, // Joint 2
        {{0, 0, 1}}, // Joint 3
        {{0, 0, 1}}, // Joint 4
        {{0, 0, 1}}, // Joint 5
        {{0, 0, 1}}, // Joint 6
        {{0, 0, 1}}, // Gripper Base Joint
        // {{0, 0, 1}}, // Joint 7
        // {{0, 0, 1}}  // Joint 8
    }};

    builder.joint_limits =
    {{
        {{-2.618, 2.618}},   // Joint 1
        {{0, M_PI}},         // Joint 2
        {{-2.967, 0}},       // Joint 3
        {{-1.745, 1.745}},   // Joint 4
        {{-1.22, 1.22}},     // Joint 5
        {{-2.0944, 2.0944}}, // Joint 6
        {{0, 0}},            // Gripper Base Joint
        // {{0, 0.035}},        // Joint 7
        // {{-0.035, 0}}        // Joint 8
    }};

    builder.children_indices =
    {{
        {1},
        {2},
        {3},
        {4},
        {5},
        {6},
        {7,8},
        // {},
        // {}
    }};

    // for (std::size_t i = 0; i < NumLink; ++i)
    // {
    //     builder.children_indices[i].clear();
    // }

    // // Populate children lists based on the internal piper_parent_index array
    // for (std::size_t child_idx = 0; child_idx < piper_parent_index.size(); ++child_idx)
    // {
    //     std::size_t parent_id = piper_parent_index[child_idx]; // This is the 1-based ID of the parent
    //     if (parent_id > 0) { // If it has a parent (not the root)
    //         std::size_t parent_idx = parent_id - 1; // Convert 1-based ID to 0-based index
    //         if (parent_idx < NumLink) // Safety check
    //         {
    //             // Add the child's 0-based index to the parent's children list
    //             // The Builder's buildRigidBodyById expects 0-based index + 1 for ID.
    //             // But children_indices stores the direct 0-based indices of children.
    //             builder.children_indices[parent_idx].push_back(child_idx);
    //         }
    //         // Note: In a robust implementation, you might want to handle the error case
    //         // where parent_idx >= NumJoint, indicating invalid parent_index data.
    //     }
    // }

    return builder;
}
} // namespace PiperArmModel
#endif // __PIPER_MODEL_HPP__