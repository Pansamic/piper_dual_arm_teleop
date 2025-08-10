/**
 * @file arm_planner.hpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief Robotic arm trajectory planner.
 * @version 0.1
 * @date 2025-08-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <vector>
#include <tuple>
#include <chrono>
#include <concepts>
#include <Eigen/Core>
#include <config.h>

template<typename T>
concept FixedSizeEigenVector = 
    std::is_base_of_v<Eigen::Matrix<typename T::Scalar, T::RowsAtCompileTime, T::ColsAtCompileTime>, T> &&
    T::RowsAtCompileTime != Eigen::Dynamic && 
    T::ColsAtCompileTime != Eigen::Dynamic;

template<FixedSizeEigenVector JointPosition>
class ArmPlanner
{
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    using Duration = std::chrono::steady_clock::duration;

    static std::tuple<std::vector<TimePoint>, std::vector<JointPosition>>
    plan(const TimePoint& start_time, const double min_duration, const JointPosition& start, const JointPosition& end, std::size_t amount);
};

template<FixedSizeEigenVector JointPosition>
class LinearArmPlanner : ArmPlanner<JointPosition>
{
private:
    using Base = ArmPlanner<JointPosition>;
    using TimePoint = Base::TimePoint;
    using Duration = Base::Duration;
public:

    static std::tuple<std::vector<TimePoint>, std::vector<JointPosition>>
    plan(const TimePoint& start_time, const double min_duration, const JointPosition& start, const JointPosition& end, std::size_t amount)
    {
        std::vector<JointPosition> trajectory;
        std::vector<TimePoint> time_point;
        double duration = min_duration + CONFIG_TRAJECTORY_DECAY_COEF * (end - start).squaredNorm();
        double interval = duration / (amount - 1);
        Duration step_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(interval));

        for (std::size_t i = 0; i < amount; ++i)
        {
            trajectory.emplace_back(start + (end - start) * (i / (amount - 1)));
            time_point.emplace_back(start_time + step_duration * i);
        }

        return std::make_tuple(time_point, trajectory);
    }
};