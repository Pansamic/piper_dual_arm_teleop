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


class ArmPlanner
{
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    using Duration = std::chrono::steady_clock::duration;

    template<typename T, std::size_t NumDof, std::size_t WayPointAmount>
    static std::tuple<std::vector<TimePoint>, Eigen::Matrix<T, NumDof, WayPointAmount>>
    plan(const TimePoint& start_time, const double min_duration, const Eigen::Vector<T, NumDof>& start, const Eigen::Vector<T, NumDof>& end);
};

class LinearArmPlanner : ArmPlanner
{
private:
    using TimePoint = ArmPlanner::TimePoint;
    using Duration = ArmPlanner::Duration;
public:

    template<typename T, std::size_t NumDof, std::size_t WayPointAmount>
    static std::tuple<std::array<TimePoint, WayPointAmount>, Eigen::Matrix<T, NumDof, WayPointAmount>>
    plan(const TimePoint& start_time, const T min_duration, const Eigen::Vector<T, NumDof>& start, const Eigen::Vector<T, NumDof>& end)
    {
        Eigen::Matrix<T, NumDof, WayPointAmount> trajectory;
        std::array<TimePoint, WayPointAmount> time_point;
        /* expand trajectory duration with joint position L2 norm.
         * Avoid abrupt arm movement if a quick huge target joint position is coming. */
        T duration = min_duration + CONFIG_TRAJECTORY_DAMPING_COEF * (end - start).squaredNorm();
        /* Assume "n" is the waypoint amount, then there are "n-1" portions in this trajectory.
         * The time duration of each segment is supposed to be [n-1, n-2, n-3, ... , 1].
         * So the sum of them is n*(n-1)/2. Let's say the weight is n*(n-1)/2. */
        static constexpr T weight = static_cast<T>(WayPointAmount * (WayPointAmount - 1) / 2);
        T portion = duration / weight;
        Duration time_portion = std::chrono::duration_cast<Duration>(std::chrono::duration<T>(portion));
        Eigen::Vector<T, NumDof> step_position = (end - start) / static_cast<T>(WayPointAmount - 1);
        TimePoint current_time = start_time;
        for (std::size_t i = 0 ; i < WayPointAmount; ++i)
        {
            trajectory.col(i) = (start + step_position * i);
            time_point[i] = current_time;
            current_time += time_portion * ( WayPointAmount - 1 - i );
        }
        return std::make_tuple(time_point, trajectory);
    }
};