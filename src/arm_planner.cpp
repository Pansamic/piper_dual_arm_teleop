/**
 * @file arm_planner.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief AgileX Piper robotic arm trajectory planner.
 * @version 0.1
 * @date 2025-06-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <arm_planner.h>

ArmPlanner::ArmPlanner(
    RingBuffer<JointState>& left_arm_target_waypoint_buffer,
    RingBuffer<JointState>& right_arm_target_waypoint_buffer,
    RingBuffer<JointState>& left_arm_trajectory_buffer,
    RingBuffer<JointState>& right_arm_trajectory_buffer,
    size_t freq_plan, size_t freq_ctrl):
    dt_plan_(1.0/static_cast<double>(freq_plan)), dt_ctrl_(1.0/static_cast<double>(freq_ctrl)),
    traj_len_(freq_ctrl/freq_plan),
    left_arm_trajectory_buffer_(left_arm_trajectory_buffer),
    right_arm_trajectory_buffer_(right_arm_trajectory_buffer),
    left_arm_target_joint_state_buffer_(left_arm_target_waypoint_buffer),
    right_arm_target_joint_state_buffer_(right_arm_target_waypoint_buffer),
    left_arm_last_target_joint_state_(Eigen::Vector<double, ArmModel::num_dof_>::Zero()),
    right_arm_last_target_joint_state_(Eigen::Vector<double, ArmModel::num_dof_>::Zero()),
    plan_thread_(std::thread(&ArmPlanner::threadPlan, this)), running_(true)
{

}

ArmPlanner::~ArmPlanner()
{
    this->running_ = false;
    if ( this->plan_thread_.joinable() )
    {
        plan_thread_.join();  // Ensure thread finishes before destruction
    }
}

// void ArmPlanner::interpolateLinear(RingBuffer<JointState>& traj_buf, const std::array<JointState, plan_waypoint_amount_>& waypoints)
// {
//     constexpr size_t dof = ArmModel::num_dof_;
//     const size_t num_segments = plan_waypoint_amount_ - 1;
//     const size_t total_samples = traj_len_;
//     const double dt = dt_ctrl_;
//     const double T = dt_plan_ / double(num_segments);
//     const size_t samples_per_segment = total_samples / num_segments;

//     for (size_t seg = 0; seg < num_segments; ++seg)
//     {
//         const Eigen::VectorXd& q0 = waypoints[seg].joint_pos;
//         const Eigen::VectorXd& q1 = waypoints[seg + 1].joint_pos;

//         for (size_t s = 0; s < samples_per_segment; ++s)
//         {
//             double alpha = double(s) / double(samples_per_segment);
//             Eigen::VectorXd q = (1.0 - alpha) * q0 + alpha * q1;
//             Eigen::VectorXd v = (q1 - q0) / T;

//             JointState joint_state;
//             joint_state.joint_pos = q;
//             joint_state.joint_vel = v;

//             traj_buf.push(joint_state);
//         }
//     }
// }

// void ArmPlanner::interpolateBSpline(RingBuffer<JointState>& traj_buf, const std::array<JointState, plan_waypoint_amount_>& waypoints)
// {
//     constexpr size_t dof = ArmModel::num_dof_;
//     const int degree = 3;  // Cubic B-spline
//     const int n = plan_waypoint_amount_ - 1;  // Number of control points - 1
//     const int k = degree + 1;  // order
//     const double t_start = 0.0;
//     const double t_end = 1.0;
//     const size_t total_samples = traj_len_;

//     // Knot vector: uniform, clamped
//     std::vector<double> U(n + k + 1);
//     int num_knots = U.size();

//     for (int i = 0; i < num_knots; ++i)
//     {
//         if (i < k) U[i] = 0.0;
//         else if (i <= n) U[i] = double(i - degree) / double(n - degree + 1);
//         else U[i] = 1.0;
//     }

//     std::function<double(int, int, double)> CoxDeBoor = [&](int i, int d, double t) -> double
//     {
//         if (d == 0)
//         {
//             return (U[i] <= t && t < U[i + 1]) ? 1.0 : 0.0;
//         }

//         double denom1 = U[i + d] - U[i];
//         double denom2 = U[i + d + 1] - U[i + 1];
//         double term1 = 0.0, term2 = 0.0;

//         if (denom1 > 1e-8)
//             term1 = (t - U[i]) / denom1 * CoxDeBoor(i, d - 1, t);
//         if (denom2 > 1e-8)
//             term2 = (U[i + d + 1] - t) / denom2 * CoxDeBoor(i + 1, d - 1, t);

//         return term1 + term2;
//     };

//     auto CoxDeBoorDerivative = [&](int i, int d, double t) -> double
//     {
//         if (d == 0) return 0.0;

//         double denom1 = U[i + d] - U[i];
//         double denom2 = U[i + d + 1] - U[i + 1];

//         double term1 = 0.0, term2 = 0.0;
//         if (denom1 > 1e-8)
//             term1 = 1.0 / denom1 * CoxDeBoor(i, d - 1, t);
//         if (denom2 > 1e-8)
//             term2 = 1.0 / denom2 * CoxDeBoor(i + 1, d - 1, t);

//         return d * (term1 - term2);
//     };

//     for (size_t s = 0; s < total_samples; ++s)
//     {
//         double tau = t_start + (t_end - t_start) * (double(s) / (total_samples - 1));

//         JointState joint_state;
//         joint_state.joint_pos.setZero();
//         joint_state.joint_vel.setZero();

//         for (int i = 0; i <= n; ++i)
//         {
//             double Ni = CoxDeBoor(i, degree, tau);
//             double dNi_dtau = CoxDeBoorDerivative(i, degree, tau);

//             joint_state.joint_pos += Ni * waypoints[i].joint_pos;
//             joint_state.joint_vel += dNi_dtau * waypoints[i].joint_pos;
//         }

//         // Reparameterize velocity to real time (t ∈ [0, dt_plan_])
//         joint_state.joint_vel *= 1.0 / dt_plan_;

//         traj_buf.push(joint_state);
//     }
// }

// void ArmPlanner::interpolateQuinticPolynomial(RingBuffer<JointState>& traj_buf, const std::array<JointState, plan_waypoint_amount_>& waypoints)
// {
//     constexpr size_t dof = ArmModel::num_dof_;
//     const size_t num_segments = plan_waypoint_amount_ - 1;
//     const size_t total_samples = traj_len_;
//     const double dt = dt_ctrl_;
//     const double T = dt_plan_ / double(num_segments);
//     const size_t samples_per_segment = total_samples / num_segments;

//     Eigen::Matrix<double, 6, 6> M;
//     double T2 = T * T;
//     double T3 = T2 * T;
//     double T4 = T3 * T;
//     double T5 = T4 * T;

//     M <<
//         1,  0,   0,    0,     0,      0,
//         0,  1,   0,    0,     0,      0,
//         0,  0,   2,    0,     0,      0,
//         1,  T,   T2,   T3,    T4,     T5,
//         0,  1, 2*T,  3*T2,  4*T3,   5*T4,
//         0,  0,   2,   6*T, 12*T2,  20*T3;

//     Eigen::Matrix<double, 6, 6> M_inv = M.inverse();

//     for (size_t seg = 0; seg < num_segments; ++seg)
//     {
//         const auto& q0 = waypoints[seg].joint_pos;
//         const auto& v0 = waypoints[seg].joint_vel;
//         const auto& qT = waypoints[seg + 1].joint_pos;
//         const auto& vT = waypoints[seg + 1].joint_vel;

//         Eigen::Matrix<double, dof, 6> coeffs;

//         for (size_t j = 0; j < dof; ++j)
//         {
//             Eigen::Matrix<double, 6, 1> b;
//             b << q0[j], v0[j], 0.0, qT[j], vT[j], 0.0;
//             coeffs.row(j) = (M_inv * b).transpose();
//         }

//         for (size_t s = 0; s < samples_per_segment; ++s)
//         {
//             double t = s * dt;
//             Eigen::VectorXd q(dof), v(dof);

//             for (size_t j = 0; j < dof; ++j)
//             {
//                 const auto& a = coeffs.row(j);
//                 q[j] = a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t + a[4]*t*t*t*t + a[5]*t*t*t*t*t;
//                 v[j] = a[1] + 2*a[2]*t + 3*a[3]*t*t + 4*a[4]*t*t*t + 5*a[5]*t*t*t*t;
//             }

//             JointState wp;
//             wp.joint_pos = q;
//             wp.joint_vel = v;

//             traj_buf.push(wp);
//         }
//     }
// }

void ArmPlanner::planDualArmLinear(
    std::array<JointState, plan_waypoint_amount_>& left_arm_plan_waypoints,
    std::array<JointState, plan_waypoint_amount_>& right_arm_plan_waypoints,
    const JointState& left_arm_begin,
    const JointState& left_arm_end,
    const JointState& right_arm_begin,
    const JointState& right_arm_end)
{
    // fill internal_waypoints_[0..N-1]
    for (size_t i = 0; i < this->plan_waypoint_amount_; ++i)
    {
        double alpha = double(i+1) / (this->plan_waypoint_amount_ + 1);

        left_arm_plan_waypoints[i].joint_pos = left_arm_begin.joint_pos + alpha * (left_arm_end.joint_pos - left_arm_begin.joint_pos);
        right_arm_plan_waypoints[i].joint_pos = right_arm_begin.joint_pos + alpha * (right_arm_end.joint_pos - right_arm_begin.joint_pos);
        
        /* zero velocities/accelerations here, to be filled by interpolation */
        left_arm_plan_waypoints[i].joint_vel.setZero();
        right_arm_plan_waypoints[i].joint_vel.setZero();

        /* uncomment the following line if joint acceleration exists in waypoint struct */
        left_arm_plan_waypoints[i].joint_acc.setZero();
        right_arm_plan_waypoints[i].joint_acc.setZero();
    }
}

void ArmPlanner::threadPlan()
{
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

    JointState left_arm_target_joint_state;
    JointState right_arm_target_joint_state;

    struct timespec wakeup_time = {0,0};
    static struct timespec cycletime = {0, static_cast<long>(this->dt_plan_ * 1e9)};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    while ( this->running_ )
    {
        increase_time_spec(&wakeup_time, &cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        /* Use non-blocking way `try_pop()` to avoid waiting.
         * If fails to pop, the target will remain the same. */
        this->left_arm_target_joint_state_buffer_.try_pop(left_arm_target_joint_state);
        this->right_arm_target_joint_state_buffer_.try_pop(right_arm_target_joint_state);

        /* Use linear plan to generate internal waypoints */
        this->planDualArmLinear(
            left_arm_plan_waypoints, right_arm_plan_waypoints,
            this->left_arm_last_target_joint_state_, left_arm_target_joint_state,
            this->right_arm_last_target_joint_state_, right_arm_target_joint_state);
        
    }
}