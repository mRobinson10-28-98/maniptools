#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <thread>

#include "maniptools_include.hpp"

using namespace DualNumberTools;

// Get the desired velocity on the circle at time t
// Circle trace velocity = PI rad/s (complete circle in 2s)
TwistType CircleTrajectory(double t)
{
    double r = 2;

    // position = <r*cos(theta), r*sin(theta)
    // velocity is derivative
    double x = r * -sin(M_PI * t);
    double y = r * cos(M_PI * t);

    TwistType tw {x, y, 0, 0, 0, 0};
    return tw;
}

// 1ms time steps
const double SIM_CLOCK_FREQUENCY = 1000;

int main()
{
    // Initialize joint states to 0 for all (pos, vel, accel, effort)
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(5);
    JointControllerStepResponse joint_controller;
    joint_controller.Initialize(q_init, q_init, q_init, q_init);

    // Set clock frequency to 1000 Hz
    SimClock& sim_clock = SimClock::GetInstance();

    // Create manip model
    std::array<double, 5> link_lengths {5,4,3,2,1};
    Manip5Dof manip(joint_controller, link_lengths);

    // Setup
    Eigen::Vector<double, 5> q_i {M_PI/2, 0, 0, 0, 0};
    sim_clock.SetSimFreq(10);
    manip.CommandJointConfig(q_i);
    manip.StepModel();
    sim_clock.SetSimFreq(SIM_CLOCK_FREQUENCY);

    // Desired start position
    Eigen::Matrix4d g_circle_start {
        {1, 0, 0, 7},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    double slerp_t = 0.1;

    // Current pose
    Eigen::Matrix4d g_t = manip.GetPose();
    DualQuaternion g_t_quat(g_t);
    Eigen::Matrix3d R_t = g_t_quat.RotationMatrix();
    Eigen::Vector3d p_t = g_t_quat.PositionVector();

    // Pose diff: g_t * g_diff = g_circle_start; g_diff = g_t^(-1) * g_circle_start
    Eigen::Matrix4d g_diff = g_t.inverse() * g_circle_start;
    DualQuaternion g_diff_quat(g_diff);
    Eigen::Vector3d p_diff = g_diff_quat.PositionVector();
    double delta_p_mag = p_diff.norm();

    std::cout << "Starting manip pose: \n" << g_t << "\n\n";
    std::cout << "Starting pose diff: \n" << g_diff << "\n\n";
    std::cout << "Starting position offset: \n" << p_diff << "\n\n";
    std::cout << "Starting position offset mag: \n" << delta_p_mag << "\n\n";
    std::cout << "Jac: \n" << manip.GetSAJacobian() << "\n\n";

    std::cout << "Press enter to move to circle start:";
    std::cin.get();

    // 1ms period
    const auto period = std::chrono::milliseconds((int)(1000.0 / SIM_CLOCK_FREQUENCY));
    auto next_wakeup = std::chrono::steady_clock::now() + period;

    // Initialize manipulator loop
    while(delta_p_mag >= 0.001)
    {
        manip.StepModel();

        // Current pose
        g_t = manip.GetPose();
        g_t_quat = DualQuaternion(g_t);
        R_t = g_t_quat.RotationMatrix();
        p_t = g_t_quat.PositionVector();

        // Pose diff: g_t * g_diff = g_circle_start; g_diff = g_t^(-1) * g_circle_start
        g_diff = g_t.inverse() * g_circle_start;
        g_diff_quat = DualQuaternion(g_diff);
        p_diff = g_diff_quat.PositionVector();
        delta_p_mag = p_diff.norm();

        q_i = manip.CalculateJointConfigScLERP(g_circle_start, slerp_t);

        manip.CommandJointConfig(q_i);

        if( (int(sim_clock.GetSimTime() * 1000)) % 1000 == 0)
        {
            std::cout << "Pose: \n" << g_t << "\n\n";
            std::cout << "Pose circle: \n" << g_circle_start << "\n\n";
            // std::cout << "Jac: \n" << manip.GetSAJacobian() << "\n\n";
        }

        // real time
        std::this_thread::sleep_until(next_wakeup);
        next_wakeup += period;
    }

    std::cout << "Press enter to trace circle:";
    std::cin.get();

    // Trace circle poorly
    while(true)
    {
        manip.StepModel();

            // Current pose
        g_t = manip.GetPose();
        g_t_quat = DualQuaternion(g_t);
        R_t = g_t_quat.RotationMatrix();
        p_t = g_t_quat.PositionVector();


        TwistType twist_vel = CircleTrajectory(sim_clock.GetSimTime());
        Eigen::VectorXd theta_dot = PseudoInverse(manip.GetSAJacobian()) * twist_vel;
        manip.CommandJointVel(theta_dot);

        if( (int(sim_clock.GetSimTime() * 1000)) % 100 == 0)
        {
            std::cout << "Position: \n" << p_t << "\n\n";
        }
    }
}