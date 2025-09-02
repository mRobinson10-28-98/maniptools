#include <array>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "Common.hpp"
#include "DualNumberTools.hpp"
#include "JointControllerStepResponse.hpp"
#include "Manip5Dof.hpp"
#include "SimClock.hpp"
#include "TwistJoint.hpp"

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

int main()
{
    // Initialize joint states to 0 for all (pos, vel, accel, effort)
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(5);
    JointControllerStepResponse joint_controller;
    joint_controller.Initialize(q_init, q_init, q_init, q_init);

    // Set clock frequency to 1000 Hz
    SimClock& sim_clock = SimClock::GetInstance();
    sim_clock.SetSimFreq(1000);

    // Create manip model
    std::array<double, 5> link_lengths {5,4,3,2,1};
    Manip5Dof manip(joint_controller, link_lengths);
    manip.StepModel();

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


    // Initialize manipulator loop
    while(delta_p_mag >= 0.001)
    {
        manip.CommandJointConfigScLERP(g_circle_start, slerp_t);
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

        if( (int(sim_clock.GetSimTime() * 1000)) % 1000 == 0)
        {
            std::cout << "Pose Diff: \n" << g_diff << "\n\n";
        }
    }
}