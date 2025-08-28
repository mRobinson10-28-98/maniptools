#include <array>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "Common.hpp"
#include "JointControllerStepResponse.hpp"
#include "Manip5Dof.hpp"
#include "SimClock.hpp"
#include "TwistJoint.hpp"

Eigen::Quaterniond RotToQuat(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q(R);
    return q;
}

Eigen::AngleAxisd QuatToAngleAxis(Eigen::Quaterniond q)
{
    Eigen::AngleAxisd A(q);
    return A;
}

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

    Eigen::Matrix3d R_circle_start = g_circle_start.block(0,0,3,3);
    Eigen::Quaterniond quat_circle_start = RotToQuat(R_circle_start);

    // TODO - iteratively until manip pose is close to g_circle_start
    // 1. SLERP/ScLERP between g_0 and circle start using small interpolatory value (0.1 perhaps)
    // - - - translate poses from SE(3) to Eigen::Quaternion and use slerp function
    // 2. Determine instanstanious twist between current pose and next pose
    // - - - translational compenent is just position b - position a
    // - - - rotational component is rotation matrix to angle-axis
    // 3. Calculate joint velocities for that twist using psuedo inverse
    // 4. Integrate joint velocities by sim frequency (1ms) to calculate next frame joint positions
    // 5. Command joint positions

    double slerp_t = 0.1;

    // Current pose
    Eigen::Matrix4d g_t = manip.GetPose();
    Eigen::Matrix3d R_t = g_t.block(0,0,3,3);
    Eigen::Quaterniond quat_t = RotToQuat(R_t);

    // Pose diff: g_t * g_diff = g_circle_start; g_diff = g_t^(-1) * g_circle_start
    Eigen::Matrix4d g_diff = g_t.inverse() * g_circle_start;
    Eigen::Matrix3d R_diff = g_diff.block(0, 0, 3, 3);
    Eigen::Vector3d delta_p = g_diff.block(0,3,3,1);
    Eigen::Vector3d delta_p_lerp = delta_p * slerp_t;
    double delta_p_mag = delta_p.norm();

    // Quats
    Eigen::Quaterniond quat_slerp = quat_t.slerp(slerp_t, quat_circle_start);
    Eigen::AngleAxisd A_slerp = QuatToAngleAxis(quat_slerp);

    TwistType twist_t;
    twist_t.head(3) = delta_p_lerp;
    twist_t.tail(3) = A_slerp.axis();

    // Joint commands
    Eigen::VectorXd q, q_dot;
    Eigen::MatrixXd J_SA = manip.GetSAJacobian();
    // pseudo inverse
    Eigen::MatrixXd J_SA_inv = PseudoInverse(J_SA);

    std::cout << "Starting manip pose: \n" << g_t << "\n\n";
    std::cout << "Starting pose diff: \n" << g_diff << "\n\n";
    std::cout << "Starting position offset: \n" << delta_p << "\n\n";
    std::cout << "Starting position offset mag: \n" << delta_p_mag << "\n\n";


    // Initialize manipulator loop
    // while(delta_p_mag >= 0.001);
    // {
        // Step model
        manip.StepModel();
        g_t = manip.GetPose();
        J_SA = manip.GetSAJacobian();
        J_SA_inv = PseudoInverse(J_SA);

        std::cout << "Jacobian: \n" << J_SA << "\n\n";
        std::cout << "Jacobian inverse 1: \n" << J_SA * J_SA.transpose() << "\n\n";
        std::cout << "Jacobian inverse 2: \n" << (J_SA * J_SA.transpose()).inverse() << "\n\n";

        // 1. SLERP/ScLERP between g_0 and circle start using small interpolatory value (0.1 perhaps)
        // - - - translate poses from SE(3) to Eigen::Quaternion and use slerp function for orientation 
        // - - - and lerp for position
        g_diff = g_t.inverse() * g_circle_start;
        R_diff = g_diff.block(0, 0, 3, 3);
        delta_p = g_diff.block(0,3,3,1);
        delta_p_lerp = delta_p * slerp_t;
        delta_p_mag = delta_p.norm();

        R_t = g_t.block(0,0,3,3);
        quat_t = RotToQuat(R_t);
        quat_slerp = quat_t.slerp(slerp_t, quat_circle_start);

        // 2. Determine instanstanious twist between current pose and next pose
        // - - - translational compenent is just position b - position a
        // - - - rotational component is rotation matrix to angle-axis
        A_slerp = QuatToAngleAxis(quat_slerp);
        twist_t.head(3) = delta_p_lerp;
        twist_t.tail(3) = A_slerp.axis();

        std::cout << "Twist: \n" << twist_t << "\n\n";

        // 3. Calculate joint velocities for that twist using psuedo inverse
        q_dot = J_SA_inv * twist_t;


        // 4. Integrate joint velocities by sim frequency (1ms) to calculate next frame joint positions
        q = manip.GetTheta() + q_dot / sim_clock.GetSimFreq();

        std::cout << "q_dot: \n" << q_dot << "\n\n";
        std::cout << "next q: \n" << twist_t << "\n\n";

        // 5. Command joint positions

        // 6. calculate while condition
        delta_p_mag = delta_p.norm();
    //}
}