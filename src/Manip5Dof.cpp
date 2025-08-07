#include <iostream>

#include "Manip5Dof.hpp"


Manip5Dof::Manip5Dof(I_JointController& jointController, std::array<double, 5> link_lengths): 
    ManipNDof(jointController)
{
    std::vector<TwistJoint> joint_twists;

    Eigen::Vector3d tmp_v;
    Eigen::Vector3d z_axis {0, 0, 1};

    double link_length_sum = 0;

    // Assume default joint config is straight across x-axis; 
    // therefore point vector to joint 'i' should be sum of all previous links in x-direction
    // NOTE: Initial joint should always be at origin in z-direction
    for (int i = 0; i < 5; i++)
    {
        tmp_v << link_length_sum, 0, 0;
        joint_twists.push_back(TwistJoint(tmp_v, z_axis));

        link_length_sum += link_lengths[i];

    }

    Eigen::Matrix4d g_0 = Eigen::Matrix4d::Identity();
    g_0.col(3).head(1) << link_length_sum;

    Initialize(joint_twists, g_0);
}
