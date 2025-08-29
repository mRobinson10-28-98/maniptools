#include <array>
#include <iostream>
#include <Eigen/Dense>

#include "Common.hpp"
#include "DualNumberTools.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp"
#include "SimClock.hpp"

using namespace DualNumberTools;

int main()
{
    Eigen::Quaterniond q1 {0, 1, 2, 3};
    Eigen::Quaterniond q2 {0, 1, 2, 3};

    Eigen::Quaterniond q_prod = q1 * q2.conjugate();
    std::cout << "Quat prod: \n" << q_prod << "\n\n";

    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::AngleAxisd aa(M_PI/2, z_axis);
    Eigen::Matrix3d rot(aa);
    Eigen::AngleAxisd aa_copy(rot);

    Eigen::Quaterniond q_rot(aa_copy);
    std::cout << "Quat from rot: \n" << q_rot << "\n\n";
    std::cout << "Rotation matrix: \n" << rot << "\n\n";

    std::cout << rot(0,1) << std::endl;
    std::cout << z_axis(1) << std::endl;
}