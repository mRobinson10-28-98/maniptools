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

    DualQuaternion a;
    DualQuaternion b;
    DualQuaternion c = a + b;
}