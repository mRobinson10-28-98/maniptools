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
    DualNumber d1(1,2);
    DualNumber d2(3,4);

    DualNumber d_sum = d1 + d2;
    DualNumber d_prod = d1 * d2;

    std::cout << d_sum.real << ", " << d_sum.dual << "\n\n";
    std::cout << d_prod.real << ", " << d_prod.dual << "\n\n";
}