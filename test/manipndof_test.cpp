#include <gtest/gtest.h>

#include "skew.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp"

const double ERROR_BOUND = 1e-6;

TEST(Manip5Dof_test, PositionCheck)
{
    double x, y, z, total_length;
    JointControllerSnap myJC;
    std::array<double, 5> link_lengths {5,4,3,2,1};

    total_length = 0;
    for (double l: link_lengths)
    {
        total_length += l;
    }

    Manip5Dof myManip(myJC, link_lengths);

    std::vector<double> q {0, 0, 0, 0, 0};
    myManip.CommandJointConfig(q);
    myManip.StepJointController();
    Eigen::Matrix4d fk = myManip.Fk();

    x = fk(0, 3);
    y = fk(1, 3);
    z = fk(2, 3);

    ASSERT_NEAR(x, total_length, ERROR_BOUND);
    ASSERT_NEAR(y, 0, ERROR_BOUND);
    ASSERT_NEAR(z, 0, ERROR_BOUND);

    q.at(0) = M_PI;

    myManip.CommandJointConfig(q);
    myManip.StepJointController();
    fk = myManip.Fk();

    x = fk(0, 3);
    y = fk(1, 3);
    z = fk(2, 3);

    ASSERT_NEAR(x, -total_length, ERROR_BOUND);
    ASSERT_NEAR(y, 0, ERROR_BOUND);
    ASSERT_NEAR(z, 0, ERROR_BOUND);
}