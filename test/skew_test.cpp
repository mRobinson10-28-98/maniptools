#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Common.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp"

const double ERROR_BOUND = 1e-6;

TEST(skew3d_test, Test1)
{
    Eigen::Vector3d test_vector {1,2,-3};

    Eigen::Matrix3d skewed_vector {
        {0, 3, 2},
        {-3, 0, -1},
        {-2, 1, 0}
    };

    ASSERT_EQ(skewed_vector, skew3d(test_vector));
}