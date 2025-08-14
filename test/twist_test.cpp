#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "skew.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp"

const double ERROR_BOUND = 1e-6;

// Test TwistJoint Constructor with axis w and axis position q
// - Given:
// - - - q = x-direction
// - - - w = z axis
// - Expect twist translational comp to be in negative y direction at origin
// - Expect twist rotational comp to be z axis
TEST(TwistJoint_test, TestConstructor_qw1)
{
    Eigen::Vector3d q_test {1,0,0};
    Eigen::Vector3d z_axis {0,0,1};
    TwistType expected_twist{0, -1, 0, 0, 0, 1};

    TwistJoint joint(q_test, z_axis);
    ASSERT_EQ(expected_twist, joint.GetTwist());
}

// Test TwistJoint Constructor with axis w and axis position q
// - Given:
// - - - q = y-direction
// - - - w = z axis
// - Expect twist translational comp to be in positive x direction at origin
// - Expect twist rotational comp to be z axis
TEST(TwistJoint_test, TestConstructor_qw2)
{
    Eigen::Vector3d q_test {0, 1, 0};
    Eigen::Vector3d z_axis {0,0,1};
    TwistType expected_twist{1, 0, 0, 0, 0, 1};

    TwistJoint joint(q_test, z_axis);
    ASSERT_EQ(expected_twist, joint.GetTwist());
}

// Test TwistJoint Constructor with axis w and axis position q
// - Given:
// - - - q = origin
// - - - w = z axis
// - Expect twist translational comp to be zero (axis located at origin)
// - Expect twist rotational comp to be z axis
TEST(TwistJoint_test, TestConstructor_qw3)
{
    Eigen::Vector3d q_test {0, 0, 0};
    Eigen::Vector3d z_axis {0,0,1};
    TwistType expected_twist{0, 0, 0, 0, 0, 1};

    TwistJoint joint(q_test, z_axis);
    ASSERT_EQ(expected_twist, joint.GetTwist());
}

// Test TwistJoint Constructor with axis w and axis position q
// - Given:
// - - - q = x-direction
// - - - w = z axis stretched
// - Expect twist translational comp to be in negative y direction at origin with appropriate mag
// - Expect twist rotational comp to be z axis and normalized
TEST(TwistJoint_test, TestConstructor_qw4)
{
    Eigen::Vector3d q_test {1, 0, 0};
    Eigen::Vector3d z_axis {0,0,5};
    TwistType expected_twist{0, -1, 0, 0, 0, 1};

    TwistJoint joint(q_test, z_axis);
    ASSERT_EQ(expected_twist, joint.GetTwist());
}

// Test TwistJoint Constructor with axis w and axis position q
// - Given:
// - - - q = z-direction
// - - - w = x axis
// - Expect twist translational comp to be in positive y direction at origin with appropriate mag
// - Expect twist rotational comp to be x axis
TEST(TwistJoint_test, TestConstructor_qw5)
{
    Eigen::Vector3d q_test {0, 0, 5};
    Eigen::Vector3d x_axis {1,0,0};
    TwistType expected_twist{0, 5, 0, 1, 0, 0};

    TwistJoint joint(q_test, x_axis);
    ASSERT_EQ(expected_twist, joint.GetTwist());
}

// Test TwistJoint TwistExponential method with 90d rotation about z axis
// - Given: Joint axis is located on x-axis with orientation of z-axis
// - Expect: 
// - - - Rotation matrix of transform depicts 3d rotation about z axis by 90 degrees
// - - - Translation vector moves origin below twist joint
TEST(TwistJoint_test, TestExponential_zaxis_90d)
{
    Eigen::Vector3d q_test {1,0,0};
    Eigen::Vector3d z_axis {0,0,1};

    TwistJoint joint(q_test, z_axis);

    Eigen::Matrix4d expected_transform {
        {0, -1, 0, 1},
        {1, 0, 0, -1},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    Eigen::Matrix4d calc_transform = joint.TwistExponential(M_PI/2);

    // Iterate through matrix to check each element
    for(uint row = 0; row < 4; row++)
    {
        for(uint col = 0; col < 4; col++)
        {
            ASSERT_NEAR(expected_transform(row, col), calc_transform(row, col), ERROR_BOUND);
        }
    }
}

// Test TwistJoint TwistExponential method with 180d rotation about z axis
// - Given: Joint axis is located on x-axis with orientation of z-axis
// - Expect: 
// - - - Rotation matrix of transform depicts 3d rotation about z axis by 180 degrees
// - - - Translation vector moves origin to the right of joint
TEST(TwistJoint_test, TestExponential_zaxis_180d)
{
    Eigen::Vector3d q_test {1,0,0};
    Eigen::Vector3d z_axis {0,0,1};

    TwistJoint joint(q_test, z_axis);

    Eigen::Matrix4d expected_transform {
        {-1, 0, 0, 2},
        {0, -1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    Eigen::Matrix4d calc_transform = joint.TwistExponential(M_PI);

    // Iterate through matrix to check each element
    for(uint row = 0; row < 4; row++)
    {
        for(uint col = 0; col < 4; col++)
        {
            ASSERT_NEAR(expected_transform(row, col), calc_transform(row, col), ERROR_BOUND);
        }
    }
}

// Test TwistJoint TwistExponential method with 90d rotation about x axis
// - Given: Joint axis is located on z-axis with orientation of negative x-axis
// - Expect: 
// - - - Rotation matrix of transform depicts 3d rotation about negative x axis by 90 degrees
// - - - Translation vector moves origin down y axis and up z axis
TEST(TwistJoint, TestExponential_xaxis_90d)
{
    Eigen::Vector3d q_test {0,0,5};
    Eigen::Vector3d x_axis {-1,0,0};

    TwistJoint joint(q_test, x_axis);

    Eigen::Matrix4d expected_transform {
        {1, 0, 0, 0},
        {0, 0, 1, -5},
        {0, -1, 0, 5},
        {0, 0, 0, 1}
    };

    Eigen::Matrix4d calc_transform = joint.TwistExponential(M_PI/2);

    // Iterate through matrix to check each element
    for(uint row = 0; row < 4; row++)
    {
        for(uint col = 0; col < 4; col++)
        {
            ASSERT_NEAR(expected_transform(row, col), calc_transform(row, col), ERROR_BOUND);
        }
    }
}