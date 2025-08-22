#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "skew.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp" 
// #include "JointControllerMock.hpp"

const double ERROR_BOUND = 1e-6;

class Manip5DofTest : public ::testing::Test
{
protected:
    // Configurable for test
    // JointControllerMock mJcMock;
    JointControllerSnap mJcMock;
    std::array<double, 5> mLinkLengths {5, 4, 3, 2, 1};

    // Class under test
    std::unique_ptr<Manip5Dof> mManip;

    // Other props
    double mTotalLength;

    // Setup test object
    void SetUp() override 
    {
        mManip = std::make_unique<Manip5Dof>(mJcMock, mLinkLengths);
        mTotalLength = 0;
        for (double l: mLinkLengths)
        {
            mTotalLength += l;
        }

    }

    void TearDown() override
    {

    }
};

// Test1 - at null joint config, check forward kinematics transform
TEST_F(Manip5DofTest, TestForwardKinematics1)
{
    std::vector<double> q {0, 0, 0, 0, 0};
    mManip->CommandJointConfig(q);
    mManip->StepModel();
    Eigen::Matrix4d fk_calculated = mManip->GetPose();

    Eigen::Matrix4d fk_expected {
        {1, 0, 0, mTotalLength},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // Iterate through matrix to check each element
    for(uint row = 0; row < 4; row++)
    {
        for(uint col = 0; col < 4; col++)
        {
            ASSERT_NEAR(fk_calculated(row, col), fk_expected(row, col), ERROR_BOUND);
        }
    }
}

// Test2 - rotate j0 180d and check forward kinematics transform
TEST_F(Manip5DofTest, TestForwardKinematics2)
{
    std::vector<double> q {M_PI, 0, 0, 0, 0};
    mManip->CommandJointConfig(q);
    mManip->StepModel();
    Eigen::Matrix4d fk_calculated = mManip->GetPose();

    Eigen::Matrix4d fk_expected {
        {-1, 0, 0, -mTotalLength},
        {0, -1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    // Iterate through matrix to check each element
    for(uint row = 0; row < 4; row++)
    {
        for(uint col = 0; col < 4; col++)
        {
            ASSERT_NEAR(fk_calculated(row, col), fk_expected(row, col), ERROR_BOUND);
        }
    }
}

TEST_F(Manip5DofTest, TestForwardDifferentialKinematics1)
{
    std::vector<double> q {M_PI, 0, 0, 0, 0};
    mManip->CommandJointConfig(q);
    mManip->StepModel();
    Eigen::Matrix4d fk_calculated = mManip->GetPose();
}