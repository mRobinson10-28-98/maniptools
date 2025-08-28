#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "Common.hpp"
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
        // Initialize joints to 0s (pos, vel, accel, effort)
        Eigen::VectorXd v = Eigen::VectorXd::Zero(5);
        mJcMock.Initialize(v, v, v, v);

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
    // Set sim clock to very high frequency so joint pos doesnt change much per step
    SimClock& c = SimClock::GetInstance();
    c.SetSimFreq(1000);

    // In null config, twist should have translational component in pos y direction
    std::vector<double> q_dot {0, 0, 0, 0, M_PI};
    mManip->CommandJointVel(q_dot);
    mManip->StepModel();

    TwistType expected_twist {0, M_PI, 0, 0, 0, M_PI};
    TwistType dk_twist = mManip->GetTwist();

    for (int i = 0; i < 6; i++)
    {
        ASSERT_NEAR(dk_twist[i], expected_twist[i], ERROR_BOUND);
    }

    // Reset, then set joint vel to first joint
    std::vector<double> q {0, 0, 0, 0, 0};
    q_dot[0] = M_PI;
    q_dot[4] = 0;

    mManip->CommandJointConfig(q);
    mManip->StepModel();
    mManip->CommandJointVel(q_dot);
    mManip->StepModel();

    expected_twist[1] = mTotalLength * M_PI;
    dk_twist = mManip->GetTwist();

    for (int i = 0; i < 6; i++)
    {
        ASSERT_NEAR(dk_twist[i], expected_twist[i], ERROR_BOUND);
    }
}