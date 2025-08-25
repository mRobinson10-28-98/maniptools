#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "JointControllerStepResponse.hpp" 
#include "SimClock.hpp"
// #include "JointControllerMock.hpp"

const double ERROR_BOUND = 1e-2;

TEST(StepResponse_test, StepPositionFrom0)
{
    SimClock& c = SimClock::GetInstance();

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_dot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_ddot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(2);

    JointControllerStepResponse jc;
    jc.Initialize(v_zero, v_zero, v_zero, v_zero);

    // Setting tau = 0.1 forces joints to be 95% approached by t=0.3s
    jc.SetTau(0.1);
    

    // 10 Hz frequency; steps = 0.1s
    c.SetSimFreq(10);

    // Set command and step 3 times so t=0.3s
    Eigen::VectorXd t_c = Eigen::VectorXd::Zero(2);
    t_c << 1, 2;
    jc.PositionCommand(t_c);

    // Make sure joints are still 0 when clock has not been stepped
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_EQ(t, v_zero);
    ASSERT_EQ(t_dot, v_zero);
    ASSERT_EQ(t_ddot, v_zero);
    ASSERT_EQ(e, v_zero);

    // Step clock #1
    // ~63.2% done
    c.StepClock();
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t[0], 0.632, ERROR_BOUND);
    ASSERT_NEAR(t[1], 2*0.632, ERROR_BOUND);

    // Position of j1 jumped by 0.632 rad in 0.1s
    ASSERT_NEAR(t_dot[0], 6.32, ERROR_BOUND * 10);
    ASSERT_NEAR(t_dot[1], 2*6.32, ERROR_BOUND * 10);

    // Velocity of j1 jumped by 6.32 rad/s in 0.1s
    ASSERT_NEAR(t_ddot[0], 63.2, ERROR_BOUND * 100);
    ASSERT_NEAR(t_ddot[1], 2*63.2, ERROR_BOUND * 100);

    // Step clock #2 and 3
    // ~95% done
    c.StepClock();
    c.StepClock();
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t[0], 0.95, ERROR_BOUND);
    ASSERT_NEAR(t[1], 2*0.95, ERROR_BOUND);
}

TEST(StepResponse_test, StepVelocityFrom0)
{
    SimClock& c = SimClock::GetInstance();

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_dot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_ddot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(2);

    JointControllerStepResponse jc;
    jc.Initialize(v_zero, v_zero, v_zero, v_zero);

    // Setting tau = 0.1 forces joints to be 95% approached by t=0.3s
    jc.SetTau(0.1);
    
    // 10 Hz frequency; steps = 0.1s
    c.SetSimFreq(10);

    // Set command and step 3 times so t=0.3s
    Eigen::VectorXd t_c = Eigen::VectorXd::Zero(2);
    t_c << 1, 2;
    jc.VelocityCommand(t_c);

    // Make sure joints are still 0 when clock has not been stepped
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_EQ(t, v_zero);
    ASSERT_EQ(t_dot, v_zero);
    ASSERT_EQ(t_ddot, v_zero);
    ASSERT_EQ(e, v_zero);

    // Step clock #1
    // ~63.2% done
    c.StepClock();
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t_dot[0], 0.632, ERROR_BOUND);
    ASSERT_NEAR(t_dot[1], 2*0.632, ERROR_BOUND);

    // Velocity of j1 is 0.632 for 0.1s
    ASSERT_NEAR(t[0], 0.0632, ERROR_BOUND / 10);
    ASSERT_NEAR(t[1], 2*0.0632, ERROR_BOUND / 10);

    // Velocity of j1 jumped by 0.632 rad/s in 0.1s
    ASSERT_NEAR(t_ddot[0], 6.32, ERROR_BOUND * 10);
    ASSERT_NEAR(t_ddot[1], 2*6.32, ERROR_BOUND * 10);

    // Step clock #2 and 3
    // ~95% done
    c.StepClock();
    c.StepClock();
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t_dot[0], 0.95, ERROR_BOUND);
    ASSERT_NEAR(t_dot[1], 2*0.95, ERROR_BOUND);
}

TEST(StepResponse_test, StepPositionFromPreviousState)
{
    SimClock& c = SimClock::GetInstance();

    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_dot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd t_ddot = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(2);

    JointControllerStepResponse jc;
    jc.Initialize(v_zero, v_zero, v_zero, v_zero);

    // Setting tau = 0.1 forces joints to be 95% approached by t=0.3s
    jc.SetTau(0.1);

    // 10 Hz frequency; steps = 0.1s
    c.SetSimFreq(10);

    // Set command and step 3 times so t=0.3s
    Eigen::VectorXd t_c = Eigen::VectorXd::Zero(2);
    t_c << 1, 2;
    jc.PositionCommand(t_c);

    // Step clock 3 times to get ~95%
    c.StepClock();
    c.StepClock();
    c.StepClock();
    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t[0], 0.95, ERROR_BOUND);
    ASSERT_NEAR(t[1], 2*0.95, ERROR_BOUND);

    t_c << 3, 4;
    jc.PositionCommand(t_c);

    // Step clock 4 times
    c.StepClock();
    c.StepClock();
    c.StepClock();
    c.StepClock();

    jc.GetJointState(t, t_dot, t_ddot, e);
    ASSERT_NEAR(t[0], 3, 0.1);
    ASSERT_NEAR(t[1], 4, 0.1);
}