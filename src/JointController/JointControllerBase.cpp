#include "JointControllerBase.hpp"

void JointControllerBase::Initialize(
    const Eigen::VectorXd& t,
    const Eigen::VectorXd& t_dot,
    const Eigen::VectorXd& t_ddot,
    const Eigen::VectorXd& e
)
{
    mTheta = t;
    mThetaDot = t_dot;
    mThetaDdot = t_ddot;
    mEffort = e;

    mCommandTime = mClock.GetSimTime();

    mJointCommand = t;
    mStartingJointState = t;
}

void JointControllerBase::PositionCommand(const Eigen::VectorXd& t_c)
{
    // Set command properties and states
    mCommandTime = mClock.GetSimTime();
    mControlType = POSITION_CONTROL_TYPE;
    mJointCommand = t_c;
    mStartingJointState = mTheta;
}

void JointControllerBase::VelocityCommand(const Eigen::VectorXd& t_c)
{
    // Set command properties and states
    mCommandTime = mClock.GetSimTime();
    mControlType = VELOCITY_CONTROL_TYPE;
    mJointCommand = t_c;
    mStartingJointState = mThetaDot;
}

void JointControllerBase::AccelerationCommand(const Eigen::VectorXd& t_c)
{
    // Set command properties and states
    mCommandTime = mClock.GetSimTime();
    mControlType = ACCELERATION_CONTROL_TYPE;
    mJointCommand = t_c;
    mStartingJointState = mThetaDdot;
}

void JointControllerBase::EffortCommand(const Eigen::VectorXd& t_c)
{
    // Set command properties and states
    mCommandTime = mClock.GetSimTime();
    mControlType = EFFORT_CONTROL_TYPE;
    mJointCommand = t_c;
    mStartingJointState = mEffort;
}