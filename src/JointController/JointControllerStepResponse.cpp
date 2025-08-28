#include <cmath>
#include "JointControllerStepResponse.hpp"

JointControllerStepResponse::JointControllerStepResponse()
{
}

// Joint positions (t), joint velocities (t_dot), joint accel (t_ddot), and joint efforts (e)
void JointControllerStepResponse::GetJointState(
    Eigen::VectorXd& t,
    Eigen::VectorXd& t_dot,
    Eigen::VectorXd& t_ddot,
    Eigen::VectorXd& e
)
{
    // TODO use derivatives and taylor series approx
    // TODO add sensor noise?

    Eigen::VectorXd previous_theta = mTheta;
    Eigen::VectorXd previous_theta_dot = mThetaDot;

    double ellapsed_time = mClock.GetSimTime() - mCommandTime;

    if(mControlType == POSITION_CONTROL_TYPE)
    {
        // First order step response function
        mTheta = mJointCommand + (mStartingJointState - mJointCommand) * exp(-ellapsed_time / mTau);
        
        // Discrete time derivatives
        mThetaDot = (mTheta - previous_theta) * mClock.GetSimFreq();
        mThetaDdot = (mThetaDot - previous_theta_dot) * mClock.GetSimFreq();
    }
    else if(mControlType == VELOCITY_CONTROL_TYPE)
    {
        // First order step response function
        mThetaDot = mJointCommand + (mStartingJointState - mJointCommand) * exp(-ellapsed_time / mTau);

        // Discrete time integral
        mTheta += mThetaDot / mClock.GetSimFreq();

        // Discrete time derivative
        mThetaDdot = (mThetaDot - previous_theta_dot) * mClock.GetSimFreq();
    }
    else if(mControlType == ACCELERATION_CONTROL_TYPE)
    {
        // First order step response function
        mThetaDdot = mJointCommand + (mStartingJointState - mJointCommand) * exp(-ellapsed_time / mTau);
        
        // Discrete time integral
        mThetaDot += mThetaDdot / mClock.GetSimFreq();
        mTheta += mThetaDot / mClock.GetSimFreq();
    }
    else if(mControlType == EFFORT_CONTROL_TYPE)
    {
        // TODO
    }
    else
    {
        mTheta = mTheta;
        mThetaDot = mThetaDot;
        mThetaDdot = mThetaDdot;
        mEffort = mEffort;
    }

    // return joint state
    t = mTheta;
    t_dot = mThetaDot;
    t_ddot = mThetaDdot;
    e = mEffort;
}

void JointControllerStepResponse::SetTau(double t)
{
    mTau = t;
}