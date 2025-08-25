#include "JointControllerSnap.hpp"

JointControllerSnap::JointControllerSnap()
{
}

// Joint positions (t), joint velocities (t_dot), joint accel (t_ddot), and joint efforts (e)
void JointControllerSnap::GetJointState(
    Eigen::VectorXd& t,
    Eigen::VectorXd& t_dot,
    Eigen::VectorXd& t_ddot,
    Eigen::VectorXd& e
)
{
    if(mControlType == POSITION_CONTROL_TYPE)
    {
        mTheta = mJointCommand;
        mThetaDot.setZero();
        mThetaDdot.setZero();
    }
    else if(mControlType == VELOCITY_CONTROL_TYPE)
    {
        mTheta.setZero();
        mThetaDot = mJointCommand;
        mThetaDdot.setZero();
    }
    else if(mControlType == ACCELERATION_CONTROL_TYPE)
    {
        mTheta.setZero();
        mThetaDot.setZero();
        mThetaDdot = mJointCommand;
    }
    else if(mControlType == EFFORT_CONTROL_TYPE)
    {
        // TODO
    }
    else
    {
        // Nothing to see here
    }

    // return joint state
    t = mTheta;
    t_dot = mThetaDot;
    t_ddot = mThetaDdot;
    e = mEffort;
}