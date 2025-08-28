#ifndef _JOINT_CONTROLLER_BASE_HPP_
#define _JOINT_CONTROLLER_BASE_HPP_

#include <Eigen/Dense>

#include "Common.hpp"
#include "I_JointController.hpp"
#include "SimClock.hpp"

class JointControllerBase: public I_JointController
{
public:
    virtual void Initialize(
        const Eigen::VectorXd& t,
        const Eigen::VectorXd& t_dot,
        const Eigen::VectorXd& t_ddot,
        const Eigen::VectorXd& e
    ) override; 

    // Current config, commanded config
    virtual void PositionCommand(const Eigen::VectorXd& t_c) override;
    virtual void VelocityCommand(const Eigen::VectorXd& t_c) override;
    virtual void AccelerationCommand(const Eigen::VectorXd& t_c) override;
    virtual void EffortCommand(const Eigen::VectorXd& t_c) override;

protected:
    // ---
    // Control Properties
    // ---

    SimClock& mClock {SimClock::GetInstance()};

    // Time at which the most recent command was given
    double mCommandTime;

    // Type of control based on most recent command (pos, vel, accel, or effort)
    ControlType mControlType {POSITION_CONTROL_TYPE};

    // Time constant
    double mTau {0.1};

    // Most recent joint command
    Eigen::VectorXd mJointCommand;
    Eigen::VectorXd mStartingJointState;

    // ---
    // Joint States
    // ---
    Eigen::VectorXd mTheta;
    Eigen::VectorXd mThetaDot;
    Eigen::VectorXd mThetaDdot;
    Eigen::VectorXd mEffort;
};

#endif //_JOINT_CONTROLLER_BASE_HPP_