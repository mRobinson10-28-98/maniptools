#ifndef _MANIPNDOF_HPP_
#define _MANIPNDOF_HPP_

#include <array>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "Common.hpp"
#include "SimClock.hpp"
#include "TwistJoint.hpp"
#include "I_JointController.hpp"

class ManipNDof
{
public:
    ManipNDof() = delete;
    ManipNDof(I_JointController& jointController);
    ~ManipNDof(){}

    void Initialize(std::vector<TwistJoint> pManipTwists, 
        Eigen::Matrix4d g_0);

    // Hertz frequency
    void SetRunFrequency(double f);

    // Getters
    int GetDof();
    Eigen::VectorXd GetTheta();
    Eigen::VectorXd GetThetaDot();
    Eigen::Matrix4d GetPose();
    TwistType GetTwist();
    Eigen::MatrixXd GetSGJacobian();
    Eigen::MatrixXd GetSAJacobian();

    void CommandJointConfig(Eigen::VectorXd thetas);
    void CommandJointVel(Eigen::VectorXd thetas);

    // 1. Step Joint Controller and get joint states (position, velocity, acceleration, effort)
    // 2. Calculated forward kinematics
    // 3. Calculate differential fk
    void StepModel();

    Eigen::VectorXd CalculateJointConfigScLERP(Eigen::Matrix4d poseFinal, double interpParam);

protected:
    // Calculate Forward Kinematics
    // Note: Set joint config prior
    void Fk();

    // Calculate differential kinematics; calculate end-effector twist
    // Note: Set joint config and joint velocity prior
    void Dk();

private:
    // Sim clock
    SimClock& mClock {SimClock::GetInstance()};
    // ---
    // Static Manipulator Properties
    // ---
    // Number of joints
    uint mDof;

    I_JointController& mJointController;

    // Geometric description of all n joints (twists)
    std::vector<TwistJoint> mJointTwists;

    // ---
    // Control Properties
    // ---
    ControlType mControlType;

    // ---
    // Zeroth Order Properties
    // ---
    Eigen::VectorXd mTheta;
    Eigen::VectorXd mThetaCommand;

    // Forward kinematics from base joint to joint 'i' for calculating adjoints and transforming joint twists
    std::vector<Eigen::Matrix4d> mJointTransforms;

    // Initial pose at default joint configuration
    Eigen::Matrix4d mG_0;

    // Current pose
    Eigen::Matrix4d mGt;

    // ---
    // First Order Properties
    // ---
    Eigen::VectorXd mThetaDot;
    Eigen::VectorXd mThetaDotCommand;

    // ---
    // Second Order Properties (Ddot == 'double dot')
    // ---
    Eigen::VectorXd mThetaDdot;
    Eigen::VectorXd mThetaDdotCommand;
    
    // Spatial Geometric Jacobian & Spatial analytic jacobian
    Eigen::MatrixXd mSGJacobian;
    Eigen::MatrixXd mSAJacobian;

    // End-effector twist
    TwistType mEndEffectorTwist;

    // ---
    // Effort Properties
    // ---
    Eigen::VectorXd mEffort;
    Eigen::VectorXd mEffortCommand;
};

#endif //_MANIPNDOF_HPP_