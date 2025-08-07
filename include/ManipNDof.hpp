#ifndef _MANIPNDOF_HPP_
#define _MANIPNDOF_HPP_

#include <array>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

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

    void CommandJointConfig(std::vector<double> thetas);
    void CommandJointVel(std::vector<double> thetas);

    void StepJointController();

    // Calculate Forward Kinematics
    // Note: Set joint config prior
    Eigen::Matrix4d Fk();

    // Calculate differential kinematics; calculate end-effector twist
    // Note: Set joint config and join velocity prior
    TwistType Dk();

private:
    enum ControlType
    {
        POSITION_CONTROL_TYPE,
        VELOCITY_CONTROL_TYPE,
        TORQUE_CONTROL_TYPE
    };

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

    // Spatial Geometric Jacobian & Spatial analytic jacobian
    Eigen::MatrixXd mSGJacobian;
    Eigen::MatrixXd mSAJacobian;

    // End-effector twist
    TwistType mEndEffectorTwist;
};

#endif //_MANIPNDOF_HPP_