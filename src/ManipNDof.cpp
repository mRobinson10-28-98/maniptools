#include <iostream>

#include "skew.hpp"
#include "ManipNDof.hpp"

ManipNDof::ManipNDof(I_JointController& jointController): mJointController(jointController)
{
}

void ManipNDof::Initialize(std::vector<TwistJoint> pManipTwists, 
    Eigen::Matrix4d g_0)
{
    mDof = pManipTwists.size();
    mJointTwists = pManipTwists;
    mG_0 = g_0;

    // Set
    mTheta.resize(mDof);
    mThetaCommand.resize(mDof);
    mThetaDot.resize(mDof);
    mThetaDotCommand.resize(mDof);
    mJointTwists.resize(mDof);
    mJointTransforms.resize(mDof);
    mSGJacobian.resize(6, mDof);
    mSAJacobian.resize(6, mDof);

    mControlType = POSITION_CONTROL_TYPE;
}

int ManipNDof::GetDof()
{
    return mDof;
}

Eigen::VectorXd ManipNDof::GetTheta()
{
    return mTheta;
}
Eigen::VectorXd ManipNDof::GetThetaDot()
{
    return mThetaDot;
}
Eigen::Matrix4d ManipNDof::GetPose()
{
    return mGt;
}

void ManipNDof::CommandJointConfig(std::vector<double> thetas)
{
    if(mDof != thetas.size())
    {
        throw std::runtime_error("Manip Dof does not match dimension of thetas given.");
    }    
    
    for (uint i = 0; i < mDof; i++)
    {
        mThetaCommand(i) = thetas[i];
    }

    mControlType = POSITION_CONTROL_TYPE;
}

void ManipNDof::CommandJointVel(std::vector<double> thetas)
{
    if(mDof != thetas.size())
    {
        throw std::runtime_error("Manip Dof does not match dimension of thetas given.");
    }    

    for (uint i = 0; i < mDof; i++)
    {
        mThetaDotCommand(i) = thetas[i];
    }

    mControlType = VELOCITY_CONTROL_TYPE;
}

void ManipNDof::StepJointController()
{
    if (mControlType == POSITION_CONTROL_TYPE)
    {
        mJointController.PositionControl(mTheta, mThetaCommand);
    }
    else if (mControlType == VELOCITY_CONTROL_TYPE)
    {
        mJointController.VelocityControl(mThetaDot, mThetaDotCommand);
    }
}

Eigen::Matrix4d ManipNDof::Fk()
{
    mGt = Eigen::Matrix4d::Identity();

    for (uint i = 0; i < mDof; i++)
    {
        mJointTransforms[i] = mGt;
        mGt *= mJointTwists[i].TwistExponential(mTheta(i));
    }

    mGt *= mG_0;

    return mGt;
}

TwistType ManipNDof::Dk()
{
    // Calculate each adjoint and adjust each joint twist for geometric jacobian
    for (uint i = 0; i < mDof; i++)
    {
        mJointTwists[i].TwistAdjointSpatial(mJointTransforms[i]);
        mSGJacobian.col(i) = mJointTwists[i].GetTwist();
    }

    Eigen::Matrix<double, 6, 6> geometricAnalyticMapAdjoint;

    geometricAnalyticMapAdjoint.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    geometricAnalyticMapAdjoint.block(3,3,3,3) = Eigen::Matrix3d::Identity();
    geometricAnalyticMapAdjoint.block(3,0,3,3) = Eigen::Matrix3d::Zero();

    Eigen::Vector3d position = mGt.col(3).head(3);
    geometricAnalyticMapAdjoint.block(0,3,3,3) = -skew3d(position);

    mSAJacobian = geometricAnalyticMapAdjoint * mSGJacobian;

    mEndEffectorTwist = mSAJacobian * mThetaDot;

    return mEndEffectorTwist;
}

