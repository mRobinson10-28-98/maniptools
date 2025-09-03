#include <iostream>

#include "Common.hpp"
#include "DualNumberTools.hpp"
#include "ManipNDof.hpp"
#include "PathPlanningTools.hpp"

using namespace DualNumberTools;
using namespace PathPlanningTools;

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

TwistType ManipNDof::GetTwist()
{
    return mEndEffectorTwist;
}

Eigen::MatrixXd ManipNDof::GetSGJacobian()
{
    return mSGJacobian;
}

Eigen::MatrixXd ManipNDof::GetSAJacobian()
{
    return mSAJacobian;
}

void ManipNDof::SetRunFrequency(double f)
{
    mClock.SetSimFreq(f);
}

void ManipNDof::CommandJointConfig(Eigen::VectorXd thetas)
{
    if(mDof != thetas.size())
    {
        throw std::runtime_error("Manip Dof does not match dimension of thetas given.");
    }    
    
    for (uint i = 0; i < mDof; i++)
    {
        mThetaCommand(i) = thetas[i];
    }

    mJointController.PositionCommand(mThetaCommand);

    mControlType = POSITION_CONTROL_TYPE;
}

void ManipNDof::CommandJointVel(Eigen::VectorXd thetas)
{
    if(mDof != thetas.size())
    {
        throw std::runtime_error("Manip Dof does not match dimension of thetas given.");
    }    

    for (uint i = 0; i < mDof; i++)
    {
        mThetaDotCommand(i) = thetas[i];
    }

    mJointController.VelocityCommand(mThetaDotCommand);

    mControlType = VELOCITY_CONTROL_TYPE;
}

void ManipNDof::StepModel()
{
    // Step simulation clock and get joint state of next step
    mClock.StepClock();
    mJointController.GetJointState(mTheta, mThetaDot, mThetaDdot, mEffort);

    // Calculate manipulator
    Fk();
    Dk();
}

void ManipNDof::CommandJointConfigScLERP(Eigen::Matrix4d poseFinal, double interpParam)
{
    // Current and final pose dual-quat
    DualQuaternion q_t(mGt);
    DualQuaternion q_f(poseFinal);

    // ScLERP
    DualQuaternion pose_i = ScLERP(q_t, q_f, interpParam);
    
    // Deconstruct poses
    Eigen::Vector3d p_t = q_t.PositionVector();
    Eigen::Quaterniond q_r_t = q_t.real;
    Eigen::Vector3d p_i = pose_i.PositionVector();
    Eigen::Quaterniond q_r_i = pose_i.real;

    // Calculate pose vectors
    Eigen::Vector<double,7> pose_vector_t(
        p_t(0),
        p_t(1),
        p_t(2),
        q_r_t.w(),
        q_r_t.x(),
        q_r_t.y(),
        q_r_t.z()
    );

    Eigen::Vector<double,7> pose_vector_i(
        p_i(0),
        p_i(1),
        p_i(2),
        q_r_i.w(),
        q_r_i.x(),
        q_r_i.y(),
        q_r_i.z()
    );

    // Calculate pose vector Jacobian 'B'
    Eigen::Matrix<double, 6, 7> J2;
    J2.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    J2.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
    J2.block(0, 3, 3, 4) = 2 * skew3d(p_t) * skewQuat(q_r_t);
    J2.block(3, 3, 3, 4) = 2 * skewQuat(q_r_t);
    Eigen::MatrixXd B = PseudoInverse(mSAJacobian) * J2;

    // Calculate next joint config
    Eigen::VectorXd theta_dot = B * (pose_vector_i - pose_vector_t);
    // Eigen::VectorXd theta_i = mTheta + (0.5 * theta_dot) / mClock.GetSimFreq();
    Eigen::VectorXd theta_i = mTheta + (0.5 * theta_dot);
    CommandJointConfig(theta_i);
}

void ManipNDof::Fk()
{
    mGt = Eigen::Matrix4d::Identity();

    for (uint i = 0; i < mDof; i++)
    {
        mJointTransforms[i] = mGt;
        mGt *= mJointTwists[i].TwistExponential(mTheta(i));
    }

    mGt *= mG_0;
}

void ManipNDof::Dk()
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
}

