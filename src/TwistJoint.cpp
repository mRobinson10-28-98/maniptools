#include <math.h>

#include "TwistJoint.hpp"
#include "Common.hpp"

// Default
TwistJoint::TwistJoint()
{
    mTwist << 0, 0, 0, 0, 0, 1;
    mBaseTwist = mTwist;
}

// Given a 6x1 twist
TwistJoint::TwistJoint(TwistType twist)
{
    mTwist = twist;

    mBaseTwist = mTwist;
}

// Given position vector to joint axis q and axis w, assemble twist
TwistJoint::TwistJoint(Eigen::Vector3d q, Eigen::Vector3d w)
{
    w.normalize();
    mTwist.head(3) = -w.cross(q);
    mTwist.tail(3) = w;

    mBaseTwist = mTwist;
}

Eigen::Matrix4d TwistJoint::TwistExponential(double theta)
{
    Eigen::Matrix4d transform;
    Eigen::Vector3d v = mBaseTwist.head(3);
    Eigen::Vector3d w = mBaseTwist.tail(3);

    Eigen::Matrix3d rotation, w_skew;
    Eigen::Vector3d translation;

    //Rodrigues formula
    w_skew = skew3d(w);
    rotation = Eigen::Matrix3d::Identity() + w_skew * sin(theta) + w_skew * w_skew * (1 - cos(theta));
    translation = (Eigen::Matrix3d::Identity() - rotation) * w.cross(v) + (w * w.transpose() * v * theta);

    transform.block(0,0,3,3) = rotation;
    transform.col(3).head(3) = translation;
    transform.row(3) << 0, 0, 0, 1;
    return transform;
}

TwistType TwistJoint::TwistAdjointSpatial(Eigen::Matrix4d previousSpatialTransform)
{
    Eigen::Matrix3d R = previousSpatialTransform.block(0,0,3,3);
    Eigen::Vector3d p = previousSpatialTransform.col(3).head(3);

    Eigen::Matrix<double, 6, 6> Ad;
    Ad.block(0,0,3,3) = R;
    Ad.block(3,3,3,3) = R;
    Ad.block(0,3,3,3) = skew3d(p) * R;
    Ad.block(3,0,3,3) = Eigen::Matrix3d::Zero();

    mTwist = Ad * mBaseTwist;
    return mTwist;
}


TwistType TwistJoint::GetTwist()
{
    return mTwist;
}

TwistType TwistJoint::GetBaseTwist()
{
    return mBaseTwist;
}