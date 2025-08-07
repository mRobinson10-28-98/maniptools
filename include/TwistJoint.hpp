#ifndef _TWIST_JOINT_HPP_
#define _TWIST_JOINT_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> TwistType;

class TwistJoint
{
public:
    // Default
    TwistJoint();
    // Given a 6x1 twist
    TwistJoint(TwistType twist);
    // Given position vector to joint axis q and axis w, assemble twist
    TwistJoint(Eigen::Vector3d q, Eigen::Vector3d w);
    ~TwistJoint(){}

    Eigen::Matrix4d TwistExponential(double theta);
    TwistType TwistAdjointSpatial(Eigen::Matrix4d previousSpatialTransform);

    TwistType GetTwist();
    TwistType GetBaseTwist();

private:
    // Twist in current joint configuration (using spatial adjoint matrix)
    TwistType mTwist;

    // Twist in base joint configuration (constant)
    TwistType mBaseTwist;
};

#endif //_TWIST_JOINT_HPP_