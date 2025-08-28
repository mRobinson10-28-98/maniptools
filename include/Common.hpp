#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#include <Eigen/Dense>

enum ControlType
{
    POSITION_CONTROL_TYPE,
    VELOCITY_CONTROL_TYPE,
    ACCELERATION_CONTROL_TYPE,
    EFFORT_CONTROL_TYPE
};

Eigen::Matrix3d skew3d(Eigen::Vector3d v);

Eigen::MatrixXd PseudoInverse(Eigen::MatrixXd m);

#endif //_COMMON_HPP_