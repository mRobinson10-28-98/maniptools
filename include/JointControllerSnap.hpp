#ifndef _JOINT_CONTROLLER_SNAP_HPP_
#define _JOINT_CONTROLLER_SNAP_HPP_

#include <Eigen/Dense>
#include "I_JointController.hpp"

class JointControllerSnap: public I_JointController
{
public:
    JointControllerSnap(){}
    ~JointControllerSnap(){}

    void PositionControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c);
    void VelocityControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c);
};

#endif //_JOINT_CONTROLLER_SNAP_HPP_