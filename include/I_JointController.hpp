#ifndef _I_JOINT_CONTROLLER_HPP_
#define _I_JOINT_CONTROLLER_HPP_

#include <Eigen/Dense>

class I_JointController
{
public:
    virtual ~I_JointController(){};

    // Current config, commanded config
    virtual void PositionControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c) = 0;
    virtual void VelocityControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c) = 0;
};

#endif //_I_JOINT_CONTROLLER_HPP_