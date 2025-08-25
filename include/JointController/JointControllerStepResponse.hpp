#ifndef _JOINT_CONTROLLER_STEP_RESPONSE_HPP_
#define _JOINT_CONTROLLER_STEP_RESPONSE_HPP_

#include "JointControllerBase.hpp"

class JointControllerStepResponse: public JointControllerBase
{
public:
    JointControllerStepResponse();
    ~JointControllerStepResponse(){};

    // Joint positions (t), joint velocities (t_dot), joint accel (t_ddot), and joint efforts (e)
    void GetJointState(
        Eigen::VectorXd& t,
        Eigen::VectorXd& t_dot,
        Eigen::VectorXd& t_ddot,
        Eigen::VectorXd& e
    ) override; 

    void SetTau(double t);
};

#endif //_JOINT_CONTROLLER_STEP_RESPONSE_HPP_