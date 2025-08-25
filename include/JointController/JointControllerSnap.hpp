#ifndef _JOINT_CONTROLLER_SNAP_HPP_
#define _JOINT_CONTROLLER_SNAP_HPP_

#include "JointControllerBase.hpp"

class JointControllerSnap: public JointControllerBase
{
public:
    JointControllerSnap();
    ~JointControllerSnap(){};

    // Joint positions (t), joint velocities (t_dot), joint accel (t_ddot), and joint efforts (e)
    void GetJointState(
        Eigen::VectorXd& t,
        Eigen::VectorXd& t_dot,
        Eigen::VectorXd& t_ddot,
        Eigen::VectorXd& e
    ) override; 
};

#endif //_JOINT_CONTROLLER_SNAP_HPP_