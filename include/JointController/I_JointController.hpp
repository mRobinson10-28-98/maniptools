#ifndef _I_JOINT_CONTROLLER_HPP_
#define _I_JOINT_CONTROLLER_HPP_

#include <Eigen/Dense>

class I_JointController
{
public:
    virtual ~I_JointController(){};

    virtual void Initialize(
        const Eigen::VectorXd& t,
        const Eigen::VectorXd& t_dot,
        const Eigen::VectorXd& t_ddot,
        const Eigen::VectorXd& e
    ) = 0; 

    // Current config, commanded config
    virtual void PositionCommand(const Eigen::VectorXd& t_c) = 0;
    virtual void VelocityCommand(const Eigen::VectorXd& t_c) = 0;
    virtual void AccelerationCommand(const Eigen::VectorXd& t_c) = 0;
    virtual void EffortCommand(const Eigen::VectorXd& t_c) = 0;

    // Joint positions (t), joint velocities (t_dot), joint accel (t_ddot), and joint efforts (e)
    virtual void GetJointState(
        Eigen::VectorXd& t,
        Eigen::VectorXd& t_dot,
        Eigen::VectorXd& t_ddot,
        Eigen::VectorXd& e
    ) = 0; 
};

#endif //_I_JOINT_CONTROLLER_HPP_