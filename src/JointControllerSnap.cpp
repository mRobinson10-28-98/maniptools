#include "JointControllerSnap.hpp"

void JointControllerSnap::PositionControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c)
{
    t = t_c;
}

void JointControllerSnap::VelocityControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c)
{
    t = t_c;
}