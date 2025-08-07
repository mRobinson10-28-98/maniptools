#include "JointControllerLerp.hpp"

// Public Interface
void JointControllerLerp::PositionControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c)
{
    for (uint i = 0; i < t.rows(); i++)
    {
        t(i) = Lerp(t(i), t_c(i), mPosScale);
    }
}

void JointControllerLerp::VelocityControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c)
{
    for (uint i = 0; i < t.rows(); i++)
    {
        t(i) = Lerp(t(i), t_c(i), mVelScale);
    }}

void JointControllerLerp::SetPositionScalar(double lerp_scalar)
{
    mPosScale = lerp_scalar;
}

void JointControllerLerp::SetVelocityScalar(double lerp_scalar)
{
    mVelScale = lerp_scalar;
}

double JointControllerLerp::Lerp(double current, double commanded, double inc)
{
    return current + (commanded - current) * inc;
}
