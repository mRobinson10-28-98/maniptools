#ifndef _JOINT_CONTROLLER_LERP_HPP_
#define _JOINT_CONTROLLER_LERP_HPP_

#include <Eigen/Dense>
#include "I_JointController.hpp"

class JointControllerLerp: public I_JointController
{
public:
    JointControllerLerp(){}
    ~JointControllerLerp(){}

    void PositionControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c);
    void VelocityControl(Eigen::VectorXd& t, Eigen::VectorXd& t_c);

    // Outside Interface
    void SetPositionScalar(double lerp_scalar);
    void SetVelocityScalar(double lerp_scalar);

protected:
    // 0 <= inc <= 1
    double Lerp(double current, double commanded, double inc);

private:
    // Probably eventually need to be adjusted to be a real slope once sim time is introduced
    double mPosScale;
    double mVelScale;
};

#endif //_JOINT_CONTROLLER_LERP_HPP_