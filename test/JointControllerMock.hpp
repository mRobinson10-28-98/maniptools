#ifndef _JOINT_CONTROLLER_MOCK_HPP_
#define _JOINT_CONTROLLER_MOCK_HPP_

#include <gmock/gmock.h>
#include "I_JointController.hpp"

class JointControllerMock : public I_JointController
{
public:
    MOCK_METHOD(void, PositionControl, (Eigen::VectorXd& t, Eigen::VectorXd& t_c), (override));
    MOCK_METHOD(void, VelocityControl, (Eigen::VectorXd& t, Eigen::VectorXd& t_c), (override));
};

#endif //_JOINT_CONTROLLER_MOCK_HPP_