#ifndef _MANIP5DOF_HPP_
#define _MANIP5DOF_HPP_

#include <array>
#include <iostream>
#include <Eigen/Dense>

#include "I_JointController.hpp"
#include "ManipNDof.hpp"
#include "TwistJoint.hpp"

class Manip5Dof: public ManipNDof
{
public:
    Manip5Dof() = delete;
    Manip5Dof(I_JointController& jointController, std::array<double, 5> link_lengths);
    ~Manip5Dof(){}
};

#endif //_MANIP5DOF_HPP_