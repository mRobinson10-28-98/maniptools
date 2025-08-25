#include <array>
#include <iostream>
#include <Eigen/Dense>

#include "skew.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"

#include "JointControllerSnap.hpp"
#include "SimClock.hpp"

int main()
{
    // Twist Test
    Eigen::Vector3d q_test {1,0,0};
    Eigen::Vector3d z_axis {0,0,1};
    TwistJoint joint(q_test, z_axis);
    std::cout << "Testing joint twist exp: 0 degrees\n" << joint.TwistExponential(0) << std::endl;
    std::cout << "Testing joint twist exp: 90 degrees\n" << joint.TwistExponential(M_PI/2) << std::endl;
    std::cout << "Testing joint twist exp: 180 degrees\n" << joint.TwistExponential(M_PI) << std::endl;
    std::cout << "Testing joint twist exp: 270 degrees\n" << joint.TwistExponential(3 * M_PI / 4) << std::endl;
    std::cout << "Testing joint twist exp: 360 degrees\n" << joint.TwistExponential(2 * M_PI) << std::endl;
    std::cout << "\n\n";
    JointControllerSnap myJC;

    std::array<double, 5> link_lengths {5,4,3,2,1};
    Manip5Dof myManip(myJC, link_lengths);

    std::vector<double> q {M_PI, 0, 0, 0, 0};
    // Command
    myManip.CommandJointConfig(q);
    // Step sim
    myManip.StepModel();
    // Get State
    Eigen::Matrix4d fk = myManip.GetPose();

    std::cout << "Planar 5-bar manip with j0 rotated 180 degrees: \n" << fk << std::endl;

    // Reset to null config and command velocity
    q.at(0) = 0;
    myManip.CommandJointConfig(q);
    myManip.StepModel();
    fk = myManip.GetPose();
    std::cout << "Planar 5-bar manip null config: \n" << fk << std::endl;

    std::vector<double> q_dot {1, 0, 0, 0, 0};
    myManip.CommandJointVel(q_dot);
    myManip.StepModel();

    // Should be in positive y direction
    TwistType my_twist = myManip.GetTwist();
    std::cout << "Planar 5-bar end-effector twist with j0 = +1rad/s: \n" << my_twist << std::endl;

    q_dot.at(0) = 0;
    q_dot.at(4) = 1;
    myManip.CommandJointVel(q_dot);
    myManip.StepModel();

    // Should be in positive y direction but much smaller
    my_twist = myManip.GetTwist();
    std::cout << "Planar 5-bar end-effector twist with j5 = +1rad/s: \n" << my_twist << std::endl;

    q.at(3) = M_PI/2;
    
    SimClock& mClock = SimClock::GetInstance();
    mClock.SetSimFreq(1000);
    std::cout << "Clock Time: " << mClock.GetSimTime() << std::endl;
    mClock.StepClock();
    std::cout << "Clock Time: " << mClock.GetSimTime() << std::endl;

    SimClock& mClock2 = SimClock::GetInstance();
    std::cout << "Clock Time 2: " << mClock2.GetSimTime() << std::endl;
}