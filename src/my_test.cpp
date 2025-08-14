#include <array>
#include <iostream>
#include <Eigen/Dense>

#include "skew.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"

#include "JointControllerSnap.hpp"

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
    myManip.CommandJointConfig(q);
    myManip.StepJointController();
    Eigen::Matrix4d fk = myManip.Fk();
    std::cout << "Planar 5-bar manip with j0 rotated 180 degrees: \n" << fk << std::endl;

    // Reset to null config and command velocity
    q.at(0) = 0;
    myManip.CommandJointConfig(q);
    myManip.StepJointController();
    fk = myManip.Fk();
    std::cout << "Planar 5-bar manip null config: \n" << fk << std::endl;

    std::vector<double> q_dot {1, 0, 0, 0, 0};
    myManip.CommandJointVel(q_dot);
    myManip.StepJointController();

    // Should be in positive y direction
    TwistType my_twist = myManip.Dk();
    std::cout << "Planar 5-bar end-effector twist with j0 = +1rad/s: \n" << my_twist << std::endl;

    q_dot.at(0) = 0;
    q_dot.at(4) = 1;
    myManip.CommandJointVel(q_dot);
    myManip.StepJointController();

    // Should be in positive y direction but much smaller
    my_twist = myManip.Dk();
    std::cout << "Planar 5-bar end-effector twist with j5 = +1rad/s: \n" << my_twist << std::endl;

    q.at(3) = M_PI/2;
    
    // // random tests
    // std::array<TwistJoint, 5> ts;

    // std::array<double, 5> ls {5, 4, 3, 2, 1};

    // ls[0] = 5;

    // Eigen::Vector3d tmp_v;
    // Eigen::Vector3d z_axis {0, 0, 1};

    // for (int i = 0; i < 5; i++)
    // {
    //     tmp_v << ls[i], 0, 0;
    //     ts[i] = TwistJoint(tmp_v, z_axis);
    // }

    // for (int i = 0; i < 5; i++)
    // {
    //     std::cout << "Twist " << i << ": \n" << ts[i].GetTwist() << std::endl;
    // }

    // Eigen::Vector3d my_vector;
    // my_vector << 1, 2, 3;
    // std::cout << "My vector: \n" << my_vector << std::endl;

    // Eigen::Matrix3d my_vector_skewed = skew3d(my_vector);    
    // std::cout << "My vector 1,2,3 skewed: \n" << my_vector_skewed << std::endl;

    // std::cout << "Identity \n" << Eigen::Matrix3d::Identity() << std::endl;

    // std::cout << "Vector x vector transposed is 3x3: \n" << my_vector * my_vector.transpose() << std::endl;
    // std::cout << "3x3 identity times column is column: \n" << Eigen::Matrix3d::Identity() * my_vector << std::endl;

    // // 1 bar test; link length of 2
    // double l = 2.0;
    // Eigen::Vector3d p_0 {l, 0, 0};
    // Eigen::Vector3d q {0, 0, 0};
    // Eigen::Vector3d axis {0, 0, 1};

    // Eigen::Matrix4d g_0;
    // g_0.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    // g_0.col(3).head(3) = p_0;
    // g_0.row(3) << 0, 0, 0, 1;

    // TwistJoint MyJoint(q, axis);

    // std::cout << "Initial transform of 1-bar EE: \n" << g_0 << std::endl;
    // std::cout << "1-bar EE after rotation of 180 degrees: \n" << MyJoint.TwistExponential(M_PI) * g_0 << std::endl;
}