#include <array>
#include <iostream>
#include <Eigen/Dense>

#include "Common.hpp"
#include "DualNumberTools.hpp"
#include "TwistJoint.hpp"
#include "Manip5Dof.hpp"
#include "JointControllerSnap.hpp"
#include "SimClock.hpp"

using namespace DualNumberTools;

int main()
{
    Eigen::Quaterniond q_r(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(1, 0, 0)));
    double w_r = q_r.w();
    Eigen::Vector3d v_r(q_r.x(), q_r.y(), q_r.z());

    Eigen::Quaterniond q_r_conj = q_r.conjugate();
    double w_r_conj = q_r_conj.w();
    Eigen::Vector3d v_r_conj(q_r_conj.x(), q_r_conj.y(), q_r_conj.z());

    Eigen::Quaterniond p(0, 1, 2, 3);
    Eigen::Quaterniond q_d = ScalarMultiplyQuaternion(p * q_r, 0.5);
    double w_d = q_d.w();
    Eigen::Vector3d v_d(q_d.x(), q_d.y(), q_d.z());

    Eigen::Quaterniond q_d_conj = q_d.conjugate();
    double w_d_conj = q_d_conj.w();
    Eigen::Vector3d v_d_conj(q_d_conj.x(), q_d_conj.y(), q_d_conj.z());

    Eigen::Quaterniond p2 = ScalarMultiplyQuaternion(q_d * q_r_conj, 2);
    Eigen::Quaterniond p2_conj = ScalarMultiplyQuaternion(q_d_conj * q_r, -2);

    std::cout << "q_r: \n" << q_r << "\n\n";
    std::cout << "q_r_conj: \n" << q_r_conj << "\n\n";

    std::cout << "q_d: \n" << q_d << "\n\n";
    std::cout << "q_d_conj: \n" << q_d_conj << "\n\n";

    std::cout << "p2: \n" << p2 << "\n\n";
    std::cout << "p2_conj: \n" << p2_conj << "\n\n";

    std::cout << "q_d * q_r.conj: \n" << q_d * q_r.conjugate() << "\n\n";
    std::cout << "q_d_conj * q_r_conj.conj: \n" << q_d_conj * q_r_conj.conjugate() << "\n\n";

    std::cout << "Manual calc: \n" << 2 * (w_d * v_r_conj + w_r_conj * v_d + v_d.cross(v_r_conj)) << "\n\n";
    std::cout << "Manual calc: \n" << 2 * (w_d_conj * v_r + w_r * v_d_conj + v_d_conj.cross(v_r)) << "\n\n";

}