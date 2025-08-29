#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "PathPlanningTools.hpp"

using namespace DualNumberTools;
using namespace PathPlanningTools;

const double ERROR_BOUND = 1e-6;

TEST(ScLERP_test, TranslateTest)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos_init(1.0, 0.0, 0.0);
    Eigen::Vector3d pos_final(1.0, 0.0, 4.0);
    Eigen::Vector3d pos_interp(1.0, 0.0, 2.0);

    DualQuaternion pose_initial(rot, pos_init);
    DualQuaternion pose_final(rot, pos_final);

    // Expected pose
    DualQuaternion pose_interp(rot, pos_interp);

    // Calculated pose
    DualQuaternion pose_sclerp = ScLERP(pose_initial, pose_final, 0.5);
    Eigen::Matrix3d rot_sclerp = pose_sclerp.RotationMatrix();
    Eigen::Vector3d pos_sclerp = pose_sclerp.PositionVector();

    std::cout << "ROT sclerp: \n" << rot_sclerp << "\n\n";
    std::cout << "position sclerp: \n" << pos_sclerp << "\n\n";

        // Check position
    for (int comp = 0; comp < 3; comp++)
    {
        ASSERT_NEAR(pos_sclerp(comp), pos_interp(comp), ERROR_BOUND);
    }

    // Check rotation matrix
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_sclerp(row, col), rot(row, col), ERROR_BOUND);
        }
    }

}