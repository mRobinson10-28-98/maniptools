#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "PathPlanningTools.hpp"

using namespace DualNumberTools;
using namespace PathPlanningTools;

const double ERROR_BOUND = 1e-6;

class PathPlanningTest : public ::testing::Test
{
protected:
    Eigen::Vector3d x_axis {1.0, 0.0, 0.0};
    Eigen::Vector3d y_axis {0.0, 1.0, 0.0};
    Eigen::Vector3d z_axis {0.0, 0.0, 1.0};

    Eigen::Matrix3d R_x;
    Eigen::Matrix3d R_y;
    Eigen::Matrix3d R_z;

    Eigen::Matrix4d Transform(Eigen::Matrix3d R, Eigen::Vector3d p)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block(0, 0, 3, 3) = R;
        transform(0, 3) = p(0);
        transform(1, 3) = p(1);
        transform(2, 3) = p(2);
        return transform;
    }

    // Compare calculated rotation matrix to expected rotation matrix
    void Test_Rot(Eigen::Matrix3d R_calc, Eigen::Matrix3d R_exp)
    {
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                ASSERT_NEAR(R_calc(row, col), R_exp(row, col), ERROR_BOUND);
            }
        }
    }

    // Compare calculated position vector to expected
    void Test_Pos(Eigen::Vector3d pos_calc, Eigen::Vector3d pos_exp)
    {
        ASSERT_NEAR(pos_calc(0), pos_exp(0), ERROR_BOUND);
        ASSERT_NEAR(pos_calc(1), pos_exp(1), ERROR_BOUND);
        ASSERT_NEAR(pos_calc(2), pos_exp(2), ERROR_BOUND);
    }

    // Compare calculated transformation matrix to expected
    void Test_Transform(Eigen::Matrix4d T_calc, Eigen::Matrix4d T_exp)
    {
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                ASSERT_NEAR(T_calc(row, col), T_exp(row, col), ERROR_BOUND);
            }
        }
    }

    // Setup test object
    void SetUp() override 
    {
        R_x = Eigen::AngleAxisd(M_PI/4, x_axis);
        R_y = Eigen::AngleAxisd(M_PI/4, y_axis);
        R_z = Eigen::AngleAxisd(M_PI/4, z_axis);
    }

    void TearDown() override
    {

    }
};

TEST_F(PathPlanningTest, ScLERP_Translate)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos_init(1.0, 0.0, 0.0);
    Eigen::Vector3d pos_final(1.0, 0.0, 4.0);

    DualQuaternion pose_initial(rot, pos_init);
    DualQuaternion pose_final(rot, pos_final);

    // Calculated pose
    DualQuaternion pose_interp_calc = ScLERP(pose_initial, pose_final, 0.5);
    Eigen::Vector3d pos_interp_calc = pose_interp_calc.PositionVector();
    Eigen::Matrix3d rot_interp_calc = pose_interp_calc.RotationMatrix();

    std::cout << "position sclerp: \n" << pos_interp_calc << "\n\n";
    std::cout << "ROT sclerp: \n" << rot_interp_calc << "\n\n";

    // Expected pose
    Eigen::Vector3d pos_interp_exp(1.0, 0.0, 2.0);

    // Position
    Test_Pos(pos_interp_calc, pos_interp_exp);

    // Check rotation matrix
    Test_Rot(rot_interp_calc, Eigen::Matrix3d::Identity());
}

TEST_F(PathPlanningTest, ScLERP_Rotatex)
{
    Eigen::Vector3d pos_init(1.0, 2.0, 3.0);
    Eigen::Vector3d pos_final = pos_init;

    DualQuaternion pose_initial(Eigen::Matrix3d::Identity(), pos_init);
    DualQuaternion pose_final(R_x * R_x, pos_final);

    // Calculated pose
    DualQuaternion pose_interp_calc = ScLERP(pose_initial, pose_final, 0.5);
    Eigen::Vector3d pos_interp_calc = pose_interp_calc.PositionVector();
    Eigen::Matrix3d rot_interp_calc = pose_interp_calc.RotationMatrix();

    std::cout << "position sclerp: \n" << pos_interp_calc << "\n\n";
    std::cout << "ROT sclerp: \n" << rot_interp_calc << "\n\n";

    // Position
    Test_Pos(pos_interp_calc, pos_init);

    // Check rotation matrix
    Test_Rot(rot_interp_calc, R_x);
}

TEST_F(PathPlanningTest, ScLERP_RotxPosx)
{
    Eigen::Vector3d pos_init(1.0, 2.0, 3.0);
    Eigen::Vector3d pos_final(5.0, 2.0, 3.0);

    DualQuaternion pose_initial(Eigen::Matrix3d::Identity(), pos_init);
    DualQuaternion pose_final(R_x * R_x, pos_final);

    // Calculated pose
    DualQuaternion pose_interp_calc = ScLERP(pose_initial, pose_final, 0.5);
    Eigen::Vector3d pos_interp_calc = pose_interp_calc.PositionVector();
    Eigen::Matrix3d rot_interp_calc = pose_interp_calc.RotationMatrix();

    std::cout << "position sclerp: \n" << pos_interp_calc << "\n\n";
    std::cout << "ROT sclerp: \n" << rot_interp_calc << "\n\n";

    // Expected pose
    Eigen::Vector3d pos_interp_exp(3.0, 2.0, 3.0);

    // Position
    Test_Pos(pos_interp_calc, pos_interp_exp);

    // Check rotation matrix
    Test_Rot(rot_interp_calc, R_x);
}

TEST_F(PathPlanningTest, ScLERP_RotzPosx)
{
    Eigen::Vector3d pos_init(4.0, 0.0, 0.0);
    Eigen::Vector3d pos_final(0.0, 0.0, 0.0);

    DualQuaternion pose_initial(Eigen::Matrix3d::Identity(), pos_init);

    // Rotate CW about axis centered at <3, 0, 0>
    // final pose is rotated 180d at origin
    DualQuaternion pose_final(Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, z_axis)), pos_final);

    // Calculated pose
    DualQuaternion pose_interp_calc = ScLERP(pose_initial, pose_final, 0.5);
    Eigen::Vector3d pos_interp_calc = pose_interp_calc.PositionVector();
    Eigen::Matrix3d rot_interp_calc = pose_interp_calc.RotationMatrix();

    std::cout << "position sclerp: \n" << pos_interp_calc << "\n\n";
    std::cout << "ROT sclerp: \n" << rot_interp_calc << "\n\n";

    // Expected pose
    Eigen::Vector3d pos_interp_exp(2.0, 2.0, 0.0);

    // Position
    Test_Pos(pos_interp_calc, pos_interp_exp);

    // Check rotation matrix
    Test_Rot(rot_interp_calc, Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/2, z_axis)));
}