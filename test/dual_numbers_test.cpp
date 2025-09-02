#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "DualNumberTools.hpp"

using namespace DualNumberTools;

const double ERROR_BOUND = 1e-6;

class DualNumberToolsTest : public ::testing::Test
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

TEST_F(DualNumberToolsTest, DualNumberMath)
{
    // Reference:
    // d1 = a + bi
    // d2 = c + di
    DualNumber d1(1,2);
    DualNumber d2(3,4);

    // - - - 
    // EXPECTED SOLUTIONS
    // - - -

    // d1 + d2 = (a + c) + (b + d)i
    DualNumber d_sum(4, 6);
    DualNumber d_min(-2, -2);

    // d1 * d2 = ac + (ad + bc)i
    DualNumber d_prod(3, 10);

    DualNumber d1_copy(1,2);

    ASSERT_EQ(d1 + d2, d_sum);
    ASSERT_EQ(d1 - d2, d_min);
    ASSERT_EQ(d1 * d2, d_prod);
    ASSERT_EQ(d1 == d2, false);
    ASSERT_EQ(d1 == d1_copy, true);
}

TEST_F(DualNumberToolsTest, DualVectorMath)
{
    // Reference:
    // v1 = a + bi
    // v2 = c + di
    Eigen::Vector3d v1_real {1, 2, 3};
    Eigen::Vector3d v1_dual {4, 5, 6};
    Eigen::Vector3d v2_real {7, 8, 9};
    Eigen::Vector3d v2_dual {10, 11, 12};

    DualVector v1(v1_real, v1_dual);
    DualVector v2(v2_real, v2_dual);

    // - - - 
    // EXPECTED SOLUTIONS
    // - - -

    Eigen::Vector3d v_sum_real {8, 10, 12};
    Eigen::Vector3d v_sum_dual {14, 16, 18};
    DualVector v_sum(v_sum_real, v_sum_dual);

    Eigen::Vector3d v_min_real {-6, -6, -6};
    Eigen::Vector3d v_min_dual {-6, -6, -6};
    DualVector v_min(v_min_real, v_min_dual);

    DualVector v1_copy(v1_real, v1_dual);

    // Check indexing
    DualNumber v1_0(1, 4);
    ASSERT_EQ(v1[0], v1_0);

    // Addition and subtraction
    ASSERT_EQ(v1 + v2, v_sum);
    ASSERT_EQ(v1 - v2, v_min);


    ASSERT_EQ(v1 == v2, false);
    ASSERT_EQ(v1 == v1_copy, true);
}

TEST_F(DualNumberToolsTest, ConstructQuaternions)
{
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    Eigen::Matrix4d transform = Transform(R_z, pos);

    DualQuaternion q1(R_z, pos);
    DualQuaternion q2(transform);

    // Q1
    Eigen::Vector3d q1_pos_from_function = q1.PositionVector();
    Eigen::Matrix3d q1_rot_from_function = q1.RotationMatrix();
    Eigen::Matrix4d q1_transform_from_function = q1.TransformationMatrix();
    // Q2
    Eigen::Vector3d q2_pos_from_function = q2.PositionVector();
    Eigen::Matrix3d q2_rot_from_function = q2.RotationMatrix();
    Eigen::Matrix4d q2_transform_from_function = q2.TransformationMatrix();


    // Check positions
    Test_Pos(q1_pos_from_function, pos);
    Test_Pos(q2_pos_from_function, pos);

    // Check rotation matrices
    Test_Rot(q1_rot_from_function, R_z);
    Test_Rot(q2_rot_from_function, R_z); 

    // Check transformation matrices
    Test_Transform(q1_transform_from_function, transform);
    Test_Transform(q2_transform_from_function, transform);
}

TEST_F(DualNumberToolsTest, QuaternionProducts_Rotate)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    DualQuaternion q1(R_z, pos);
    DualQuaternion q_prod = q1 * q1;
    Eigen::Matrix3d rot_calculated = q_prod.RotationMatrix();
    Eigen::Vector3d pos_calculated = q_prod.PositionVector();


    // Expected
    Eigen::Matrix3d rot_expected = R_z * R_z;

    // Position
    Test_Pos(pos_calculated, pos);

    // Check rotation matrix
    Test_Rot(rot_calculated, rot_expected);
}


TEST_F(DualNumberToolsTest, QuaternionProducts_Translate)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos1(1, 2, 3);
    Eigen::Vector3d pos2(4, 5, 6);

    DualQuaternion q1(rot, pos1);
    DualQuaternion q2(rot, pos2);
    DualQuaternion q_prod = q1 * q2;
    Eigen::Vector3d pos_calculated = q_prod.PositionVector();
    Eigen::Matrix3d rot_calculated = q_prod.RotationMatrix();

    // Expected
    Eigen::Vector3d pos_expected(5, 7, 9);
    Eigen::Matrix3d rot_expected = Eigen::Matrix3d::Identity();

    // Position
    Test_Pos(pos_calculated, pos_expected);

    // Check rotation matrix
    Test_Rot(rot_calculated, rot_expected);
}

TEST_F(DualNumberToolsTest, QuaternionProducts_Transform)
{
    Eigen::Vector3d pos1(1, 0, 0);
    Eigen::Vector3d pos2(0, 1, 1);

    Eigen::Matrix3d R_z_90d = R_z * R_z;
    Eigen::Matrix3d R_x_90d = R_x * R_x;

    DualQuaternion q1(R_z_90d, pos1);
    DualQuaternion q2(R_x_90d, pos2);

    DualQuaternion q_prod = q1 * q2;
    Eigen::Vector3d pos_calculated = q_prod.PositionVector();
    Eigen::Matrix3d rot_calculated = q_prod.RotationMatrix();

    // Expected
    Eigen::Vector3d pos_expected(0, 0, 1);
    Eigen::Matrix3d rot_expected = R_z_90d * R_x_90d;

    DualQuaternion q_exp(rot_expected, pos_expected);

    // Position
    Test_Pos(pos_calculated, pos_expected);

    // Check rotation matrix
    Test_Rot(rot_calculated, rot_expected);
}

TEST_F(DualNumberToolsTest, QuaternionPower_Translate)
{
    Eigen::Vector3d pos(2, 3, 4);
    DualQuaternion q(Eigen::Matrix3d::Identity(), pos);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    // Expectations
    Eigen::Vector3d pos_pow_exp(4, 6, 8);
    Eigen::Matrix3d rot_pow_exp = Eigen::Matrix3d::Identity();

    // Position
    Test_Pos(pos_pow, pos_pow_exp);

    // Check rotation matrix
    Test_Rot(rot_pow, rot_pow_exp);
}

TEST_F(DualNumberToolsTest, QuaternionPower_Rotatex)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    DualQuaternion q(R_x, pos);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    // Expectations
    Eigen::Vector3d pos_pow_exp = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot_pow_exp = R_x * R_x;

    // Position
    Test_Pos(pos_pow, pos_pow_exp);

    // Check rotation matrix
    Test_Rot(rot_pow, rot_pow_exp);
}

TEST_F(DualNumberToolsTest, QuaternionPower_Rotateyz)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_yz = R_y * R_z;
    DualQuaternion q(R_yz, pos);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    // Expectations
    Eigen::Vector3d pos_pow_exp = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot_pow_exp = R_yz * R_yz;

    // Position
    Test_Pos(pos_pow, pos_pow_exp);

    // Check rotation matrix
    Test_Rot(rot_pow, rot_pow_exp);
}

TEST_F(DualNumberToolsTest, QuaternionPower_RotxPosx)
{
    Eigen::Vector3d pos(2, 0, 0);
    DualQuaternion q(R_x, pos);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    // Expectations
    Eigen::Vector3d pos_pow_exp(4, 0, 0);
    Eigen::Matrix3d rot_pow_exp = R_x * R_x;

    // Position
    Test_Pos(pos_pow, pos_pow_exp);

    // Check rotation matrix
    Test_Rot(rot_pow, rot_pow_exp);
}


TEST_F(DualNumberToolsTest, QuaternionPower_RotyPosx)
{
    Eigen::Vector3d pos(2, 0, 0);
    DualQuaternion q(R_y * R_y, pos);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    std::cout << "position pow: \n" << pos_pow << "\n\n";
    std::cout << "ROT pow: \n" << rot_pow << "\n\n";

    // Expectations
    Eigen::Vector3d pos_pow_exp(2, 0, -2);
    Eigen::Matrix3d rot_pow_exp = R_y * R_y * R_y * R_y;

    // Position
    Test_Pos(pos_pow, pos_pow_exp);

    // Check rotation matrix
    Test_Rot(rot_pow, rot_pow_exp);
}