#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "DualNumberTools.hpp"

using namespace DualNumberTools;

const double ERROR_BOUND = 1e-6;

TEST(DualNumberTools_test, DualNumberMath)
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

TEST(DualNumberTools_test, DualVectorMath)
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

TEST(DualNumberTools_test, ConstructQuaternions)
{
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::AngleAxisd aa(M_PI/2, z_axis);
    Eigen::Matrix3d rot(aa);
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block(0, 0, 3, 3) = rot;
    transform(0, 3) = pos(0);
    transform(1, 3) = pos(1);
    transform(2, 3) = pos(2);

    DualQuaternion q1(rot, pos);
    DualQuaternion q2(transform);

    // Q1
    Eigen::Vector3d q1_pos_from_function = q1.PositionVector();
    Eigen::Matrix3d q1_rot_from_function = q1.RotationMatrix();
    Eigen::Matrix4d q1_trans_from_function = q1.TransformationMatrix();
    // Q2
    Eigen::Vector3d q2_pos_from_function = q2.PositionVector();
    Eigen::Matrix3d q2_rot_from_function = q2.RotationMatrix();
    Eigen::Matrix4d q2_trans_from_function = q2.TransformationMatrix();

    // Bottom row of SE(3) transformation matrix is <0,0,0,1>
    // Q1
    ASSERT_EQ(q1_trans_from_function(3, 0), 0);
    ASSERT_EQ(q1_trans_from_function(3, 1), 0);
    ASSERT_EQ(q1_trans_from_function(3, 2), 0);
    ASSERT_EQ(q1_trans_from_function(3, 3), 1);
    // Q2
    ASSERT_EQ(q2_trans_from_function(3, 0), 0);
    ASSERT_EQ(q2_trans_from_function(3, 1), 0);
    ASSERT_EQ(q2_trans_from_function(3, 2), 0);
    ASSERT_EQ(q2_trans_from_function(3, 3), 1);

    // Check position
    for (int comp = 0; comp < 3; comp++)
    {
        // Q1
        ASSERT_NEAR(q1_pos_from_function(comp), pos(comp), ERROR_BOUND);
        ASSERT_NEAR(q1_trans_from_function(comp, 3), pos(comp), ERROR_BOUND);
        // Q2
        ASSERT_NEAR(q2_pos_from_function(comp), pos(comp), ERROR_BOUND);
        ASSERT_NEAR(q2_trans_from_function(comp, 3), pos(comp), ERROR_BOUND);
    }

    // Check rotation matrix
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            // Q1
            ASSERT_NEAR(q1_rot_from_function(row, col), rot(row, col), ERROR_BOUND);
            ASSERT_NEAR(q1_trans_from_function(row, col), rot(row, col), ERROR_BOUND);
            // Q2
            ASSERT_NEAR(q2_rot_from_function(row, col), rot(row, col), ERROR_BOUND);
            ASSERT_NEAR(q2_trans_from_function(row, col), rot(row, col), ERROR_BOUND);
        }
    }

}

TEST(DualNumberTools_test, QuaternionProducts_Rotate)
{
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::AngleAxisd aa1(M_PI/2, z_axis);
    Eigen::Matrix3d rot_input(aa1);
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    DualQuaternion q1(rot_input, pos);
    DualQuaternion q_prod = q1 * q1;
    Eigen::Matrix3d rot_calculated = q_prod.RotationMatrix();
    Eigen::Vector3d pos_calculated = q_prod.PositionVector();


    // Expected
    Eigen::AngleAxisd aa2(M_PI, z_axis);
    Eigen::Matrix3d rot_expected = rot_input * rot_input;
    DualQuaternion q_exp(rot_expected, pos);

    // Position
    ASSERT_NEAR(q_prod.dual.w(), q_exp.dual.w(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.x(), q_exp.dual.x(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.y(), q_exp.dual.y(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.z(), q_exp.dual.z(), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(0), 0, ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(1), 0, ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(2), 0, ERROR_BOUND);


    // Orientation
    ASSERT_NEAR(q_prod.real.w(), q_exp.real.w(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.x(), q_exp.real.x(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.y(), q_exp.real.y(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.z(), q_exp.real.z(), ERROR_BOUND);

    // Check rotation matrix
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_calculated(row, col), rot_expected(row, col), ERROR_BOUND);
        }
    }
}


TEST(DualNumberTools_test, QuaternionProducts_Translate)
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
    DualQuaternion q_exp(rot, pos_expected);

    // Position
    ASSERT_NEAR(q_prod.dual.w(), q_exp.dual.w(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.x(), q_exp.dual.x(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.y(), q_exp.dual.y(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.dual.z(), q_exp.dual.z(), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(0), pos_expected(0), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(1), pos_expected(1), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(2), pos_expected(2), ERROR_BOUND);


    // Orientation
    ASSERT_NEAR(q_prod.real.w(), q_exp.real.w(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.x(), q_exp.real.x(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.y(), q_exp.real.y(), ERROR_BOUND);
    ASSERT_NEAR(q_prod.real.z(), q_exp.real.z(), ERROR_BOUND);

    // Check rotation matrix
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_calculated(row, col), rot_expected(row, col), ERROR_BOUND);
        }
    }
}

TEST(DualNumberTools_test, QuaternionProducts_Transform)
{
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::AngleAxisd aa1(M_PI/2, z_axis);
    Eigen::Matrix3d rot1(aa1);
    Eigen::AngleAxisd aa2(M_PI/2, x_axis);
    Eigen::Matrix3d rot2(aa2);

    Eigen::Vector3d pos1(1, 0, 0);
    Eigen::Vector3d pos2(0, 1, 1);

    DualQuaternion q1(rot1, pos1);
    DualQuaternion q2(rot2, pos2);
    DualQuaternion q_prod = q1 * q2;
    Eigen::Vector3d pos_calculated = q_prod.PositionVector();
    Eigen::Matrix3d rot_calculated = q_prod.RotationMatrix();

    // Expected
    Eigen::Vector3d pos_expected(0, 0, 1);
    Eigen::Matrix3d rot_expected = rot1 * rot2;

    DualQuaternion q_exp(rot_expected, pos_expected);

    // Position
    ASSERT_NEAR(pos_calculated(0), pos_expected(0), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(1), pos_expected(1), ERROR_BOUND);
    ASSERT_NEAR(pos_calculated(2), pos_expected(2), ERROR_BOUND);


    // Orientation
    // Check rotation matrix
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_calculated(row, col), rot_expected(row, col), ERROR_BOUND);
        }
    }
}

TEST(DualNumberTools_test, QuaternionConjugate)
{
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::AngleAxisd aa1(M_PI/2, x_axis);
    Eigen::Matrix3d rot(aa1);
    Eigen::Vector3d pos(1, 0, 0);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block(0, 0, 3, 3) = rot;
    transform(0, 3) = pos(0);
    transform(1, 3) = pos(1);
    transform(2, 3) = pos(2);
    DualQuaternion q(transform);

    // Calculations
    DualQuaternion q_conj = q.conjugate();
    Eigen::Vector3d pos_inv = q_conj.PositionVector();
    Eigen::Matrix3d rot_inv = q_conj.RotationMatrix();

    // Expectations
    Eigen::Matrix3d rot_inv_exp = rot.inverse();
    Eigen::Vector3d pos_inv_exp(-1, 0, 0);

    ASSERT_NEAR(pos_inv(0), pos_inv_exp(0), ERROR_BOUND);
    ASSERT_NEAR(pos_inv(1), pos_inv_exp(1), ERROR_BOUND);
    ASSERT_NEAR(pos_inv(2), pos_inv_exp(2), ERROR_BOUND);



    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_inv(row, col), rot_inv_exp(row, col), ERROR_BOUND);
        }
    }
}

TEST(DualNumberTools_test, QuaternionPower)
{
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::AngleAxisd aa1(M_PI/2, x_axis);
    Eigen::Matrix3d rot(aa1);
    Eigen::Vector3d pos(1, 0, 0);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block(0, 0, 3, 3) = rot;
    transform(0, 3) = pos(0);
    transform(1, 3) = pos(1);
    transform(2, 3) = pos(2);
    DualQuaternion q(transform);

    // Calculations
    DualQuaternion q_pow = q.pow(2);
    Eigen::Vector3d pos_pow = q_pow.PositionVector();
    Eigen::Matrix3d rot_pow = q_pow.RotationMatrix();

    std::cout << "Pos power: \n" << pos_pow << "\n\n";
    std::cout << "Rot power: \n" << rot_pow << "\n\n";

    // Expectations
    Eigen::AngleAxisd aa_pow(M_PI, x_axis);
    Eigen::Matrix3d rot_pow_exp(aa_pow);
    Eigen::Vector3d pos_pow_exp(2, 0, 0);

    // Test
    ASSERT_NEAR(pos_pow(0), pos_pow_exp(0), ERROR_BOUND);
    ASSERT_NEAR(pos_pow(1), pos_pow_exp(1), ERROR_BOUND);
    ASSERT_NEAR(pos_pow(2), pos_pow_exp(2), ERROR_BOUND);



    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            ASSERT_NEAR(rot_pow(row, col), rot_pow_exp(row, col), ERROR_BOUND);
        }
    }
}