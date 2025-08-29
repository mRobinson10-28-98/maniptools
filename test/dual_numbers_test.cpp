#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "DualNumberTools.hpp"

using namespace DualNumberTools;

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