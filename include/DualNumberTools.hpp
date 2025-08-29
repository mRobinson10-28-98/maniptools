#ifndef _DUAL_NUMBER_TOOLS_HPP_
#define _DUAL_NUMBER_TOOLS_HPP_

#include <Eigen/Dense>

namespace DualNumberTools
{
    struct DualNumber
    {
        DualNumber();

        DualNumber(double r, double d);

        DualNumber operator+(const DualNumber other) const;

        DualNumber operator-(const DualNumber other) const;

        DualNumber operator*(const DualNumber other) const;

        bool operator==(const DualNumber other) const;

        double real;
        double dual;
    };

    struct DualVector
    {
        DualVector();

        DualVector(Eigen::Vector3d r, Eigen::Vector3d d);

        DualNumber operator[](std::size_t index) const;

        DualVector operator+(const DualVector other) const;

        DualVector operator-(const DualVector other) const;

        // Dual vector dot product = dual number
        DualNumber operator*(const DualVector other) const;

        // Dual number * dual vector product = dual vector
        DualVector operator*(const DualNumber other) const;

        bool operator==(const DualVector other) const;

        DualVector cross(DualVector d) const;

        Eigen::Vector3d real;
        Eigen::Vector3d dual;
    };

    struct DualQuaternion
    {
        DualQuaternion();

        DualQuaternion(Eigen::Quaterniond r, Eigen::Quaterniond d);
        DualQuaternion(DualNumber n, DualVector v);

        DualQuaternion operator+(const DualQuaternion other) const;
        DualQuaternion operator-(const DualQuaternion other) const;
        DualQuaternion operator*(const DualQuaternion other) const;

        DualQuaternion pow(double t) const;

        // // Accessors
        DualNumber scalar() const;
        DualVector vector() const;
        DualQuaternion conjugate() const;

        // Pose accessors
        Eigen::Vector3d PositionVector() const;
        Eigen::Matrix3d RotationMatrix() const;
        Eigen::Matrix4d TransformationMatrix() const;

        Eigen::Quaterniond real;
        Eigen::Quaterniond dual;
    };

    Eigen::Quaterniond AddQuaternions(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
    Eigen::Quaterniond ScalarMultiplyQuaternion(Eigen::Quaterniond q, double d);

} // end DualNumberTools

#endif // _DUAL_NUMBER_TOOLS_HPP_