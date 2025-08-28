
#ifndef _UNIT_QUATERNION_HPP_
#define _UNIT_QUATERNION_HPP_

#include <Eigen/Dense>

class UnitQuaternion {
public:
    // Constructors
    UnitQuaternion(); // Identity
    UnitQuaternion(double w, double x, double y, double z);
    UnitQuaternion(const Eigen::Quaterniond& quat);
    UnitQuaternion(const Eigen::AngleAxisd& aa);

    // Overload assignment from a rotation matrix
    UnitQuaternion& operator=(const Eigen::Matrix3d& rotation);

    // Static identity
    static UnitQuaternion Identity();

    // Normalize quaternion
    void normalize();

    Eigen::Quaterniond skew();

    // Accessors
    double w() const;
    double x() const;
    double y() const;
    double z() const;

private:
    // Underlying quaternion
    Eigen::Quaterniond q;
};

#endif //_UNIT_QUATERNION_HPP_