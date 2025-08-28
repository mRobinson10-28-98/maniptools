#include "UnitQuaternion.hpp"

// Default constructor: identity quaternion
UnitQuaternion::UnitQuaternion()
    : q(Eigen::Quaterniond::Identity()) {}

// Constructor from coefficients
UnitQuaternion::UnitQuaternion(double w, double x, double y, double z)
    : q(w, x, y, z) {
    q.normalize();
}

// Constructor from Eigen quaternion
UnitQuaternion::UnitQuaternion(const Eigen::Quaterniond& quat)
    : q(quat.normalized()) {}

// Constructor from angle-axis
UnitQuaternion::UnitQuaternion(const Eigen::AngleAxisd& aa)
    : q(aa) {
    q.normalize();
}

UnitQuaternion& UnitQuaternion::operator=(const Eigen::Matrix3d& rotation) {
    q = Eigen::Quaterniond(rotation).normalized();
    return *this;
}

// Identity static method
UnitQuaternion UnitQuaternion::Identity() {
    return UnitQuaternion(Eigen::Quaterniond::Identity());
}

// Normalize
void UnitQuaternion::normalize() {
    q.normalize();
}

// Accessors
double UnitQuaternion::w() const { return q.w(); }
double UnitQuaternion::x() const { return q.x(); }
double UnitQuaternion::y() const { return q.y(); }
double UnitQuaternion::z() const { return q.z(); }