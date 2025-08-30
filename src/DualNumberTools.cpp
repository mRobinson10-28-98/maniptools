#include <cmath>
#include <iostream>

#include "DualNumberTools.hpp"

using namespace DualNumberTools;

// - - -
// Dual Numbers
// - - -

DualNumber::DualNumber()
{
    real = 0;
    dual = 0;
}

DualNumber::DualNumber(double r, double d) 
{
    real = r;
    dual = d;
}

DualNumber DualNumber::operator+(const DualNumber other) const 
{
    DualNumber d_sum(real + other.real, dual + other.dual);
    return d_sum;
}

DualNumber DualNumber::operator-(const DualNumber other) const 
{
    DualNumber d_min(real - other.real, dual - other.dual);
    return d_min;
}

DualNumber DualNumber::operator*(const DualNumber other) const 
{
    DualNumber d_prod(real * other.real, real * other.dual + dual * other.real);
    return d_prod;
}

bool DualNumber::operator==(const DualNumber other) const
{
    return (real == other.real) && (dual == other.dual);
}

// - - -
// Dual Vectors
// - - -

DualVector::DualVector()
{
    real = Eigen::Vector3d::Zero();
    dual = Eigen::Vector3d::Zero();
}

DualVector::DualVector(Eigen::Vector3d r, Eigen::Vector3d d) 
{
    real = r;
    dual = d;
}

DualVector::DualVector(DualNumber dn1, DualNumber dn2, DualNumber dn3)
{
    real(0) = dn1.real;
    dual(0) = dn1.dual;
    real(1) = dn2.real;
    dual(1) = dn2.dual;
    real(2) = dn3.real;
    dual(2) = dn3.dual;
}

DualNumber DualVector::operator[](std::size_t index) const
{
    if (index >= 3) throw std::out_of_range("Dual Vector index out of range");
    DualNumber d(real[index], dual[index]);

    return d;
}

DualVector DualVector::operator+(const DualVector other) const 
{
    DualVector d_sum(real + other.real, dual + other.dual);
    return d_sum;
}

DualVector DualVector::operator-(const DualVector other) const 
{
    DualVector d_min(real - other.real, dual - other.dual);
    return d_min;
}

// Dual vector dot product = dual number
DualNumber DualVector::operator*(const DualVector other) const 
{
    DualNumber d_prod;
    d_prod = ((*this)[0] * other[0]) + ((*this)[1] * other[1]) + ((*this)[2] * other[2]);
    return d_prod;
}

// Dual number * dual vector product = dual vector
DualVector DualVector::operator*(const DualNumber other) const 
{
    DualVector d_prod(
        other * (*this)[0],
        other * (*this)[1],
        other * (*this)[2]
    );

    return d_prod;
}

bool DualVector::operator==(const DualVector other) const
{
    return ((*this)[0] == other[0]) && ((*this)[1] == other[1]) && ((*this)[2] == other[2]);
}

DualVector DualVector::cross(DualVector d2) const
{
    DualVector d_cross;
    d_cross[0] = (*this)[1] * d2[2] - (*this)[2] * d2[1];
    d_cross[1] = (*this)[0] * d2[2] - (*this)[2] * d2[0];
    d_cross[2] = (*this)[0] * d2[1] - (*this)[1] * d2[0];

    return d_cross;
}

// - - -
// Dual Quaternions
// - - -

DualQuaternion::DualQuaternion()
{
    real = Eigen::Quaterniond::Identity();
    dual = Eigen::Quaterniond::Identity();
}

DualQuaternion::DualQuaternion(Eigen::Quaterniond r, Eigen::Quaterniond d) 
{
    real = r;
    dual = d;
}

DualQuaternion::DualQuaternion(DualNumber n, DualVector v)
{
    real.w() = n.real;
    real.vec() = v.real;
    dual.w() = n.dual;
    dual.vec() = v.dual;

    real = ScalarMultiplyQuaternion(real, (1/real.norm()));
}

DualQuaternion::DualQuaternion(Eigen::Matrix3d rot, Eigen::Vector3d pos)
{
    Eigen::AngleAxisd angle_axis(rot);
    real = Eigen::Quaterniond(angle_axis);
    Eigen::Quaterniond p_quat {0, pos[0], pos[1], pos[2]};
    dual = ScalarMultiplyQuaternion(p_quat * real, 0.5);
}

DualQuaternion::DualQuaternion(Eigen::Matrix4d transform)
{
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Vector3d pos(transform(0,3), transform(1,3), transform(2,3));

    // Copy from above constructor bc i cant get delegation to work so...
    Eigen::AngleAxisd angle_axis(rot);
    real = Eigen::Quaterniond(angle_axis);
    Eigen::Quaterniond p_quat {0, pos[0], pos[1], pos[2]};
    dual = ScalarMultiplyQuaternion(p_quat * real, 0.5);
}

DualQuaternion DualQuaternion::operator+(const DualQuaternion other) const
{
    DualQuaternion q(
        AddQuaternions(real, other.real), 
        AddQuaternions(dual, other.dual)
    );
    return q;
}

DualQuaternion DualQuaternion::operator-(const DualQuaternion other) const
{
    Eigen::Quaterniond q_real_neg = Eigen::Quaterniond(-other.real.w(), -other.real.x(), -other.real.y(), -other.real.z());
    Eigen::Quaterniond q_dual_neg = Eigen::Quaterniond(-other.dual.w(), -other.dual.x(), -other.dual.y(), -other.dual.z());
    DualQuaternion q_neg(q_real_neg, q_dual_neg);
    DualQuaternion q(
        AddQuaternions(real, q_neg.real), 
        AddQuaternions(dual, q_neg.dual)
    );    
    return q;
}

DualQuaternion DualQuaternion::operator*(const DualQuaternion other) const
{
    DualQuaternion q(
        real * other.real,
        AddQuaternions(real * other.dual, dual * other.real)
    );

    return q;
}

DualQuaternion DualQuaternion::pow(double t) const
{

    // // Step 1: Logarithm of dq
    // Eigen::Quaterniond qr = real;
    // Eigen::Quaterniond qd = dual;

    // // Normalize qr to ensure it's unit
    // qr.normalize();

    // double theta = 2 * std::acos(qr.w());  // angle of rotation
    // Eigen::Vector3d axis;
    // if (std::abs(theta) < 1e-8) 
    // {
    //     axis = Eigen::Vector3d::UnitX();  // default axis
    // } 
    // else 
    // {
    //     axis = qr.vec().normalized();
    // }

    // // Log of real part
    // Eigen::Vector3d log_qr = axis * theta;

    // // Compute the translation from dual part
    // Eigen::Quaterniond trans_quat = ScalarMultiplyQuaternion(qd * qr.conjugate(), 2.0);
    // Eigen::Vector3d translation(trans_quat.x(), trans_quat.y(), trans_quat.z());

    // // Construct the logarithm of the dual quaternion
    // Eigen::Vector3d v = log_qr;
    // Eigen::Vector3d d = 0.5 * (translation);

    // // Step 2: Multiply by scalar t
    // Eigen::Vector3d v_t = v * t;
    // Eigen::Vector3d d_t = d * t;

    // // Step 3: Exponential map
    // double theta_t = v_t.norm();
    // Eigen::Quaterniond qr_t;
    // if (theta_t < 1e-8) {
    //     qr_t = Eigen::Quaterniond(1, 0, 0, 0);  // Identity rotation
    // } else {
    //     Eigen::Vector3d axis_t = v_t.normalized();
    //     qr_t = Eigen::AngleAxisd(theta_t, axis_t);
    // }

    // Eigen::Quaterniond qd_t(0, d_t.x(), d_t.y(), d_t.z());
    // qd_t = ScalarMultiplyQuaternion(qd_t * qr_t, 0.5);

    // return DualQuaternion(qr_t, qd_t);

    Eigen::Vector3d x_axis(1, 0, 0);
    double theta = 2 * std::acos(real.w());
    Eigen::Vector3d u = x_axis;

    if (std::abs(theta) < 1e-6)
    {
        u = x_axis;
    }
    else
    {
        Eigen::Vector3d u = real.vec() * (1/std::sin(theta/2)); 
    }

    Eigen::Quaterniond p_quat = ScalarMultiplyQuaternion(dual * real.conjugate(), 2);
    Eigen::Vector3d p(p_quat.x(), p_quat.y(), p_quat.z());
    std::cout << "TEST p: \n" << p << "\n\n";
    double d = p.dot(u);
    double h = d / theta;
    // Eigen::Vector3d m = 0.5 * (p.cross(u) + h * u);
    Eigen::Vector3d m = 0.5 * p.cross(u) + (p - d * u) * (1/std::tan(theta/2));

    DualVector u_hat;
    u_hat.real = u;
    u_hat.dual = m;

    DualNumber A_n, A_vn;
    A_n.real = std::cos((t * theta)/2);
    A_n.dual = -((t * d)/2) * std::sin((t * theta)/2);
    A_vn.real = std::sin((t * theta)/2);
    A_vn.dual = ((t * d)/2) * std::cos((t * theta)/2);

    DualVector A_v = u_hat * A_vn;

    std::cout << "theta: \n" << theta << "\n\n";
    std::cout << "u \n" << u << "\n\n";
    std::cout << "m \n" << m << "\n\n";
    std::cout << "d: \n" << d << "\n\n";
    std::cout << "h: \n" << h << "\n\n";
    std::cout << "A_n.real \n" << A_n.real << "\n\n";
    std::cout << "A_vn.real \n" << A_vn.real << "\n\n";
    std::cout << "A_v.real \n" << A_v.real << "\n\n";

    DualQuaternion q(A_n, A_v);
    return q;
}


// // Accessors
DualNumber DualQuaternion::scalar() const
{
    DualNumber n(real.w(), dual.w());
    return n;
}

DualVector DualQuaternion::vector() const
{
    DualVector v(real.vec(), dual.vec());
    return v;
}

DualQuaternion DualQuaternion::conjugate() const
{
    DualQuaternion q(real.conjugate(), dual.conjugate());
    return q;
}

// Pose accessors
Eigen::Vector3d DualQuaternion::PositionVector() const
{
    Eigen::Quaterniond q = ScalarMultiplyQuaternion(dual, 2) * real.conjugate();
    Eigen::Vector3d p {q.x(), q.y(), q.z()};
    return p;
}

Eigen::Matrix3d DualQuaternion::RotationMatrix() const
{
    double theta = 2 * std::acos(real.w());

    // dont divide by 0 when theta = 0
    if(abs(theta) < 1e-6)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Vector3d axis = real.vec() * (1/std::sin(theta/2));
        Eigen::AngleAxis angle_axis(theta, axis);
        return angle_axis.toRotationMatrix();
    }
}

Eigen::Matrix4d DualQuaternion::TransformationMatrix() const
{
    Eigen::Vector3d pos = PositionVector();
    Eigen::Matrix3d rot = RotationMatrix();
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block(0, 0, 3, 3) = rot;
    transform(0, 3) = pos(0);
    transform(1, 3) = pos(1);
    transform(2, 3) = pos(2);

    return transform;
}

// - - -
// Quaternion math
// - - -

Eigen::Quaterniond DualNumberTools::AddQuaternions(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    Eigen::Quaterniond q;
    q.w() = q1.w() + q2.w();
    q.vec() = q1.vec() + q2.vec();

    return q;
}

Eigen::Quaterniond DualNumberTools::ScalarMultiplyQuaternion(Eigen::Quaterniond q, double d)
{
    return Eigen::Quaternion(q.w() * d, q.x() * d, q.y() * d, q.z() * d);
}
