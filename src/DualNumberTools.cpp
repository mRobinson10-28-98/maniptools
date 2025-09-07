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
    real = Eigen::Quaterniond(Eigen::AngleAxisd(rot));
    Eigen::Quaterniond p_quat {0, pos[0], pos[1], pos[2]};
    dual = ScalarMultiplyQuaternion(p_quat * real, 0.5);
}

DualQuaternion::DualQuaternion(Eigen::Matrix4d transform) : 
    DualQuaternion(Eigen::Matrix3d(transform.block(0, 0, 3, 3)), Eigen::Vector3d(transform(0, 3), transform(1, 3), transform(2, 3)))
{
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
    // 1. Dual-Quaternion to Plucker Screw coordinates
    Eigen::Vector3d p = PositionVector();
    double w_r = real.w();
    Eigen::Vector3d v_r = real.vec();
    Eigen::Vector3d v_d = dual.vec();

    Eigen::Vector3d x_axis(1, 0, 0);
    double theta = 2 * std::acos(real.w());

    double d, v_r_mag;
    Eigen::Vector3d u, m; 

    if (std::abs(theta) < 1e-6)
    {
        Eigen::Vector3d p_t = p * t;
        return DualQuaternion(Eigen::Matrix3d::Identity(), p_t);
    }
    else
    {
        v_r_mag = std::sqrt(v_r.dot(v_r));
        d = -2 * dual.w() / v_r_mag;
        u = v_r / v_r_mag; 
        m = (v_d - u * ((d * w_r)/2)) / v_r_mag;

        DualVector u_hat(u, m);

        DualNumber A_n, A_vn;
        A_n.real = std::cos((t * theta)/2);
        A_n.dual = -((t * d)/2) * std::sin((t * theta)/2);
        A_vn.real = std::sin((t * theta)/2);
        A_vn.dual = ((t * d)/2) * std::cos((t * theta)/2);

        DualVector A_v = u_hat * A_vn;
        DualQuaternion q(A_n, A_v);

        // std::cout << "Starting P: \n" << p << "\n\n";
        // std::cout << "theta: \n" << theta << "\n\n";
        // std::cout << "u \n" << u << "\n\n";
        // std::cout << "m \n" << m << "\n\n";
        // std::cout << "d: \n" << d << "\n\n";
        // std::cout << "A_n.real \n" << A_n.real << "\n\n";
        // std::cout << "A_vn.real \n" << A_vn.real << "\n\n";
        // std::cout << "A_vn.dual \n" << A_vn.dual << "\n\n";
        // std::cout << "A_v.real \n" << A_v.real << "\n\n";
        // std::cout << "A_v.dual \n" << A_v.dual << "\n\n";
        // std::cout << "A_v = \n";
        // std::cout << "(" << u(0) << " + " << m(0) << ")" << " * " << "(" << A_vn.real << " + " << A_vn.dual << ")\n";
        // std::cout << "(" << u(1) << " + " << m(1) << ")\n";
        // std::cout << "(" << u(2) << " + " << m(2) << ")\n";
        // std::cout << "q.real \n" << q.real << "\n\n";
        // std::cout << "Rot: \n" << q.RotationMatrix() << "\n\n";
        // std::cout << "q.dual \n" << q.dual << "\n\n";
        // std::cout << "Pos vector: \n" << q.PositionVector() << "\n\n";

        return q;
    }
}


// // Accessors
DualNumber DualQuaternion::scalar() const
{
    return DualNumber(real.w(), dual.w());
}

DualVector DualQuaternion::vector() const
{
    return DualVector(real.vec(), dual.vec());
}

DualQuaternion DualQuaternion::conjugate() const
{
    return DualQuaternion(real.conjugate(), dual.conjugate());
}

// Pose accessors
Eigen::Vector3d DualQuaternion::PositionVector() const
{
    Eigen::Quaterniond q = ScalarMultiplyQuaternion(dual, 2) * real.conjugate();
    return Eigen::Vector3d(q.x(), q.y(), q.z());
}

Eigen::Matrix3d DualQuaternion::RotationMatrix() const
{
    double theta = 2 * std::acos(real.w());

    // dont divide by 0 when theta = 0
    if(std::abs(theta) < 1e-6)
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
