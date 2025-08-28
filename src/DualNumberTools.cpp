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
    DualVector d_prod;
    d_prod[0] = other * (*this)[0];
    d_prod[1] = other * (*this)[1];
    d_prod[2] = other * (*this)[2];

    return d_prod;
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