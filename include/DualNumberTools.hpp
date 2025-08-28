#ifndef _DUAL_NUMBER_TOOLS_HPP_
#define _DUAL_NUMBER_TOOLS_HPP_

#include <Eigen/Dense>

namespace DualNumberTools
{
    struct DualNumber
    {
        DualNumber()
        {
            real = 0;
            dual = 0;
        }

        DualNumber(double r, double d) 
        {
            real = r;
            dual = d;
        }

        DualNumber operator+(const DualNumber other) const 
        {
            DualNumber d_sum(real + other.real, dual + other.dual);
            return d_sum;
        }

        DualNumber operator-(const DualNumber other) const 
        {
            DualNumber d_min(real - other.real, dual - other.dual);
            return d_min;
        }

        DualNumber operator*(const DualNumber other) const 
        {
            DualNumber d_prod(real * other.real, real * other.dual + dual * other.real);
            return d_prod;
        }

        double real;
        double dual;
    };

    struct DualVector
    {
        DualVector()
        {
            real = Eigen::Vector3d::Zero();
            dual = Eigen::Vector3d::Zero();
        }

        DualVector(Eigen::Vector3d r, Eigen::Vector3d d) 
        {
            real = r;
            dual = d;
        }

        DualNumber operator[](std::size_t index) const
        {
            if (index >= 3) throw std::out_of_range("Dual Vector index out of range");
            DualNumber d(real[index], dual[index]);

            return d;
        }

        DualVector operator+(const DualVector other) const 
        {
            DualVector d_sum(real + other.real, dual + other.dual);
            return d_sum;
        }

        DualVector operator-(const DualVector other) const 
        {
            DualVector d_min(real - other.real, dual - other.dual);
            return d_min;
        }

        // Dual vector dot product = dual number
        DualNumber operator*(const DualVector other) const 
        {
            DualNumber d_prod;
            d_prod = ((*this)[0] * other[0]) + ((*this)[1] * other[1]) + ((*this)[2] * other[2]);
            return d_prod;
        }

        // Dual number * dual vector product = dual vector
        DualVector operator*(const DualNumber other) const 
        {
            DualNumber d_prod;
            d_prod = ((*this)[0] * other[0]) + ((*this)[1] * other[1]) + ((*this)[2] * other[2]);
            return d_prod;
        }


        Eigen::Vector3d real;
        Eigen::Vector3d dual;
    };

    DualVector DualVectorCrossProduct(DualVector d1, DualVector d2);

    // DualVector DualNumberDualVectorProduct(DualNumber d1, DualVector d2);

    struct DualQuaternion
    {
        DualQuaternion()
        {
            real = Eigen::Quaterniond::Identity();
            dual = Eigen::Quaterniond::Identity();
        }

        DualQuaternion(Eigen::Quaterniond r, Eigen::Quaterniond d) 
        {
            real = r;
            dual = d;
        }

        Eigen::Quaterniond real;
        Eigen::Quaterniond dual;
    };


} // end DualNumberTools

#endif // _DUAL_NUMBER_TOOLS_HPP_