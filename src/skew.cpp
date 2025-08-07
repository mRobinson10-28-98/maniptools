#include "skew.hpp"

Eigen::Matrix3d skew3d(Eigen::Vector3d v)
{
    Eigen::Matrix3d skew_symetric {
        {0, -v(2), v(1)}, 
        {v(2), 0, -v(0)}, 
        {-v(1), v(0), 0}
    };
    
    return skew_symetric;
}