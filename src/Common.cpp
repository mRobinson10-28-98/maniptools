#include "Common.hpp"

Eigen::Matrix3d skew3d(Eigen::Vector3d v)
{
    Eigen::Matrix3d skew_symetric {
        {0, -v(2), v(1)}, 
        {v(2), 0, -v(0)}, 
        {-v(1), v(0), 0}
    };
    
    return skew_symetric;
}

Eigen::Matrix<double, 3, 4> skewQuat(Eigen::Quaterniond q)
{
    Eigen::Matrix<double, 3, 4> skew_symetric {
        {-q.x(), q.w(), q.z(), -q.y()}, 
        {-q.y(), -q.z(), q.w(), q.x()}, 
        {-q.z(), q.y(), -q.x(), q.w()}
    };
    
    return skew_symetric;
}

// template<typename MatrixType>
Eigen::MatrixXd PseudoInverse(Eigen::MatrixXd mat)
{
    double tolerance = 1e-6;
    // SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto& singularValues = svd.singularValues();
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    // Compute reciprocal of non-zero singular values
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            S_inv(i, i) = 1.0 / singularValues(i);
        }
    }

    // Return V * S⁺ * Uᵗ
    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}