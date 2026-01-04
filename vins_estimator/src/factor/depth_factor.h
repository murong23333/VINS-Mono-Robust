#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

struct DepthFactor
{
    DepthFactor(double _lidar_depth, double _weight) 
        : lidar_depth(_lidar_depth), weight(_weight) {}

    template <typename T>
    bool operator()(const T* const inv_dep, T* residuals) const
    {
        // Parameter[0] is inverse depth given by VINS
        // Residual = (inv_dep - 1.0 / lidar_depth) * weight
        // Using inverse depth error is more robust/linear for VINS
        
        T inv_d_est = inv_dep[0];
        T inv_d_obs = T(1.0 / lidar_depth);
        
        residuals[0] = (inv_d_est - inv_d_obs) * T(weight);
        
        return true;
    }

    static ceres::CostFunction* Create(const double lidar_depth, const double weight) 
    {
        return (new ceres::AutoDiffCostFunction<DepthFactor, 1, 1>(
            new DepthFactor(lidar_depth, weight)));
    }

    double lidar_depth;
    double weight;
};
