#include "initial_uwb_alignment.h"
#include <ros/console.h>
#include <ceres/ceres.h>

// 4-DOF Alignment Cost Functor
// Optimization variables:
//   yaw: double[1]
//   t:   double[3] (x, y, z)
// Model:
//   P_world = R_z(yaw) * P_vins + t
//   Residual = | P_world - P_anchor | - measure
struct UwbAlignmentFactor 
{
    UwbAlignmentFactor(double range, const Vector3d &p_imu, const Vector3d &p_anchor)
        : range_(range), p_imu_(p_imu), p_anchor_(p_anchor) {}

    template <typename T>
    bool operator()(const T* const yaw, const T* const t, T* residuals) const 
    {
        // Construct Rotation Matrix R_z(yaw)
        T cy = ceres::cos(yaw[0]);
        T sy = ceres::sin(yaw[0]);
        
        // Rotate P_imu (manually to avoid Eigen template issues if minimal)
        // Rz = [cy -sy 0; sy cy 0; 0 0 1]
        T p_imu_rot_x = cy * T(p_imu_.x()) - sy * T(p_imu_.y());
        T p_imu_rot_y = sy * T(p_imu_.x()) + cy * T(p_imu_.y());
        T p_imu_rot_z = T(p_imu_.z());

        // Translate
        T p_world_x = p_imu_rot_x + t[0];
        T p_world_y = p_imu_rot_y + t[1];
        T p_world_z = p_imu_rot_z + t[2];

        // Anchor
        T anc_x = T(p_anchor_.x());
        T anc_y = T(p_anchor_.y());
        T anc_z = T(p_anchor_.z());

        // Distance
        T dx = p_world_x - anc_x;
        T dy = p_world_y - anc_y;
        T dz = p_world_z - anc_z;
        T dist_sq = dx*dx + dy*dy + dz*dz;
        T dist = ceres::sqrt(dist_sq);

        residuals[0] = dist - T(range_);
        return true;
    }

    static ceres::CostFunction* Create(double range, const Vector3d &p_imu, const Vector3d &p_anchor) 
    {
        return new ceres::AutoDiffCostFunction<UwbAlignmentFactor, 1, 1, 3>(
            new UwbAlignmentFactor(range, p_imu, p_anchor));
    }

    double range_;
    Vector3d p_imu_, p_anchor_;
};

bool solveUwbAlignment(
    Vector3d* Ps, Matrix3d* Rs, std_msgs::Header* Headers, int frame_count,
    const vector<double>& uwb_times, 
    const vector<double>& uwb_ranges, 
    const vector<int>& uwb_ids,
    const map<int, Vector3d>& anchors,
    double& yaw_out, Vector3d& t_out)
{
    // 1. Build Problem
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); // Robust loss for outliers

    double yaw_param[1] = {0.0};
    double t_param[3] = {0.0, 0.0, 0.0};

    int measurement_count = 0;

    // 2. Data Association
    // For every UWB measurement, find the closest VINS frame
    for(size_t k = 0; k < uwb_times.size(); k++)
    {
        double t_uwb = uwb_times[k];
        double range = uwb_ranges[k];
        int id = uwb_ids[k];

        // Check anchor existence
        if(anchors.find(id) == anchors.end()) continue;
        
        // Sanity Check: Reject gross outliers (e.g. 47m measurement in a small room)
        if(range > 20.0 || range < 0.0) continue;

        Vector3d p_anc = anchors.at(id);

        // Find closest frame
        int closest_i = -1;
        double min_dt = 0.05; // 50ms tolerance

        for(int i = 0; i <= frame_count; i++)
        {
            double t_vins = Headers[i].stamp.toSec();
            double dt = std::abs(t_uwb - t_vins);
            if(dt < min_dt)
            {
                min_dt = dt;
                closest_i = i;
            }
        }

        if(closest_i != -1)
        {
            problem.AddResidualBlock(
                UwbAlignmentFactor::Create(range, Ps[closest_i], p_anc),
                loss_function,
                yaw_param,
                t_param
            );
            measurement_count++;
        }
    }

    if(measurement_count < 5) // Minimal constraints check
    {
        ROS_WARN("[UWB Align] Not enough measurements for alignment: %d", measurement_count);
        return false;
    }

    // 3. Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double rmse = std::sqrt(2.0 * summary.final_cost / measurement_count);

    if (summary.termination_type == ceres::CONVERGENCE || rmse < 1.0) 
    {
        yaw_out = yaw_param[0];
        t_out = Vector3d(t_param[0], t_param[1], t_param[2]);
        ROS_INFO("[UWB Align] Converged! Yaw: %.3f rad, T: %.2f %.2f %.2f. Cost: %.2f -> %.2f (RMSE: %.3f m, N=%d)", 
                 yaw_out, t_out.x(), t_out.y(), t_out.z(), summary.initial_cost, summary.final_cost, rmse, measurement_count);
        return true;
    }
    else
    {
        ROS_WARN("[UWB Align] Failed to converge. Type: %d, Cost: %.2f, RMSE: %.3f m. Threshold: 1.0m", 
                 summary.termination_type, summary.final_cost, rmse);
        return false;
    }
}
