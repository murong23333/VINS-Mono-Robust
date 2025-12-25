#pragma once

#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

struct UwbFactorFunctor
{
    UwbFactorFunctor(const Eigen::Vector3d &_uwb_anchor, const Eigen::Vector3d &_tag_offset, double _measurement, double _noise_std)
        : uwb_anchor(_uwb_anchor), tag_offset(_tag_offset), measurement(_measurement), noise_std(_noise_std)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residual) const
    {
        // pose[0,1,2] is P
        // pose[3,4,5,6] is Q (x, y, z, w) - Wait, VINS standard is usually this?
        // Let's check VINS parameter blocks. 
        // parameters[0][0..2] is P.
        // parameters[0][3..6] is Q.
        // Ceres Quaternion is [w, x, y, z]. VINS uses Eigen [x, y, z, w] or similar.
        // In parameters.cpp/estimator.cpp:
        // para_Pose[i][0] = P.x
        // para_Pose[i][3] = Q.x, [4]=y, [5]=z, [6]=w.
        // SO VINS uses [x,y,z,w].
        // ceres::QuaternionRotatePoint expects [w, x, y, z] order by default?
        // No, Ceres Jet arrays.
        // ceres::QuaternionRotatePoint document says "The quaternion is [w, x, y, z]".
        
        T p[3] = {pose[0], pose[1], pose[2]};
        T q[4]; // Convert [x,y,z,w] to [w,x,y,z] for Ceres
        q[0] = pose[6]; // w
        q[1] = pose[3]; // x
        q[2] = pose[4]; // y
        q[3] = pose[5]; // z
        
        T t_offset[3] = {T(tag_offset.x()), T(tag_offset.y()), T(tag_offset.z())};
        T p_tag_offset[3];
        
        ceres::QuaternionRotatePoint(q, t_offset, p_tag_offset);
        
        T p_tag[3];
        p_tag[0] = p[0] + p_tag_offset[0];
        p_tag[1] = p[1] + p_tag_offset[1];
        p_tag[2] = p[2] + p_tag_offset[2];
        
        T dist = sqrt(pow(p_tag[0] - T(uwb_anchor.x()), 2) +
                      pow(p_tag[1] - T(uwb_anchor.y()), 2) +
                      pow(p_tag[2] - T(uwb_anchor.z()), 2));
                      
        residual[0] = (dist - T(measurement)) / T(noise_std);
        
        return true;
    }

    Eigen::Vector3d uwb_anchor;
    Eigen::Vector3d tag_offset;
    double measurement;
    double noise_std;
};

struct UwbAnchorFactorFunctor
{
    UwbAnchorFactorFunctor(const Eigen::Vector3d &_tag_offset, double _measurement, double _noise_std)
        : tag_offset(_tag_offset), measurement(_measurement), noise_std(_noise_std)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, const T *const anchor_pos, T *residual) const
    {
        // pose[0..2] is P
        // pose[3..6] is Q (x,y,z,w)
        
        T p[3] = {pose[0], pose[1], pose[2]};
        T q[4]; // Convert [x,y,z,w] to [w,x,y,z] for Ceres
        q[0] = pose[6]; // w
        q[1] = pose[3]; // x
        q[2] = pose[4]; // y
        q[3] = pose[5]; // z
        
        // Anchor from parameter block
        T uwb_anchor[3] = {anchor_pos[0], anchor_pos[1], anchor_pos[2]};

        T t_offset[3] = {T(tag_offset.x()), T(tag_offset.y()), T(tag_offset.z())};
        T p_tag_offset[3];
        
        ceres::QuaternionRotatePoint(q, t_offset, p_tag_offset);
        
        T p_tag[3];
        p_tag[0] = p[0] + p_tag_offset[0];
        p_tag[1] = p[1] + p_tag_offset[1];
        p_tag[2] = p[2] + p_tag_offset[2];
        
        T dist = sqrt(pow(p_tag[0] - uwb_anchor[0], 2) +
                      pow(p_tag[1] - uwb_anchor[1], 2) +
                      pow(p_tag[2] - uwb_anchor[2], 2));
                      
        residual[0] = (dist - T(measurement)) / T(noise_std);
        
        return true;
    }

    Eigen::Vector3d tag_offset;
    double measurement;
    double noise_std;
};
