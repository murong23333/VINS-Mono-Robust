#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <std_msgs/Header.h>

using namespace Eigen;
using namespace std;

/**
 * @brief Solves for 4-DOF alignment (Yaw + Translation) between VINS and UWB
 * 
 * @param Ps VINS positions array (index 0 to frame_count)
 * @param Rs VINS rotations array
 * @param Headers VINS headers (for timestamps)
 * @param frame_count Number of active frames
 * @param uwb_times Vector of UWB timestamps
 * @param uwb_ranges Vector of UWB ranges
 * @param uwb_ids Vector of UWB anchor IDs
 * @param anchors Map of Anchor ID to Position
 * @param yaw_out Output Yaw
 * @param t_out Output Translation
 * @return true if successful
 */
bool solveUwbAlignment(
    Vector3d* Ps, Matrix3d* Rs, std_msgs::Header* Headers, int frame_count,
    const vector<double>& uwb_times, 
    const vector<double>& uwb_ranges, 
    const vector<int>& uwb_ids,
    const map<int, Vector3d>& anchors,
    double& yaw_out, Vector3d& t_out);
