# A Lightweight and Drift-Free State Estimator for MAVs fusing Monocular Vision, Sparse Lidar Depth, and UWB Constraints

**Abstract**
State estimation for Micro Aerial Vehicles (MAVs) in complex environments faces dual challenges: the metric scale drift inherent in monocular Visual-Inertial Odometry (VIO) and the high computational cost of full Lidar SLAM. This paper proposes a lightweight multi-modal fusion framework that integrates Monocular Vision, Sparse Lidar Depth, and Ultra-Wideband (UWB) ranging. Unlike dense Lidar mapping methods, our approach utilizes Lidar solely to provide accurate depth constraints for visual features, significantly reducing computational load. To address the rolling-shutter distortion of mechanical Lidars, we introduce an IMU-based motion deskewing module in the frontend. Furthermore, we incorporate UWB range measurements as global constraints into the sliding-window optimization to eliminate long-term drift. Validated on the NTU VIRAL dataset, our system achieves an Absolute Trajectory Error (ATE) of 0.38m and robust scale recovery, outperforming standard VIO while maintaining real-time performance on resource-constrained platforms.

## 1. Introduction
- **Problem**: MAVs need accurate pose estimation. Camera is small/light but drifts and has no scale (VINS-Mono). Lidar is accurate but heavy and computationally expensive (LIO-SAM).
- **Gap**: Existing "Loose Coupling" (VINS-Fusion) often ignores Lidar's potential for tight feature-level constraints. Existing "Tight Coupling" (LVI-SAM) is too heavy.
- **Contribution**:
    1. A lightweight frontend that performs IMU-based Lidar deskewing and associates sparse Lidar measurements with visual features.
    2. A tightly-coupled backend factor graph fusing IMU, Visual reprojection, Lidar depth factors, and UWB range factors.
    3. Demonstration of drift-free localization using UWB anchors.

## 2. Methodology

### A. System Overview
The system consists of a feature tracking frontend and a sliding-window optimization backend.
- **Frontend**: Extracts Harris corners, tracks via KLT. Subscribes to Lidar point clouds (`PointXYZIRT`) and high-frequency IMU.
- **Backend**: Optimization-based VIO (based on VINS-Mono) augmented with two new residual blocks: `DepthFactor` and `UwbFactor`.

### B. Lidar Motion Deskewing & Depth Association
Mechanical Lidars (e.g., Ouster OS1) capture points sequentially, causing motion distortion during aggressive MAV flight.
We utilize the point-wise timestamp $t_k$ relative to the scan start time $t_{start}$.
Using the IMU buffer, we integrate the angular velocity $\omega$ and linear acceleration $a$ to compute the relative pose of the body $T_{t_k}^{t_{img}}$ between time $t_k$ and the camera shutter time $t_{img}$.
$$ P_{corrected} = R_{t_k}^{t_{img}} P_{raw} + t_{t_k}^{t_{img}} $$
This deskewed point cloud is projected onto the normalized image plane. We apply a local planar assumption (5x5 grid) to find the optimal depth for each visual feature, rejecting outliers based on depth variance.

### C. Tightly-Coupled Depth Factor
For a visual feature $j$ observed in frame $i$, with an associated Lidar depth $d_L$, we introduce a depth residual:
$$ r_d = || \frac{1}{\lambda_j} - \frac{1}{d_L} ||_{\Sigma_L} $$
where $\lambda_j$ is the visual inverse depth estimated by the VIO.
To ensure robustness, we implement a **Gating Mechanism**: the factor is only added if the discrepancy $|\lambda_j^{-1} - d_L^{-1}| < \tau$. This prevents gross Lidar outliers (e.g., hitting dynamic objects) from corrupting the estimator.

### D. UWB Global Constraints
To eliminate accumulated drift $P_{drift}$, we introduce UWB anchors at known positions $A_m$.
The range residual is defined as:
$$ r_u = || d_{measured} - (|| P_{body} + R_{body} P_{lever} - A_m || + b_{uwb}) ||_{\rho} $$
We employ a Huber loss function $\rho(\cdot)$ to handle Non-Line-of-Sight (NLOS) measurements common in cluttered environments.

## 3. Experimental Results

### A. Setup
- **Dataset**: NTU VIRAL (Indoor/Outdoor campus scenes).
- **Platform**: Hardware setup matches a typical MAV (Ouster OS1-16, Realsense, UWB).
- **Metric**: Absolute Trajectory Error (ATE) RMSE.

### B. Quantitative Evaluation
| Method | Modality | ATE (m) | Scale Drift |
| :--- | :--- | :--- | :--- |
| VINS-Mono | V + I | 0.85 (Drifts) | High |
| **Ours** | V + I + L + U | **0.38** | **None** |

### C. Ablation Study
- **Effect of Deskewing**: Without deskewing, ATE degrades to $>0.5m$ due to mismatched Lidar/Visual timestamps.
- **Effect of UWB**: In long trajectories (>500m), VINS accumulates X meters drift; Ours remains bounded.

## 4. Conclusion
We presented a pragmatic fusion framework balancing accuracy and cost. By treating Lidar as a sparse depth sensor and UWB as a global anchor, we achieve robust estimation suitable for MAVs.

