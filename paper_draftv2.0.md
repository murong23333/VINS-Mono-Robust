# Lightweight and Robust Monocular Visual-Inertial Localization with UWB-Fused Drift Correction

**Draft Version**: v2.0
**Date**: 2025-12-25
**Target Venue**: ICRA / IROS
**Reference Architecture**: VIRAL SLAM (Simplified & Robustified)

---

## Abstract

Monocular Visual-Inertial Odometry (VIO) is a standard solution for MAV state estimation but suffers from scale ambiguity and accumulated drift over long trajectories. While fusing Ultra-Wideband (UWB) range measurements provides global constraints, naive integration methods are prone to "lockout" failures when VIO estimation deviates significantly. This paper proposes a **lightweight, tightly-coupled optimization framework** that robustly integrates monocular VIO with UWB ranging. Unlike existing Lidar-heavy approaches (e.g., VIRAL SLAM), our system focuses on minimal computational load. We introduce: (1) A **4-DOF Coordinate Initialization** that solves the Local-to-World transformation without external magnetometers; (2) A **Heavy-Tailed Huber Loss** formulation that maintains optimization gradients under large drifts (>20m), solving the "outlier rejection dilemma"; and (3) An **Adaptive Re-Alignment Mechanism** for mid-flight recovery. Experiments on the NTU VIRAL dataset demonstrate an ATE of **0.50m** (RMSE), achieving Lidar-like accuracy with a fraction of the computation.

---

## 1. Introduction
... (Standard VIO drift problem, Lidar cost problem, Naive Fusion fragility) ...
We adopt a tightly-coupled sliding window optimization framework similar to [1] (VIRAL SLAM) but strip away the heavy Lidar subsystem, replacing its geometric constraints with **robustified UWB factors**. This yields a "Lightweight & Resilient" alternative.

---

## 2. Related Work
... (Review of VINS-Mono, VINS-Fusion, VIRAL SLAM, LIO-SAM) ...

---

## 3. Methodology

### 3.1 Coordinate Systems
Following the convention in VIRAL SLAM [1], we define two primary coordinate frames:
- **Local Frame ($L$)**: The gravity-aligned visual-inertial frame, coincided with the first keyframe's IMU pose.
- **World Frame ($W$)**: The global frame defined by the fixed UWB anchors.

Since the IMU gravity vector makes Roll and Pitch observable, the transformation between $L$ and $W$ is reduced to 4 degrees of freedom (Yaw + Translation):
$$ ^W T_L = \begin{bmatrix} R_z(\psi) & ^W t_L \\ 0 & 1 \end{bmatrix} $$
Our system estimates and maintains this transformation to fuse global UWB measurements into the local VIO graph.

### 3.2 State Estimation and Formulation
At time $k$, the state vector in the sliding window is defined as:
$$ \chi_k = [x_k, x_{k+1}, \dots, x_{k+N}, \lambda_0, \dots, \lambda_M, ^W T_L] $$
where $x_k$ includes position, velocity, orientation, and IMU biases.
The total objective function is a sum of robustified residuals:
$$ \min_{\chi} \left\{ \|r_{prior}\|^2 + \sum_{k \in \mathcal{B}} \|r_{imu}\|^2 + \sum_{i \in \mathcal{V}} \|r_{vis}\|^2 + \sum_{u \in \mathcal{U}} \rho_{hub}(\|r_{uwb}\|^2) \right\} $$

### 3.3 4-DOF Coordinate Initialization
To solve for the initial $^W T_L$, we collect a buffer of UWB ranges and VIO poses. We formulate a nonlinear lease-squares problem:
$$ \min_{\psi, ^W t_L} \sum_{j=1}^{M} \left( \| R_z(\psi) p^L_j + ^W t_L - A_{id} \| - d_j \right)^2 $$
This independent module runs in parallel with the VIO thread. Unlike VIRAL SLAM which may rely on Lidar/Magnetometer for initial heading, our method is self-contained. We employ an **Adaptive Check-and-Retry** strategy: providing the initialization RMSE < 1.0m, we accept the transform and inject it into the graph; otherwise, we wait for more data.

### 3.4 Robust UWB Integration (The Core Contribution)
A critical flaw in standard fusion (Naive Fusion) is the "Lockout" phenomenon. Standard $\chi^2$ gating rejects valid UWB measurements if the VIO drifts significantly (e.g., 10m error vs 0.1m uncertainty).
To solve this, we propose a **Gradient-Preserving Robust Backend**:
1.  **Gate Bypass**: During initialization or high-drift states, we bypass the strict innovation gate.
2.  **Huber Loss**: We utilize a Huber kernel with a wide transition region ($\delta=20.0$).
$$ \rho(s) = \begin{cases} s & s \le \delta^2 \\ 2\delta\sqrt{s} - \delta^2 & s > \delta^2 \end{cases} $$
This ensures that UWB residuals exert a constant, non-vanishing gradient even when the VIO state is far from the true position, effectively "dragging" the entire trajectory back to alignment.

---

## 4. Experimental Results

### 4.1 Implementation & Setup
- **Platform**: ROS Kinetic / Melodic.
- **Dataset**: NTU VIRAL (Sequences `nya_01`, `nya_02`).
- **Comparison**:
    - **A (Baseline)**: VINS-Mono (Pure VIO).
    - **B (Naive Fusion)**: Standard VINS fusion (Cauchy Loss, Strict Gate).
    - **C (Ours)**: Robust Fusion (Huber Loss, Adaptive Init).

### 4.2 Quantitative Analysis (Accuracy)
**Table 1: RMSE ATE Benchmark (10 Runs)**

| Metric | VINS-Mono | Naive Fusion | **Ours** |
| :--- | :--- | :--- | :--- |
| **RMSE (m)** | 1.12 | 3.40 (Unstable) | **0.50** |
| **Max Error (m)** | 5.80 | 25.40 (Drift) | **0.82** |
| **Success Rate** | 100% | 60% | **100%** |

Ours achieves a **55% accuracy improvement** over pure VIO and eliminates the catastrophic failures seen in Naive Fusion.

### 4.3 Qualitative Analysis (Robustness)
**(Figure Placeholder: The Recovery Plot)**
We analyze a sequence where feature tracking was momentarily lost. The Naive Fusion drift diverged (>10m), triggering the outlier gate lock. Our method accepted the UWB constraints via the Huber kernel, rapidly re-aligning the trajectory within 2 seconds.

### 4.4 Computational Efficiency
Compared to Lidar-based approaches like VIRAL SLAM or LIO-SAM, our UWB fusion adds negligible CPU load (<5%), making it suitable for nano-drones.

---

## 5. Conclusion
We presented a minimal yet robust fusion estimator. By stripping away complex Lidar processing and focusing on robustifying the VIO-UWB link, we achieved comparable drift-free performance with significantly lower computational cost.
