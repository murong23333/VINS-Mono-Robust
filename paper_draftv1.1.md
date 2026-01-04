# Lightweight and Robust Monocular Visual-Inertial Localization with UWB-Fused Drift Correction

**Draft Version**: v1.1
**Date**: 2025-12-25
**Target Venue**: ICRA / IROS
**Status**: Revised Outline with Detailed Methodology

---

## Abstract

Monocular Visual-Inertial Odometry (VIO) is a standard solution for MAV state estimation but suffers from scale ambiguity and accumulated drift over long trajectories. While fusing Ultra-Wideband (UWB) range measurements can constrain drift, naive integration methods are prone to "lockout" failures when VIO estimation deviates significantly. This paper proposes a **robust, tightly-coupled optimization framework** that integrates monocular VIO with UWB ranging. We introduce: (1) A **4-DOF Coordinate Initialization** method that aligns the local VIO frame to the global UWB frame without external magnetometers; (2) A **Heavy-Tailed Loss (Huber)** formulation that maintains optimization gradients under large drifts (>20m), solving the "outlier rejection dilemma"; and (3) An **Adaptive Re-Alignment Mechanism** to recover from mid-flight resets. Experiments on the NTU VIRAL dataset demonstrate an ATE of **0.50m** (RMSE), a **55% improvement** over standard methods, with verified self-recovery capabilities.

---

## 1. Introduction
... (Same as v1.0) ...

---

## 2. Related Work
... (Same as v1.0) ...

---

## 3. Methodology

### 3.1 System Overview
Our system is built upon the VINS-Mono sliding window optimization framework. The state vector is augmented with UWB anchor parameters (optional) and the cost function includes visual, inertial, and UWB residuals.

### 3.2 Formulation
The overall cost function to be minimized is:
$$ \min_{\chi} \left\{ \|r_p - J_p \chi\|^2 + \sum_{k \in \mathcal{B}} \|r_{\mathcal{B}}(\hat{z}^{\mathcal{B}}_k, \chi)\|^2_{\Sigma_{\mathcal{B}}} + \sum_{u \in \mathcal{U}} \rho_{hub}(\|r_{\mathcal{U}}(\hat{z}^{\mathcal{U}}_u, \chi)\|^2_{\Sigma_{\mathcal{U}}}) \right\} $$
Where $\rho_{hub}$ denotes the robust Huber kernel applied specifically to UWB factors.

### 3.3 The Robustness Dilemma & Solution (New Section)
A critical challenge in VIO-UWB fusion is the conflict between **Outlier Rejection** and **Drift Recovery**.
- **The Problem ("Gate Lockout")**: To reject multipath UWB outliers (NLOS), standard filters (e.g., Kalman Filter or Ceres with Cauchy Loss) employ strict gating (e.g., $\chi^2$ test). However, monocular VIO is prone to rapid drift. If the VIO state drifts beyond the gate threshold (e.g., $>10m$) before UWB corrections are applied, the valid UWB measurements are rejected as outliers. The system enters a "Lockout" state where it drifts indefinitely despite having good signal.
- **Our Solution**: We propose a two-pronged approach:
    1.  **Relaxed Gating**: We bypass the strict $\chi^2$ test for UWB measurements during initialization and high-drift states, allowing the optimizer to "see" the large residuals.
    2.  **Huber Loss Gradient**: We replace the standard Cauchy loss with a **Huber Loss** ($\delta=20.0$). Unlike Cauchy loss, which has a vanishing gradient for large errors (effectively ignoring them), Huber loss maintains a linear gradient for distinct outliers. This exerts a constant "pulling force" on the trajectory, allowing the global solver to drag the drifted VIO state back to the correct global position.

### 3.4 4-DOF Coordinate Initialization (New Section)
Visual-Inertial systems operate in a gravity-aligned local frame ($L$), while UWB anchors are defined in a global frame ($G$).
Since IMU gravity alignment determines the Roll and Pitch, we only need to estimate a **4-DOF transformation** (Yaw $\psi$ and Translation $t_{LG}$) to align the frames.
$$ T_{LG} = \begin{bmatrix} R_z(\psi) & t_{LG} \\ 0 & 1 \end{bmatrix} $$
We formulate this as a least-squares problem using the first $N$ UWB measurements during the initialization phase:
$$ \min_{\psi, t_{LG}} \sum_{i=0}^N \| d_i - \| R_z(\psi) p_i^L + t_{LG} - A_k \| \|^2 $$
where $p_i^L$ is the VINS-estimated position in local frame, $A_k$ is the anchor position, and $d_i$ is the measured range.
This initialization is crucial for ensuring the optimization starts within the basin of convergence. We further implement a **check-and-retry** logic: if the alignment RMSE > 1.0m, we discard the result and retry, preventing bad initializations from corrupting the estimator.

### 3.5 Adaptive Re-Alignment
... (Adaptive thresholds for mid-run recovery) ...

---

## 4. Experimental Results

### 4.1 Setup
- **Dataset**: NTU VIRAL.
- **Comparison**:
    - **Method A (Naive)**: Standard Cauchy Loss + Strict Alignment (0.5m).
    - **Method B (Ours)**: Huber Loss + Relaxed Alignment (1.0m).

### 4.2 Qualitative Analysis: Solving the Lockout
**[Figure 2: Trajectory under Drift]**
*(Description)*: We illustrate a specific case (from Sequence `nya_01`, t=40s) where VIO drifted by 12m.
- **Method A (Red)**: The fusion algorithm rejected UWB data ($residual > 10m$), leading to unrecoverable failure.
- **Method B (Green)**: The Huber-based optimizer accepted the large residual and successfully corrected the trajectory within 5 keyframes.

... (Quantitative tables same as v1.0) ...

---

## 5. Conclusion
We presented a robust fusion framework that explicitly addresses the "Lockout" problem in VIO-UWB integration. By combining a 4-DOF initialization strategy with a gradient-preserving robust loss function, our system ensures long-term survivability in GPS-denied environments.
