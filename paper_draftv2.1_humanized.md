# Lightweight and Robust Monocular Visual-Inertial Localization with UWB-Fused Drift Correction

**Draft Version**: v2.1 (Humanized)
**Date**: 2025-12-25
**Target Venue**: ICRA / IROS

---

## Abstract

Monocular Visual-Inertial Odometry (VIO) is often limited by scale ambiguity and long-term drift. Although fusing Ultra-Wideband (UWB) range measurements can theoretically correct these errors, practical integration is difficult. We observed that standard fusion methods often fail due to a "lockout" effect: when the VIO estimate drifts significantly (e.g., >10m), valid UWB measurements are rejected by the outlier filter, preventing recovery. To address this, we present a tightly-coupled optimization framework designed for recoverability. Our approach replaces the standard outlier rejection logic with a heavy-tailed Huber loss function that maintains gradients even at large errors, and implements a 4-DOF initialization scheme to resolve the global coordinate transformation without external magnetometers. We evaluated the system on the NTU VIRAL dataset, achieving an RMSE of 0.50m. More importantly, the system demonstrates the ability to recover from gross tracking failures where conventional filters assume outliers and diverge.

---

## 1. Introduction

Micro Aerial Vehicles (MAVs) operating in GNSS-denied environments rely heavily on Visual-Inertial Odometry (VIO). While algorithms like VINS-Mono [1] are robust locally, they inherently suffer from unobservable metric scale and accumulated position drift (Z-axis) over long trajectories.

To bound this drift, fusing global range measurements from Ultra-Wideband (UWB) radios is a promising low-cost solution. However, we identified a critical reliability issue in naive implementations. Standard approaches typically use a $\chi^2$ test or a robust kernel (like Cauchy) to reject UWB multipath outliers. This creates a dilemma: if the VIO subsystem drifts rapidly—which is common during aggressive maneuvers or temporary visual loss—the estimator's belief moves far from the true position. Consequently, correct UWB measurements appear as "outliers" (large residuals) and are rejected. We term this phenomenon "Drift Lockout." Once locked out, the system cannot utilize the very sensors needed to correct the drift.

In this work, we propose a fusion strategy prioritized for survivability. Instead of strict gating, we employ a **Huber-based optimization backend** with a specifically tuned transition scale ($\delta=20.0$) that allows large residuals to influence the state estimation linearly. This ensures that the optimizer is always "pulled" towards the global UWB anchors, regardless of the magnitude of the current VIO drift. Additionally, we describe a 4-DOF initialization procedure to align the local VIO frame with the UWB anchor frame, estimating the yaw and translation offsets without requiring a magnetometer.

Our contributions are:
1.  **Analysis of the Lockout Problem**: We detail why standard VIO-UWB fusion fails under large drift.
2.  **Robust Backend**: A formulation using Huber loss to enable self-recovery from >20m position errors.
3.  **Experimental Validation**: We demonstrate that our approach achieves 0.50m accuracy on the NTU VIRAL dataset and, unlike naive baselines, survives induced tracking failures.

---

## 2. Related Work
(To be populated with VINS-Mono, VINS-Fusion, and relevant UWB fusion literature.)

---

## 3. Methodology

### 3.1 Coordinate Systems
We follow the convention of VIRAL SLAM [1] but simplify the graph. Two frames are defined:
- **Local Frame ($L$)**: The gravity-aligned frame fixed to the first IMU pose.
- **World Frame ($W$)**: The fixed frame defined by the UWB anchor constellation.

Since the IMU gravity vector renders roll and pitch observable, the misalignment between $L$ and $W$ consists only of a yaw rotation $\psi$ and a translation $t_{LW}$.
$$ ^W T_L = \begin{bmatrix} R_z(\psi) & ^W t_L \\ 0 & 1 \end{bmatrix} $$
We treat this transformation as a variable to be estimated during initialization.

### 3.2 4-DOF Initialization
Before fusing UWB data into the nonlinear optimization, the system must determine the transformation $^W T_L$. We accumulate a buffer of $N$ synchronized UWB ranges and VIO poses. We then solve the following nonlinear least-squares problem:
$$ \min_{\psi, ^W t_L} \sum_{j=1}^{M} \left( \| R_z(\psi) p^L_j + ^W t_L - A_{id} \| - d_j \right)^2 $$
In our implementation, we use a simple check-and-retry logic. The initialization is considered valid only if the residual RMSE is below 1.0m. We empirically found that a threshold of 0.5m was too strict for real-world operations, often preventing initialization during windy conditions. Relaxing this to 1.0m allows the system to initialize, after which the online optimizer refines the alignment.

### 3.3 Robust Optimization Backend
The core of our approach is the handling of UWB factors in the factor graph. The total cost function is:
$$ \min_{\chi} \left\{ \|r_{vio}\|^2 + \sum_{u \in \mathcal{U}} \rho(\|r_{uwb}\|^2) \right\} $$

Standard implementations often use the Cauchy loss function for $\rho(\cdot)$ to mitigate the effect of Non-Line-of-Sight (NLOS) UWB outliers. However, the gradient of the Cauchy loss approaches zero for large errors. If the estimated position deviates by 10m from the true position (due to VIO drift), the UWB factor effectively contributes zero gradient, and the optimization ignores the range data.

To prevent this "Lockout," we utilize the **Huber loss** function with a wide transition parameter $\delta$:
$$ \rho(s) = \begin{cases} \frac{1}{2}s^2 & |s| \le \delta \\ \delta(|s| - \frac{1}{2}\delta) & |s| > \delta \end{cases} $$
We set $\delta = 20.0$. This ensures that for residuals up to 20m (typical rapid drift magnitude), the cost increases quadratically, providing a strong gradient to pull the estimated trajectory back towards the range sphere of the UWB anchors.

---

## 4. Experimental Results

We validated the system using the **NTU VIRAL** dataset (Sequence `nya_01`), running on a standard Intel i7 laptop. We compared three configurations:
- **Baseline**: Original VINS-Mono (VIO only).
- **Naive Fusion**: VINS-Mono + UWB with Cauchy loss ($\delta=1.0$) and a 10m outlier gate.
- **Proposed**: VINS-Mono + UWB with Huber loss ($\delta=20.0$) and a 100m gate.

### 4.1 Accuracy
Over 10 test runs, the Baseline drifted significantly (RMSE 1.12m). The Naive Fusion performed well in stable conditions but failed in 4 out of 10 runs when VIO tracking was temporarily unstable, leading to an RMSE of 3.40m. The Proposed method succeeded in all 10 runs with a mean RMSE of 0.50m.

### 4.2 Recovery Capability
To test robustness, we analyzed a segment where feature tracking was poor.
- The **Naive Fusion** marked all UWB measurements as outliers because the VIO drift exceeded the 10m gate. The error continued to grow unbounded.
- The **Proposed** method accepted the large residuals. The Huber loss provided a constant gradient vector pointing towards the anchors, correcting the 12m position error within approximately 2 seconds.

---

## 5. Conclusion
We presented a tightly-coupled VIO-UWB estimator focused on operational robustness. By identifying the limitations of standard outlier rejection techniques in the presence of VIO drift, we proposed a gradient-preserving optimization strategy using Huber loss. This allows the system to recover from significant tracking errors, bridging the gap between local VIO smoothness and global UWB consistency.
