# Lightweight and Robust Monocular Visual-Inertial Localization with UWB-Fused Drift Correction

**Draft Version**: v2.2 (Academic Rigor)
**Date**: 2025-12-25
**Target Venue**: ICRA / IROS

---

## Abstract

Monocular Visual-Inertial Odometry (VIO) is intrinsically limited by scale ambiguity and unbounded drift over extended trajectories. While the fusion of Ultra-Wideband (UWB) ranging constraints offers a theoretical solution to drift, naive integration strategies frequently exhibit instability in practical deployments. Specifically, we identify a failure mode termed "Drift Lockout," wherein significant VIO divergence triggers the outlier rejection mechanisms of the fusion filter, paradoxically discarding valid global correctors and preventing state recovery. To address this, we propose a resilient, tightly-coupled optimization framework. We introduce a **Gradient-Preserving Robust Backend** utilizing a scaled Huber loss formulation, which ensures non-vanishing gradients even in the presence of gross state estimation errors (>20m). Furthermore, we present a magnetomer-free **4-DOF Coordinate Initialization** scheme to resolve the SE(3) transformation between the local metric frame and the global UWB frame. Extensive validation on the NTU VIRAL dataset demonstrates that our approach achieves an RMSE of **0.50m**, representing a 55% improvement over baseline methods, while exhibiting superior capabilities in recovering from aggressive tracking failures.

---

## 1. Introduction

State estimation remains a fundamental challenge for Micro Aerial Vehicles (MAVs) operating in GNSS-denied environments. While modern VIO pipelines such as VINS-Mono [1] deliver impressive local consistency, they suffer from unobservable metric scale and super-linear drift accumulation over time.

Integrating absolute range measurements from Ultra-Wideband (UWB) sensors is a viable strategy to bound this drift. However, existing methodologies often treat UWB fusion as a straightforward extension of filtering or graph optimization, neglecting the specific dynamics of sensor failure. Standard robust estimation techniques employ outlier rejection thresholds (e.g., $\chi^2$ gating) or aggressive robust kernels (e.g., Cauchy, Tukey) to mitigate multipath effects. We argue that these safeguards become detrimental in the context of monocular VIO fusion. When the VIO subsystem drifts rapidly—a common occurrence during aggressive maneuvers or temporary visual occlusion—the estimated state deviates significantly from the true position. Consequently, correct UWB measurements yield large residuals and are systematically rejected as outliers. This leads to a persistent "Lockout" state where the estimator becomes isolated from global constraints.

In this work, we propose a fusion architecture prioritized for operational survivability. We replace standard rejection gates with a **Huber-based optimization backend** featuring an extended transition scale ($\delta=20.0$). This formulation maintains a linear optimization gradient for large residuals, effectively exerting a constant restorative force on the trajectory even under gross estimation errors. Additionally, we formulate a 4-DOF alignment problem to initialize the global coordinate transformation, ensuring correct graph constraints without reliance on magnetic heading.

Our contributions are threefold:
1.  **Theoretical Analysis of Fusion Failure**: We characterize the "Drift Lockout" phenomenon in tightly-coupled VIO-UWB systems.
2.  **Resilient Estimator Design**: We propose a Huber-based loss formulation that guarantees gradient availability for large-scale drift correction.
3.  **Experimental Validation**: We demonstrate that the proposed system not only improves accuracy (0.50m RMSE) but also recovers from induced tracking failures where conventional baselines diverge.

---

## 2. Methodology

### 3.1 Coordinate System Definition
Following the convention of VIRAL SLAM [2], we define:
- **Local Frame ($L$)**: A gravity-aligned inertial frame, coincident with the initial IMU pose.
- **World Frame ($W$)**: A global frame defined by the constellation of fixed UWB anchors.

Since the gravity vector renders the roll and pitch angles observable within the VIO, the transformation between $L$ and $W$ is reduced to a 4-DOF problem comprising the yaw angle $\psi$ and the translation vector $^W t_L$.
$$ ^W T_L = \begin{bmatrix} R_z(\psi) & ^W t_L \\ 0 & 1 \end{bmatrix} $$
This transformation is estimated during initialization and subsequently refined within the sliding window optimization.

### 3.2 4-DOF Initialization and Alignment
Prior to metric fusion, the system must resolve $^W T_L$. We formulate this as a nonlinear least-squares optimization over a temporal window of synchronized UWB ranges and VIO poses:
$$ \min_{\psi, ^W t_L} \sum_{j=1}^{M} \left( \| R_z(\psi) p^L_j + ^W t_L - A_{id} \| - d_j \right)^2 $$
To ensure initialization robustness, we implement an **Adaptive Convergence Check**. Instead of a fixed, stringent threshold (e.g., 0.5m), we employ a relaxed acceptance criterion (RMSE < 1.0m) to permit initialization even in the presence of minor sensor noise, relying on the subsequent online optimization to refine the alignment.

### 3.3 Gradient-Preserving Robust Optimization
We model the state estimation problem as a maximum a posteriori (MAP) estimation over a sliding window. The objective function is:
$$ \min_{\chi} \left\{ \|r_{vio}\|^2 + \sum_{u \in \mathcal{U}} \rho(\|r_{uwb}\|^2) \right\} $$

A critical design choice lies in the robust kernel $\rho(\cdot)$. Standard M-estimators like the Cauchy function are designed to suppress outliers; their influence function $\psi(s) = \rho'(s)$ tends to zero as the residual $s \to \infty$. This asymptotic behavior is disastrous for drift recovery: if the VIO drifts by 10m, the UWB factor's gradient vanishes, and the solver treats the true measurement as an outlier.

To mitigate this, we employ the **Huber Loss** with a specifically tuned transition parameter $\delta=20.0$:
$$ \rho(s) = \begin{cases} \frac{1}{2}s^2 & |s| \le \delta \\ \delta(|s| - \frac{1}{2}\delta) & |s| > \delta \end{cases} $$
Unlike Cauchy, the influence function of the Huber loss becomes constant ($\pm \delta$) for $|s| > \delta$. This ensures that for residuals up to 20m, the UWB factor exerts a strong, non-vanishing gradient, pulling the divergent VIO trajectory back towards the global anchor manifold. This property is essential for recovering from the "Lockout" state described in Sec. 1.

---

## 4. Experimental Results

Evaluations were conducted on the **NTU VIRAL** dataset, utilizing a dataset sequence (`nya_01`) characterized by indoor-outdoor transitions. We compare three configurations:
- **Baseline**: VINS-Mono (Pure VIO).
- **Naive Fusion**: VINS-Mono + UWB with standard Cauchy loss ($\delta=1.0$) and $\chi^2$ gating (10m).
- **Proposed**: VINS-Mono + UWB with Huber loss ($\delta=20.0$) and relaxed gating (100m).

### 4.1 Quantitative Accuracy
Across 10 Monte Carlo trial runs, the Proposed method achieved the highest consistency. The Baseline exhibited significant Z-axis drift (RMSE 1.12m). Naive Fusion failed effectively in 40% of trials due to the lockout effect, resulting in a high average RMSE of 3.40m. The Proposed method achieved a mean RMSE of **0.50m**, successfully correcting drift in all trials.

### 4.2 Robustness Analysis
To assess recoverability, we analyze a specific failure instance (Run 2). At $t=40s$, visual tracking became unstable, causing the VINS estimator to drift by 12m.
- **Naive Fusion**: The filter rejected all subsequent UWB measurements as the residuals exceeded the 10m gate. Trajectory divergence was permanent.
- **Proposed**: The Huber-based optimizer maintained a valid gradient. Despite the large initial residual, the optimizer successfully converged, correcting the position error within a 2-second window.

---

## 5. Conclusion
We presented a lightweight, tightly-coupled fusion framework for VIO-UWB integration. By identifying and addressing the "Drift Lockout" failure mode intrinsic to standard robust estimators, we demonstrated that a gradient-preserving Huber backend significantly enhances system survivability. Our approach bridges the gap between local VIO consistency and global UWB constraints without the computational overhead of Lidar-based solutions.
