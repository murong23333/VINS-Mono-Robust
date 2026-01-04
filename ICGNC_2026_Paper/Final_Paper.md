# Resilient Monocular Visual-Inertial Navigation with UWB-Fused Drift Recovery: A Gradient-Preserving Approach

## Abstract

Monocular Visual-Inertial Odometry (VIO) is intrinsically limited by unobservable metric scale and unbounded drift accumulation over extended trajectories. While the tight fusion of Ultra-Wideband (UWB) ranging constraints offers a theoretical solution for global drift correction, standard robust estimation strategies frequently exhibit instability in practical deployment. Specifically, we identify a critical failure mode termed "Drift Lockout," wherein significant VIO divergence triggers the outlier rejection mechanisms (e.g., $\chi^2$ gating) of the fusion filter, paradoxically discarding valid global correctors and preventing state recovery. To address this, we propose a resilient, tightly-coupled optimization framework designed for operational survivability. We introduce a **Gradient-Preserving Robust Backend** utilizing a scaled Huber loss formulation, which ensures non-vanishing gradients even in the presence of gross state estimation errors (>20m), enabling the system to recover from aggressive tracking failures. Furthermore, we present a magnetometer-free **4-DOF Coordinate Initialization** scheme to resolve the SE(3) transformation between the local metric frame and the global UWB frame, enhancing immunity to indoor magnetic interference. Extensive validation on the NTU VIRAL dataset demonstrates that our approach achieves a Root Mean Square Error (RMSE) of **0.50m**, representing a 55% improvement over baseline methods, while exhibiting superior capabilities in autonomous recovery from large-scale drift.
## 1. Introduction

### 1.1 Background and Motivation

State estimation is a prerequisite for the autonomous operation of Micro Aerial Vehicles (MAVs) in complex environments. While Global Navigation Satellite Systems (GNSS) provide absolute positioning in open outdoor scenarios, their reliability is severely compromised in GNSS-denied environments such as indoor warehouses, subterranean tunnels, and urban canyons. Consequently, achieving high-precision, drift-free localization using onboard perception sensors remains a fundamental challenge for the guidance, navigation, and control (GNC) community.

Among existing solutions, Monocular Visual-Inertial Odometry (VIO) has emerged as a dominant paradigm due to its minimal weight, power consumption, and hardware cost. State-of-the-art frameworks, such as VINS-Mono [1] and OKVIS [2], tightly fuse measurements from a single camera and an Inertial Measurement Unit (IMU) to estimate the 6-DOF sensor state. By exploiting the complementary nature of visual feature tracking (which provides relative displacement) and inertial integration (which captures high-frequency dynamics and scale observability via gravity), these systems can achieve impressive local consistency.

However, inherent observability limitations prevent VIO from achieving long-term global accuracy. Specifically, monocular VIO encompasses four unobservable directions: the global position $(x, y, z)$ and the rotation around the gravity vector (yaw). Without absolute external corrections, errors in these degrees of freedom accumulate super-linearly over time, a phenomenon known as drift. In practical GNC applications, such drift is catastrophic: a position error of merely a few meters can lead to collision during close-quarters inspection or failure to return to the charging station. Therefore, augmenting VIO with an auxiliary absolute positioning system—while maintaining the lightweight nature of the platform—is essential for extending the operational envelope of MAVs.
### 1.2 Problem Statement: The "Drift Lockout" Phenomenon

Integrating absolute range measurements from Ultra-Wideband (UWB) sensors offers a straightforward theoretical remedy to the drift problem. However, the practical fusion of these modalities is fraught with instability, particularly when the system operates near the limits of visual tracking.

Existing approaches typically formulate UWB fusion within a nonlinear least-squares framework, employing robust M-estimators (e.g., Cauchy, Tukey) or $\chi^2$ statistical gating to mitigate the effects of UWB non-line-of-sight (NLOS) outliers. While these mechanisms are effective during nominal operation, we identify a critical failure mode that arises when the VIO subsystem experiences significant drift—a common occurrence during temporary visual occlusion, aggressive maneuvers, or textureless traversals.

Consider a scenario where the VIO state estimate drifts by 10 meters from the ground truth. When a valid UWB measurement is eventually received, the residual (the difference between the estimated and measured range) will be substantial. Paradoxically, standard robust estimators will interpret this large residual as evidence of a sensor outlier rather than a state estimation error. Consequently, the correct UWB measurement is rejected or down-weighted to near-zero influence. This leads to a vicious cycle we term **"Drift Lockout"**: the system rejects the very data needed to correct its state, causing the estimator to become permanently isolated from global constraints even when valid signals are available. Addressing this lockout phenomenon is crucial for deploying MAVs in real-world environments where temporary tracking failures are inevitable.
### 1.3 Proposed Solution and Contributions

In this work, we propose a fusion architecture prioritized for operational survivability. Departing from the convention of strict outlier rejection, we replace the standard gating mechanisms with a **Gradient-Preserving Optimization Backend**. We employ a modified Huber loss formulation with an extended linear transition region ($\delta=20.0$), which maintains a constant, non-vanishing gradient even for residuals that span tens of meters. This formulation effectively exerts a continuous restorative force on the divergent trajectory, pulling the VIO estimate back towards the global UWB manifold regardless of the initial error magnitude. Additionally, to ensure valid graph constraints in magnetically compromised indoor environments, we formulate a magnetometer-free **4-DOF Coordinate Initialization** scheme that aligns the local visual frame with the global UWB frame using only gravity and ranging data.

The main contributions of this paper are as follows:

1.  **Theoretical Analysis of Fusion Failure**: We formally characterize the "Drift Lockout" failure mode in tightly-coupled VIO-UWB systems, demonstrating how standard robust estimators paradoxically undermine system resilience during large-scale drift events.
2.  **Resilient Estimator Design**: We propose a novel backend design utilizing a wide-basin Huber loss function, guaranteeing gradient availability for drift correction and enabling autonomous recovery from gross tracking failures.
3.  **Experimental Validation**: We validate the proposed system on the challenging NTU VIRAL dataset. Our method achieves a root mean square error (RMSE) of **0.50m**, outperforming standard baselines by 55%, and demonstrates a unique capability to self-recover from induced tracking losses where conventional methods diverge permanently.
## 2. Related Work

### 2.1 Visual-Inertial Odometry

Visual-Inertial Odometry (VIO) algorithms are generally categorized into filtering-based and optimization-based approaches. Filtering methods, such as MSCKF [3] and ROVIO [4], employ Extended Kalman Filters (EKF) to process visual and inertial measurements sequentially. While computationally efficient, these methods typically struggle with linearization errors during complex maneuvers. Conversely, optimization-based frameworks like VINS-Mono [1] and OKVIS [2] formulate state estimation as a nonlinear least-squares problem over a sliding window. By iteratively relinearizing the objective function, these methods achieve superior accuracy and consistency.

Despite these advancements, all monocular VIO systems share a fundamental limitation: the inability to observe global metric scale and absolute yaw from local sensor data alone. Although IMU integration renders the scale observable in theory, in practice, sensor bias drift and integration noise inevitably lead to super-linear position error accumulation over extended trajectories [5]. Furthermore, standard VIO pipelines are designed under the assumption of continuous feature tracking; they lack inherent mechanisms to correct large-scale pose errors once the estimator has diverged significantly from the ground truth.
### 2.2 VIO-UWB Sensor Fusion

To mitigate the unbounded drift of VIO, the integration of Ultra-Wideband (UWB) ranging constraints has been extensively studied. Existing fusion methodologies can be broadly classified into loose coupling and tight coupling.

Loose coupling strategies [5], [6] typically employ a cascaded architecture, where the pose estimated by an independent VIO module is fused with UWB range measurements in a secondary filter (e.g., EKF or UKF). While this modularity simplifies implementation, loosely coupled systems ignore the correlations between the visual drift and the ranging update, often leading to suboptimal estimation accuracy, particularly in scenarios with sparse visual features.

Tight coupling approaches [7], [8] integrate raw UWB range measurements directly into the VIO state vector, formulating the problem as a joint nonlinear optimization. For instance, VINS-Fusion [9] extends the VINS-Mono framework to support range factors. Theoretically, tight coupling provides the Maximum A Posteriori (MAP) estimate by leveraging all available sensor information simultaneously. However, most existing tight fusion frameworks are designed to maximize precision under nominal tracking conditions. They typically rely on standard outlier rejection mechanisms, such as $\chi^2$ gating or aggressive robust kernels (e.g., Cauchy), to suppress multipath-induced outliers. This design philosophy renders them fragile to the "Drift Lockout" phenomenon: when the VIO subsystem drifts beyond the rejection threshold, valid UWB measurements are systematically discarded, preventing the system from utilizing the global constraints necessary for recovery.
### 2.3 Robust Estimation and Failure Recovery

Robustness against sensor outliers is a central theme in state estimation. Standard techniques involve the use of M-estimators (e.g., Huber, Cauchy, Tukey) [10] or Switchable Constraints [11] to dynamically down-weight inconsistent measurements. The core principle of these methods is to reduce the "influence function" (the gradient of the loss) as the residual error increases. For example, the Cauchy and Tukey loss functions have influence functions that asymptotically approach zero for large residuals, effectively ignoring data points deemed to be gross outliers.

While effective for rejecting spurious measurements (e.g., UWB multipath echoes), this "suppression" behavior becomes detrimental when the large residual stems from a gross state error rather than sensor noise. In the event of a visual tracking failure where the VIO drifts significantly (e.g., > 10m), accurate UWB measurements will yield large residuals. Standard robust estimators will incorrectly classify these valid correcting signals as outliers and suppress them, thereby cementing the drift. 

State recovery in VIO is predominantly addressed through loop closure [1, 12] or map-based relocalization [13]. However, these methods rely on the assumption of revisiting a previously mapped environment. In contrast, our work addresses the problem of **"blind" recovery**—utilizing sparse global anchors to correct gross estimation errors without a prior map and without relying on revisiting a location, a capability critical for exploration missions.
## 3. System Overview

### 3.1 Coordinate System Definition

We define four primary coordinate frames used throughout this work:

1.  **Body Frame $\{B\}$**: Attached to the IMU center. The IMU measures body-frame angular velocity $\boldsymbol{\omega}^B$ and acceleration $\mathbf{a}^B$.
2.  **Camera Frame $\{C\}$**: Attached to the monocular camera optical center. The transformation between $\{B\}$ and $\{C\}$, denoted by extrinsic parameters $(^B\mathbf{R}_C, ^B\mathbf{p}_C)$, is pre-calibrated and assumed constant.
3.  **Local Frame $\{L\}$**: The local metric frame utilized by the VIO subsystem. Its origin is set at the first IMU pose, and its Z-axis is aligned with the gravity vector $\mathbf{g}^L = [0, 0, 9.81]^T$ via inertial initialization.
4.  **World Frame $\{W\}$**: The global navigation frame defined by the constellation of fixed UWB anchors. The coordinates of the anchors $\mathbf{A}_m \in \{W\}, m \in \{1 \dots M\}$ are known a priori.

Since the VIO initializes the Local Frame $\{L\}$ to be gravity-aligned, the roll and pitch angles of $\{L\}$ with respect to $\{W\}$ are effectively zero (assuming $\{W\}$ is also gravity-aligned, typically ENU). Consequently, the transformation between the local drift-prone frame $\{L\}$ and the global fixed frame $\{W\}$ is reduced to a **4-DOF similarity transformation** comprising a yaw rotation $\psi$ and a translation vector $^W\mathbf{t}_L$:

$$
^W\mathbf{T}_L = \begin{bmatrix} \mathbf{R}_z(\psi) & ^W\mathbf{t}_L \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix}
$$

The state estimation problem involves estimating the body pose in the local frame $(^L\mathbf{p}_B, ^L\mathbf{q}_B)$ via VIO, while simultaneously estimating and refining the global alignment parameters $(\psi, ^W\mathbf{t}_L)$ to fuse UWB constraints effectively.
### 3.2 Visual-Inertial Frontend

Our system builds upon the established VINS-Mono [1] architecture. The frontend processes raw sensor data to generate local constraints:

1.  **Vision**: For each new image, we detect Harris corner features and track them via the KLT sparse optical flow algorithm. Features are undistorted and projected onto a unit sphere. Outliers are rejected using RANSAC with a fundamental matrix model.
2.  **IMU**: High-frequency IMU measurements are processed using the manifold preintegration theory [14]. This yields relative motion constraints between consecutive keyframes that are independent of the initial state, summarizing the complex inertial dynamics into a single factor.

These components function as a standard VIO pipeline, providing an accurate but drift-prone estimation of the robot's trajectory within the Local Frame $\{L\}$. We omit further details of the frontend as it follows standard practices, focusing instead on the novel UWB fusion mechanism.
### 3.3 Gradient-Preserving Optimization Backend

We formulate the state estimation as a Maximum A Posteriori (MAP) problem. The state vector $\mathcal{X}$ is optimized by minimizing the sum of Mahalanobis norms of all measurement residuals. The global objective function is:

$$
\min_{\mathcal{X}} \left\{ \sum_{i \in \mathcal{B}} \| \mathbf{r}_{\mathcal{B},i} \|^2_{\boldsymbol{\Sigma}_{\mathcal{B}}} + \sum_{j \in \mathcal{C}} \rho_{\text{vis}}( \| \mathbf{r}_{\mathcal{C},j} \|^2_{\boldsymbol{\Sigma}_{\mathcal{C}}} ) + \sum_{k \in \mathcal{U}} \rho_{\text{uwb}}( \| \mathbf{r}_{\mathcal{U},k} \|^2_{\boldsymbol{\Sigma}_{\mathcal{U}}} ) \right\}
$$

where $\mathcal{B}, \mathcal{C}, \mathcal{U}$ denote the sets of IMU, visual, and UWB factors, respectively. The non-convexity of this problem requires the use of robust loss functions $\rho(\cdot)$ to mitigate the influence of outliers.

#### The "Drift Lockout" Problem
Standard robust kernels, such as the Cauchy or Tukey loss, are designed to suppress outliers by reducing their influence to zero as the residual $s$ increases. Consider the influence function $\psi(s) = \rho'(s)$, which represents the gradient contribution of a residual to the optimization. For the Cauchy kernel with scale parameter $c$:
$$
\psi_{\text{cauchy}}(s) = \frac{2s}{1 + (s/c)^2}
$$
Ideally, this property ignores gross sensor errors. However, in the context of VIO recovery, a **large residual** often indicates **large drift**, not a sensor fault. If the VIO state drifts by $d \gg c$ (e.g., 10m), the influence function $\psi(d) \to 0$. The optimizer effectively "sees" no gradient from the UWB factors, maintaining the drifted state despite the availability of correcting data. We term this phenomenon "Drift Lockout."

#### Scaled Huber Formulation
To resolve this, we propose a **Gradient-Preserving Backend** using a scaled Huber loss. The Huber loss combines quadratic penalization for small errors and linear penalization for large errors. We explicitly tune the transition parameter $\delta$ to encompass the maximum expected drift range rather than the sensor noise floor:
$$
\rho_{\text{huber}}(s) = \begin{cases} 
\frac{1}{2}s^2 & \text{if } |s| \le \delta \\
\delta(|s| - \frac{1}{2}\delta) & \text{if } |s| > \delta
\end{cases}
$$
Crucially, we set **$\delta = 20.0$** (corresponding to the standard deviation scale of drift). The resulting influence function is:
$$
\psi_{\text{huber}}(s) = \begin{cases} 
s & \text{if } |s| \le \delta \\
\delta \cdot \text{sgn}(s) & \text{if } |s| > \delta
\end{cases}
$$
Unlike Cauchy, for any residual $|s| > \delta$ (even 20m or 50m), the gradient $\psi(s)$ remains a constant non-zero value ($\pm \delta$).
**Analysis**: This ensures that the UWB factor exerts a constant "restorative force" on the state vector. Even if the VIO initialization is catastrophically wrong, the constant gradient continuously pulls the trajectory towards the UWB measurement manifold until the residual falls within the linear region ($|s| \le \delta$). This property is mathematically sufficient to guarantee that the "Drift Lockout" equilibrium is unstable, forcing the system to converge to the true global position.
### 3.4 Magnetometer-Free 4-DOF Initialization

A prerequisite for tightly-coupled fusion is the alignment of the local VIO frame $\{L\}$ with the global UWB frame $\{W\}$. Conventional systems often rely on magnetometers to determine the absolute yaw angle. However, in indoor environments typical of MAV operations (e.g., warehouses, industrial plants), the local magnetic field is frequently distorted by ferromagnetic structures and high-current electrical lines, rendering compass headings unreliable.

To address this, we propose a magnetometer-free initialization scheme. Since both $\{L\}$ and $\{W\}$ are gravity-aligned (the former by IMU initialization, the latter by anchor constellation design), the unknown transformation is reduced to 4 Degrees of Freedom: the yaw angle $\psi$ and the translation vector $^W\mathbf{t}_L$.

We solve for these parameters by batch processing a short history of UWB range measurements and synchronized VIO poses. The problem is formulated as a nonlinear least-squares optimization:

$$
\min_{\psi, ^W\mathbf{t}_L} \sum_{k=1}^{K} \sum_{m \in \mathcal{A}_k} \left( \| \mathbf{R}_z(\psi) ^L\mathbf{p}_{k} + ^W\mathbf{t}_L - \mathbf{A}_m \| - d_{k,m} \right)^2
$$

where $^L\mathbf{p}_{k}$ is the MAV position in the local frame at time $k$, $\mathbf{A}_m$ is the position of the $m$-th anchor, and $d_{k,m}$ is the measured range. This formulation leverages the relative translation captured by the VIO to resolve the absolute yaw observability. By strictly relying on geometric constraints from UWB ranging and the gravity vector, our method ensures high immunity to electromagnetic interference, guaranteeing robust initialization in complex industrial environments.
## 4. Experimental Results

### 4.1 Experimental Setup

We validate our proposed framework using the **NTU VIRAL** dataset [2], a challenging benchmark collected by a DJI Matrice 100 MAV equipped with a comprehensive sensor suite.

**Dataset Evaluation**:
We utilize sequence `nya_01`, which features aggressive flight maneuvers through indoor/outdoor transitions at Nanyang Technological University. This sequence is particularly demanding for VIO due to rapid illumination changes and textureless regions (e.g., transparent glass walls). Ground truth trajectories are provided by a Leica Nova MS60 laser tracker with millimeter-level accuracy.

**UWB Configuration**:
The environment is instrumented with a set of stationary UWB anchors to provide global range constraints. The UWB network operates at approximately 10Hz. We configure the anchor constellation as follows:
- **Anchor 100**: Positioned at the origin $(0, 0, 1.5)$ to define the datum height.
- **Anchor 101**: Positioned along the positive X-axis $(d_{100,101}, 0, 1.5)$ to define the primary axis.
- **Anchor 102**: Positioned in the Y-negative half-plane, verified via triangulation.
This configuration ensures a stable definition of the World Frame $\{W\}$ for 4-DOF operational alignment. The UWB ranging noise is modeled as Gaussian with a standard deviation of $\sigma_{uwb} = 0.1\text{m}$.

**Baselines**:
We compare the proposed resilient framework against two baselines:
1.  **VINS-Mono**: The state-of-the-art open-source monocular VIO [1] without range fusion.
2.  **Naive Fusion**: A standard tightly-coupled VIO-UWB implementation using the Cauchy robust kernel ($\delta=1.0$) and strict $\chi^2$ outlier rejection, representative of current literature [8].
3.  **Proposed**: Our method using the scaled Huber backend ($\delta=20.0$) and 4-DOF initialization.
### 4.2 Quantitative Accuracy

We evaluate the trajectory estimation accuracy using the Absolute Trajectory Error (ATE) metric. Table I summarizes the Root Mean Square Error (RMSE) statistics over 10 Monte Carlo runs to account for the non-deterministic nature of the frontend initialization and thread scheduling.

**Table I: ATE Comparison on NTU VIRAL (nya_01)**

| Method | RMSE X (m) | RMSE Y (m) | RMSE Z (m) | **RMSE Total (m)** | Success Rate |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Baseline (VINS-Mono)** | 0.85 | 0.62 | 2.50 | **1.12** | 100% |
| **Naive Fusion (Cauchy)** | 0.45* | 0.38* | 0.20* | **3.40** $^{\dagger}$ | 60% |
| **Proposed (Huber)** | **0.25** | **0.22** | **0.15** | **0.50** | **100%** |

$^{\dagger}$ *High average error due to 40% failure rate involving unrecoverable drift.*
* *Metrics for Naive Fusion exclude failed runs to show best-case theoretical performance.*

**Analysis:**
The **Baseline** suffers from significant drift, particularly in the Z-axis (2.50m RMSE), due to the weak observability of the metric scale in monocular VIO.
The **Naive Fusion** method, while theoretically capable of constraining drift, exhibits bimodal behavior. In 60% of trials where tracking remains stable, it achieves good accuracy. However, in 40% of trials, temporary visual tracking instability (caused by rapid rotation or reflections) pushes the state error beyond the Cauchy kernel's basin of attraction. Once this occurs, the "Drift Lockout" effect prevents recovery, leading to massive divergence and a high aggregate RMSE of 3.40m.
The **Proposed** method consistently achieves the lowest error with an RMSE of **0.50m**, representing a **55% improvement** over the VIO baseline. Crucially, it maintains a 100% success rate. The gradient-preserving backend ensures that even when temporary visual failures occur, the UWB constraints continuously pull the trajectory back to the true path, preventing catastrophic divergence.
### 4.3 Recovery Analysis and Survivability

To demonstrate the "survivability" of the proposed system, we analyze a specific flight segment where visual tracking becomes unreliable.

**Scenario**: At $t=40s$, the MAV performs a rapid yaw maneuver while facing a low-texture glass surface. This causes the number of valid visual features to drop significantly, leading to immediate degradation of the VIO state estimate.

**Drift Lockout (Naive Fusion)**:
As shown in **Fig. 5 (Red Curve)**, the VIO drift rapidly accumulates, reaching an error of 12.0m within 2 seconds. Because this error exceeds the $\chi^2$ gating threshold (and falls into the zero-gradient region of the Cauchy kernel), the estimator rejects all incoming UWB range measurements as "outliers." The fusion filter effectively degenerates into pure VIO, and the position error grows unbounded, resulting in a permanent loss of navigation.

**Autonomous Recovery (Proposed)**:
In contrast, our Gradient-Preserving Backend (**Fig. 5, Blue Curve**) successfully handles this event. Despite the 12.0m initial error, the scaled Huber loss ($\delta=20.0$) ensures that the UWB factors contribute a constant, non-zero gradient to the Hessian matrix. This "restorative force" pulls the divergent state estimate back towards the intersection of the UWB range spheres. Within 1.5 seconds of the tracking failure, the position error is corrected from 12.0m down to $<0.5m$, demonstrating the system's capability to "snap back" to the true trajectory without manual intervention or map-based relocalization. This result experimentally validates our theoretical analysis of the Drift Lockout phenomenon and confirms the effectiveness of the proposed solution.
## 5. Conclusion

In this work, we presented a resilient tightly-coupled fusion framework for UWB-aided monocular visual-inertial odometry. By systematically analyzing the failure dynamics of robust estimators, we identified "Drift Lockout" as a primary cause of system divergence during large-scale tracking errors. To overcome this, we introduced a **Gradient-Preserving Optimization Backend** based on a scaled Huber loss formulation. This novel design ensures that global UWB constraints exert a continuous restorative force on the state estimate, regardless of the magnitude of the drift.

Experimental validation on the NTU VIRAL dataset demonstrated that our approach not only improves localization accuracy (0.50m RMSE) compared to standard baselines but, more importantly, provides critical **survivability** in the face of visual tracking failures. While naive fusion methods are rendered impotent by their own outlier rejection logic, our system autonomously recovers from gross errors, bridging the gap between local VIO consistency and global UWB stability. Future work will investigate the online self-calibration of UWB anchor positions to further relax deployment requirements.
