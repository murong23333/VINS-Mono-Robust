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
