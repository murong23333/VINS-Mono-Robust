### 3.4 Magnetometer-Free 4-DOF Initialization

A prerequisite for tightly-coupled fusion is the alignment of the local VIO frame $\{L\}$ with the global UWB frame $\{W\}$. Conventional systems often rely on magnetometers to determine the absolute yaw angle. However, in indoor environments typical of MAV operations (e.g., warehouses, industrial plants), the local magnetic field is frequently distorted by ferromagnetic structures and high-current electrical lines, rendering compass headings unreliable.

To address this, we propose a magnetometer-free initialization scheme. Since both $\{L\}$ and $\{W\}$ are gravity-aligned (the former by IMU initialization, the latter by anchor constellation design), the unknown transformation is reduced to 4 Degrees of Freedom: the yaw angle $\psi$ and the translation vector $^W\mathbf{t}_L$.

We solve for these parameters by batch processing a short history of UWB range measurements and synchronized VIO poses. The problem is formulated as a nonlinear least-squares optimization:

$$
\min_{\psi, ^W\mathbf{t}_L} \sum_{k=1}^{K} \sum_{m \in \mathcal{A}_k} \left( \| \mathbf{R}_z(\psi) ^L\mathbf{p}_{k} + ^W\mathbf{t}_L - \mathbf{A}_m \| - d_{k,m} \right)^2
$$

where $^L\mathbf{p}_{k}$ is the MAV position in the local frame at time $k$, $\mathbf{A}_m$ is the position of the $m$-th anchor, and $d_{k,m}$ is the measured range. This formulation leverages the relative translation captured by the VIO to resolve the absolute yaw observability. By strictly relying on geometric constraints from UWB ranging and the gravity vector, our method ensures high immunity to electromagnetic interference, guaranteeing robust initialization in complex industrial environments.
