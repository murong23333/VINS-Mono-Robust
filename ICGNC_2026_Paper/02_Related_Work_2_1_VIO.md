## 2. Related Work

### 2.1 Visual-Inertial Odometry

Visual-Inertial Odometry (VIO) algorithms are generally categorized into filtering-based and optimization-based approaches. Filtering methods, such as MSCKF [3] and ROVIO [4], employ Extended Kalman Filters (EKF) to process visual and inertial measurements sequentially. While computationally efficient, these methods typically struggle with linearization errors during complex maneuvers. Conversely, optimization-based frameworks like VINS-Mono [1] and OKVIS [2] formulate state estimation as a nonlinear least-squares problem over a sliding window. By iteratively relinearizing the objective function, these methods achieve superior accuracy and consistency.

Despite these advancements, all monocular VIO systems share a fundamental limitation: the inability to observe global metric scale and absolute yaw from local sensor data alone. Although IMU integration renders the scale observable in theory, in practice, sensor bias drift and integration noise inevitably lead to super-linear position error accumulation over extended trajectories [5]. Furthermore, standard VIO pipelines are designed under the assumption of continuous feature tracking; they lack inherent mechanisms to correct large-scale pose errors once the estimator has diverged significantly from the ground truth.
