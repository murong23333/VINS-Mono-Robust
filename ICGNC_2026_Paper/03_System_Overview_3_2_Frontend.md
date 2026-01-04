### 3.2 Visual-Inertial Frontend

Our system builds upon the established VINS-Mono [1] architecture. The frontend processes raw sensor data to generate local constraints:

1.  **Vision**: For each new image, we detect Harris corner features and track them via the KLT sparse optical flow algorithm. Features are undistorted and projected onto a unit sphere. Outliers are rejected using RANSAC with a fundamental matrix model.
2.  **IMU**: High-frequency IMU measurements are processed using the manifold preintegration theory [14]. This yields relative motion constraints between consecutive keyframes that are independent of the initial state, summarizing the complex inertial dynamics into a single factor.

These components function as a standard VIO pipeline, providing an accurate but drift-prone estimation of the robot's trajectory within the Local Frame $\{L\}$. We omit further details of the frontend as it follows standard practices, focusing instead on the novel UWB fusion mechanism.
