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
