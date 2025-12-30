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
