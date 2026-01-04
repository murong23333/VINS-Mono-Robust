### 4.3 Recovery Analysis and Survivability

To demonstrate the "survivability" of the proposed system, we analyze a specific flight segment where visual tracking becomes unreliable.

**Scenario**: At $t=40s$, the MAV performs a rapid yaw maneuver while facing a low-texture glass surface. This causes the number of valid visual features to drop significantly, leading to immediate degradation of the VIO state estimate.

**Drift Lockout (Naive Fusion)**:
As shown in **Fig. 5 (Red Curve)**, the VIO drift rapidly accumulates, reaching an error of 12.0m within 2 seconds. Because this error exceeds the $\chi^2$ gating threshold (and falls into the zero-gradient region of the Cauchy kernel), the estimator rejects all incoming UWB range measurements as "outliers." The fusion filter effectively degenerates into pure VIO, and the position error grows unbounded, resulting in a permanent loss of navigation.

**Autonomous Recovery (Proposed)**:
In contrast, our Gradient-Preserving Backend (**Fig. 5, Blue Curve**) successfully handles this event. Despite the 12.0m initial error, the scaled Huber loss ($\delta=20.0$) ensures that the UWB factors contribute a constant, non-zero gradient to the Hessian matrix. This "restorative force" pulls the divergent state estimate back towards the intersection of the UWB range spheres. Within 1.5 seconds of the tracking failure, the position error is corrected from 12.0m down to $<0.5m$, demonstrating the system's capability to "snap back" to the true trajectory without manual intervention or map-based relocalization. This result experimentally validates our theoretical analysis of the Drift Lockout phenomenon and confirms the effectiveness of the proposed solution.
