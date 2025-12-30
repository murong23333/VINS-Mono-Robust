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
