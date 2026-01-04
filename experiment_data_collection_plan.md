# ICGNC 论文实验数据收集计划 (Experiment Data Collection Plan)

本文件用于记录在 Ubuntu 系统上运行实验及收集数据的过程。请在 Ubuntu 环境下完成实验后，将结果填入本文件，或保存相应的 CSV/日志文件以便生成最终图表。

## 1. 实验环境准备
- **系统**: Ubuntu 18.04 / 20.04 (ROS Melodic / Noetic)
- **代码库**: VINS-Mono-Robust (确保已编译 Proposed, Naive, Original VINS 版本)
- **数据集**: NTU VIRAL Dataset (Bag files)

---

## 2. 快速配置切换指南 (Quick Configuration Switching)
为了高效完成四种模式的实验，请按照以下步骤修改代码或配置：

### 2.1 Proposed Method (Ours)
- **配置**: 使用 Scaled Huber Loss ($\delta=20.0$) 及 4-DOF Initialization。
- **检查点**: `vins_estimator/src/factor/uwb_factor.cpp` (或类似文件)
  ```cpp
  // 确保使用 Scaled Huber
  loss_function = new ceres::HuberLoss(20.0);
  ```

### 2.2 Naive Fusion (Ablation)
- **配置**: 使用 Cauchy Loss ($\delta=1.0$) 或 Huber ($\delta=1.0$)。
- **修改**: 在 `vins_estimator/src/factor/uwb_factor.cpp` 中修改 Loss Function：
  ```cpp
  // 修改为 Cauchy 或小阈值 Huber
  // loss_function = new ceres::HuberLoss(20.0); // Comment out
  loss_function = new ceres::CauchyLoss(1.0);  // Use this!
  ```
- **重新编译**: `catkin_make` (修改 C++ 代码后必须重新编译)

### 2.3 Pure VINS (Baseline)
- **配置**: 关闭 UWB 融合，仅运行 VINS。
- **修改**: 编辑 `config/ntuviral/viral_camera_imu_config.yaml`
  ```yaml
  # 临时禁用 UWB topic 或将权重设为极低
  uwb_noise_std: 999.0  # 简单黑客法：极大噪声导致 UWB 被忽略
  # 或者在代码中直接注释掉 UWB Factor 的添加
  ```

### 2.4 Pure UWB (Baseline)
- **配置**: 仅评估 UWB 测距精度（无需运行 VINS）。
- **工具**: 编写简单 Python 脚本订阅 `/uwb_endorange_info` 和 `/leica/position` (Ground Truth)，计算 $d_{measured} - d_{truth}$ 的 RMSE。

---

## 3. 实验一：定量精度分析 (对应论文 Table 1)

**目标**: 获取所有序列的绝对轨迹误差 (ATE RMSE, 单位: 米)。
**对比方法**:
1.  **Pure UWB**: 仅使用 UWB 测距解算 (Baseline 1)。
2.  **Pure VINS**: 原始 VINS-Mono，无 UWB (Baseline 2)。
3.  **Naive Fusion**: 紧耦合，Cauchy Kernel ($\delta=1.0$), Strict $\chi^2$ (Ablation)。
4.  **Proposed**: 紧耦合，Scaled Huber Backend ($\delta=20.0$), 4-DOF Init (Ours)。

**请在下方表格填入 RMSE 结果**:

| Dataset | Pure UWB | Pure VINS | Naive Fusion | Proposed (Ours) | 备注 (Note) |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **eee_01** | | | | | |
| **eee_02** | | | | | |
| **eee_03** | | | | | |
| **nya_01** | | | | | *重点序列 (Recovery)* |
| **nya_02** | | | | | *重点序列* |
| **nya_03** | | | | | |
| **sbs_01** | | | | | |
| **sbs_02** | | | | | |
| **sbs_03** | | | | | |
| **rtp_01** | | | | | |
| **rtp_02** | | | | | |
| **rtp_03** | | | | | |
| **tnp_01** | | | | | |
| **tnp_02** | | | | | |
| **tnp_03** | | | | | |
| **spms_01**| | | | | |
| **spms_02**| | | | | |
| **spms_03**| | | | | |

---

## 3. 实验二：鲁棒性与恢复分析 (对应论文 Figure 6 & 7)

**目标**: 验证系统在 "Drift Lockout" 场景下的恢复能力。
**关键序列**: `nya_01` (或任何包含剧烈旋转/特征丢失导致漂移的序列)。
**关键时间点**: 需记录故障发生时刻 (例如 t=40s) 前后的表现。

### 3.1 轨迹对比数据 (Figure 6)
我们需要导出以下三条轨迹的 CSV 文件 (格式: `timestamp, x, y, z`)：

1.  **Ground Truth**: `nya_01_gt.csv`
2.  **Naive Fusion**: `nya_01_naive_traj.csv` (应在 t=40s 附近发散)
3.  **Proposed**: `nya_01_proposed_traj.csv` (应在发散后迅速拉回)

**操作步骤**:
1.  运行 `nya_01` bag。
2.  使用 `evo_traj` 或自定义脚本保存轨迹。
3.  **请确认**: Naive 方法是否确实发生了漂移？(Yes/No): _____
4.  **请确认**: Proposed 方法是否成功恢复？(Yes/No): _____

### 3.2 ATE 随时间变化数据 (Figure 7)
我们需要计算 ATE Error 随时间的变化曲线 (APE/ATE vs Time)。

1.  **Naive Fusion ATE**: `nya_01_naive_ate.csv` (格式: `timestamp, error_value`)
2.  **Proposed ATE**: `nya_01_proposed_ate.csv`

**数据检查点**:
- 在故障时刻 (如 40s)，Naive 的 Error 应该急剧上升 (e.g. > 10m)。
- Proposed 的 Error 可能有小幅上升 (视觉丢失)，但在 1-2秒内应回落至正常水平 (< 0.5m)。

---

## 4. 实验三：(可选) 初始化验证

如果需要验证 Magnetometer-Free Initialization：
- 记录初始化所需的平均时间 (s)。
- 记录初始化后的 Yaw 角误差 (度)。

*(本部分若论文中未详细列出表格，可仅做定性验证)*

---

## 5. 数据导出清单

请确保以下文件已生成并保存到项目目录 (建议新建 `data_logs/` 文件夹):

- [ ] `table1_results.xlsx` (或填好上面的 markdown 表格)
- [ ] `nya_01_gt.csv`
- [ ] `nya_01_naive_traj.csv`
- [ ] `nya_01_proposed_traj.csv`
- [ ] `nya_01_naive_ate.csv`
- [ ] `nya_01_proposed_ate.csv`

完成后，请将这些文件带回，我们将使用 Python 脚本绘制最终用于论文的 **高清 Figure 6 和 Figure 7**。
