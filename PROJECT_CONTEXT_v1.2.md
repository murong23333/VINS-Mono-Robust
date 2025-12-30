
VINS-Mono (NTU VIRAL 扩展版) 项目环境上下文 v1.2
AI 助手长期记忆文件（更新于 2025-12-23）

1. 项目基本信息
项目名称: VINS-Mono (NTU VIRAL 数据集适配版)
核心框架: ROS 1 (Kinetic/Melodic)
构建系统: Catkin
工作空间根目录: /home/cheng/catkin_ws
主要源码目录: /home/cheng/catkin_ws/src/VINS-Mono
Docker 容器: vins_container

2. 核心架构与模块
软件包	路径	核心职责
vins_estimator	src/VINS-Mono/vins_estimator	后端优化。融合 IMU + 视觉 + UWB 约束。新增 **4-DOF 初始化对齐** 模块。
feature_tracker	src/VINS-Mono/feature_tracker	前端跟踪。单目视觉光流跟踪。
benchmark_publisher	src/VINS-Mono/benchmark_publisher	发布真值 (Ground Truth)。配合 run_ntuviral.launch 中的静态 TF 使用。

3. 数据集确认 (Dataset)
主用数据集: nya_01.bag（位于 /home/cheng/nya_01 或容器内 /data/nya_01.bag）。
UWB 锚点配置: 3个锚点系统 (ID 100, 101, 102)。
    - **位置固定**: 代码内预设高度 1.5m，不再进行在线优化 (self-calibration disabled)。

4. 当前开发状态 (截至 2025-12-23)
当前阶段: Phase 9 (UWB 4-DOF Alignment & Robust Filter) 已完成。
核心精度: 
- 最佳 ATE (RMSE): 0.1988m (Plan B, Good Init).
- 对齐状态: 完美重合 (Rviz 视觉误差 < 0.1m)。
系统特性:
视觉: 单目特征点跟踪。
UWB: 
    - **4-DOF 初始化对齐**: 在 VIO 初始化阶段，利用 Ceres 求解 Yaw 和 Translation，将 VIO 坐标系对齐到 UWB 世界系。
    - **鲁棒初始化 (Robust Init)**: 
        - 初始对齐前旁路 (Bypass) Innovation Gate，允许大偏差数据进入对齐算法。
        - 对齐算法内部包含粗检测 (Range > 20m 过滤)。
        - 仅在对齐成功后激活 3.0m Gate。
    - **锚点锁定 (Locked Anchors)**: `estimate_uwb_anchor = 0`，消除优化漂移。

5. 已实现的重大改进
A. UWB 4-DOF 初始化对齐 (Initialization)
逻辑: 新增 `initial_uwb_alignment.cpp`。
作用: 彻底解决了初始转换 (T_vio_uwb) 靠猜的问题。系统现在能自动计算出 VIO 原点在 UWB 系中的准确位置 (-1.8m 高度, 180度旋转等)。

B. 数学最优可视化 (Visualization)
问题: UWB 系 (绝对) 和 GT 系 (相对) 存在复杂变换，手动调参不准。
解决方案: 
- 编写 `align_viz.py` 脚本，基于 SVD 算法计算 VINS 输出轨迹与 GT 真值的最优变换矩阵。
- **最优参数 (Calculated)**: `7.45 -2.39 -2.94 -1.31 0 0`。
- 这一参数组已写入 `run_ntuviral.launch`，实现了像素级重合。

C. 增强的数据鲁棒性 (Robustness)
逻辑: 
- 增大 Gate Threshold 至 3.0m。
- 实现了 "Gate Bypass" 逻辑 (`estimator_node.cpp`)，解决了 "死锁" 问题（即数据偏差大导致被拒 -> 无法对齐 -> 偏差永远无法消除）。

6. 遗漏的底层逻辑 (Logic Details)
UWB 噪声参数: std = 0.1。
初始化: 可以在 `estimator_node.cpp` 中看到逻辑：`uwb_align_success` 标志位控制了 Gate 的开启。

7. 运行指南 (Demo Mode)
启动计算: `roslaunch vins_estimator run_ntuviral.launch` (已内置 Optimal TF)。
启动数据: `rosbag play /data/nya_01.bag`
启动 Rviz: 自动弹出，无需额外配置。

8. 开发守则 (AI 强制执行)
坐标系定义: 锚点坐标固定在 `initUwbAnchors` (Z=1.5m)，严禁修改。
Rviz 对齐: 若需重新校准，必须运行 `python3 align_viz.py` 重新计算参数，严禁手动通过肉眼微调。
代码回退: 严禁恢复 "Automatic TF Broadcasting" (VINS 内部广播 TF)，因为在对齐失败时会导致 GT 线消失。必须使用 Launch 文件的 Static TF 作为兜底。

9. Runtime Environment
(同 v1.0，Docker 环境配置保持不变)
