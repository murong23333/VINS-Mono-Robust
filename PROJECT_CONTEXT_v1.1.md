VINS-Mono (NTU VIRAL 扩展版) 项目环境上下文 v1.1
AI 助手长期记忆文件（更新于 2025-12-22）

1. 项目基本信息
项目名称: VINS-Mono (NTU VIRAL 数据集适配版)
核心框架: ROS 1 (Kinetic/Melodic)
构建系统: Catkin
工作空间根目录: /home/cheng/catkin_ws
主要源码目录: /home/cheng/catkin_ws/src/VINS-Mono
Docker 容器: vins_container

2. 核心架构与模块
软件包	路径	核心职责
vins_estimator	src/VINS-Mono/vins_estimator	后端优化。融合 IMU + 视觉 + UWB 约束（已移除 Lidar 深度因子）。
feature_tracker	src/VINS-Mono/feature_tracker	前端跟踪。单目视觉光流跟踪（已移除 Lidar 去畸变逻辑）。
benchmark_publisher	src/VINS-Mono/benchmark_publisher	发布真值 (Ground Truth)。已禁用自动对齐，改为手动 TF 或配置。

3. 数据集确认 (Dataset)
主用数据集: nya_01.bag（位于 /home/cheng/nya_01 或容器内 /data/nya_01.bag）。
UWB 锚点配置: 3个锚点系统 (ID 100, 101, 102)。

4. 当前开发状态 (截至 2025-12-22)
当前阶段: Phase 8 (UWB Anchor Self-Calibration & Visualization) 已完成。
核心精度: 
- 最佳 ATE (RMSE): 0.1988m (Plan B, Good Init).
- 稳定 ATE (RMSE): 0.50m (Plan B, Typical Init).
系统特性:
视觉: 单目特征点跟踪。
UWB: 
    - **自适应锚点标定 (Self-Calibration)**: 后端在线优化锚点位置 (para_Anchor)。
    - **坐标系定义 (Gauge Freedom)**: 
        - **Anchor 100**: 固定原点 (0, 0, 1.5)。
        - **Anchor 101**: 锁定 X 轴 (14.7, 0, 1.5)。使用 SubsetParameterization 锁定 Y/Z，仅优化 X（消除 Yaw 自由度）。
        - **Anchor 102**: 自由优化 (仅受三角测量约束，并在初始值给于 -Y 侧)。

5. 已实现的重大改进
A. UWB 锚点自标定 (Backend)
逻辑: 修改 `uwb_factor`，使其同时对 `para_Pose` 和 `para_Anchor` 求导。
策略: 解决了规范自由度 (Gauge Freedom) 问题，消除了 45 度/90 度旋转漂移。

B. 可视化完美对齐 (Rviz)
问题: UWB 世界坐标系 (Anchor Frame) 与真值坐标系 (Mocap Frame) 存在 90 度旋转和 1.5m 高度差。
解决方案: 
- 禁用 `benchmark_publisher` 的自动对齐 (不稳定)。
- 使用 `static_transform_publisher` 手动修正显示。
- **最佳参数**: Yaw = 1.24 rad, Z = -1.5 m。

C. 移除 Lidar 依赖 (Cleanup)
逻辑: 为了轻量化和测试 UWB 潜力，已完全移除 Lidar 深度因子和前端 Lidar 处理。

6. 遗漏的底层逻辑 (Logic Details)
UWB 噪声参数: std = 0.1 (保持不变)。
初始化: 依赖 `evaluate_viral.py` 进行 Sim3 对齐评估 ATE。

7. 学术与论文状态
论文方向: 强调 "Anchor Self-Calibration" 和 "Plug-and-Play UWB Integration"。
Demo 视频: Rviz 可视化已完全稳定，轨迹平滑且与 GT 重合，由于手动 TF 的锁定，无需担心演示时的随机抖动。

8. 运行指南 (Demo Mode)
启动计算: `roslaunch vins_estimator run_ntuviral.launch` (已内置 Manual TF)。
启动数据: `rosbag play /data/nya_01.bag`
启动 Rviz: `rosrun rviz rviz -d .../rviz_ntuviral.rviz`

9. 开发守则 (AI 强制执行)
坐标系定义: 严禁修改代码中的锚点初始化逻辑 (Plan B: 101 on X-axis)，这是物理真理。
显示调整: 若需微调 Rviz 显示，仅修改 launch 文件中的 static_transform_publisher 参数，勿动代码。

10. Runtime Environment
(同 v1.0，Docker 环境配置保持不变)
