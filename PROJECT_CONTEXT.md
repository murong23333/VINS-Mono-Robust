VINS-Mono (NTU VIRAL 扩展版) 项目环境上下文
AI 助手长期记忆文件 开启新对话时请优先读取此文件，以快速同步项目进度、核心配置与代码逻辑。

1. 项目基本信息
项目名称: VINS-Mono (NTU VIRAL 数据集适配版)
核心框架: ROS 1 (Kinetic/Melodic)
构建系统: Catkin
工作空间根目录: /home/cheng/catkin_ws
主要源码目录: /home/cheng/catkin_ws/src/VINS-Mono
Docker 容器: vins_container
2. 核心架构与模块
软件包	路径	核心职责
vins_estimator	src/VINS-Mono/vins_estimator	后端优化。融合 IMU + 视觉 + Lidar 深度 + UWB 约束。
feature_tracker	src/VINS-Mono/feature_tracker	前端跟踪。光流跟踪 + Lidar 去畸变 (Deskewing) + 深度关联。
benchmark_publisher	src/VINS-Mono/benchmark_publisher	发布真值 (Ground Truth) 用于精度评估。
3. 数据集确认 (Dataset)
主用数据集: nya_01.bag（位于 /home/cheng/nya_01 或容器内 /data/nya_01.bag）。
UWB 锚点配置: 在 nya_01 中，我们主要针对 ID 为 100 的锚点进行了配置。
4. 当前开发状态 (截至 2025-12-18)
当前阶段: Phase 7 已完成 (学术产出准备就绪)。
核心精度: ATE 0.38m (在 NTU VIRAL nya_01 数据集验证)。
系统特性:
视觉: 单目特征点跟踪（保持约 200 个特征点）。
激光: 稀疏深度关联（使用 Ouster OS1-16 作为深度计）。
UWB: 全局距离约束，消除长漂移（Topic: uwb_endorange_info）。
    - **锚点配置 (Anchor Configuration)**:
        - 系统设计使用 3 个锚点 (ID 100, 101, 102) 建立坐标系 {W}。
        - **Anchor 100**: 原点上方 1.5m (0, 0, 1.5)。
        - **Anchor 101**: X轴正方向同高度 (d_100_101, 0, 1.5)。
        - **Anchor 102**: Y轴负半区同高度 (通过三角测量确定)。
    - **初始化逻辑**: 依赖 `/uwb_exorange_info` 提供的 Anchor-to-Anchor 距离进行三角测量初始化 `initUwbAnchors()`。
5. 已实现的重大改进
A. Lidar 运动去畸变 (前端 feature_tracker)
逻辑: 在 feature_tracker_node.cpp 中通过高速 IMU 积分，将 Lidar 扫描周期内的每个点投影到当前帧图像时刻。
数据结构: 使用自定义 PointXYZIRT 结构体，利用 Ouster 的时间戳字段进行精确补偿。
B. 后端紧耦合与门控机制 (vins_estimator)
深度因子 (DepthFactor): 将关联的 Lidar 深度作为 Ceres 优化项。
门控策略: 仅当 $|1/d_{est} - 1/d_{lidar}| < 2.0$ 时才添加因子，防止异常点（如动态物体）打崩系统。
权重参数: 固定为 1.0 (实验证明权重 5.0 会导致系统初始化不稳定)。
C. 外参在线优化 (Online Refinement)
配置: estimate_extrinsic 设为 1 (周围自适应优化)。
结论: 该数据集外参精度极高，Mode 1 是最佳选择。
6. 遗漏的底层逻辑 (Logic Details)
UWB 噪声参数: 最终确定的 UWB 标准差 (std) 为 0.1。之前尝试 0.05 会导致 ATE 爆炸到 1.8m，请务必保持 0.1。
UWB 杆臂 (Tag Offset): 系统已支持从 /uwb_endorange_info 消息中实时提取 tag_offset 并在优化中生效（解决了质心不重合问题）。
IMU 缓存清理: feature_tracker 中加入了 imu_buf 清理逻辑（保留当前扫描时间前 0.2s 的数据），解决了因数据堆积导致的运行延迟。
7. 学术与论文状态
核心卖点: "轻量化 (Lightweight)"、"尺度鲁棒 (Scale-Aware)"、"Drift-Free (UWB 辅助)"。
重要文档:
论文初步大纲: paper_draft.md
投稿策略分析: paper_strategy.md
对比实验图表: /home/cheng/catkin_ws/vins_output/comparison.png
8. 运行指南 (三终端模式)
核心计算: roslaunch vins_estimator docker_nya_no_bag.launch
可视化: rosrun rviz rviz -d .../rviz_ntuviral.rviz (需配置 DISPLAY)
数据回放: rosbag play /data/nya_01.bag --clock
9. 开发守则 (AI 强制执行)
稳定性第一: 不要随意修改 Lidar 权重 (1.0) 或门控阈值 (2.0)。
环境依赖: 所有命令必须在 source /opt/ros/kinetic/setup.bash 后执行。
记忆存放: 详细任务记录位于 .gemini/antigravity/brain/5904de69-88ae-496c-8962-3022bc44e4a9/ 目录下的各 .md 文件。
10. 性能评估细节 (Evaluation)
评估脚本: evaluate_viral.py 内部实现了 Sim3 轨迹对齐。
结果解析: 即使 Rviz 中看到红绿线有固定平移偏置，只要评估脚本输出的 ATE 低（0.38m 左右），系统就是准确的，这种偏置是“世界坐标系定义”不同导致的。
11. Runtime Environment (Docker Container)
This project is explicitly designed to run inside a Docker container to ensure ROS 1 dependency compatibility.

- **Container Name**: `vins_container`
- **Execution Model**: All build and run commands must be executed via `docker exec -it vins_container ...`.
- **Critical Volume Mounts**:
    - **Codebase**: Host `/home/cheng/catkin_ws` is mounted to Container `/root/catkin_ws`. (Edits on host apply instantly).
    - **Data**: Host `/home/cheng/nya_01` (or similar) is mounted to `/data` for bag playback.
- **GUI Visualization (Rviz)**:
    - **Host Requirement**: Run `xhost +local:docker` on the host machine to allow container GUI access.
    - **Container Requirement**: Must set `export DISPLAY=:1` (or `:0` depending on host config) before running Rviz.
- **Network**: Uses host networking to communicate with local ROS tools if needed.
