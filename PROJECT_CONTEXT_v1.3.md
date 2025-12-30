
VINS-Mono (NTU VIRAL 扩展版) 项目环境上下文 v1.3
AI 助手长期记忆文件（更新于 2025-12-24）

1. 项目基本信息
项目名称: VINS-Mono (NTU VIRAL 数据集适配版)
核心框架: ROS 1 (Kinetic/Melodic)
构建系统: Catkin
工作空间根目录: /home/cheng/catkin_ws
主要源码目录: /home/cheng/catkin_ws/src/VINS-Mono
Docker 容器: vins_container

2. 核心架构与模块
软件包	路径	核心职责
vins_estimator	src/VINS-Mono/vins_estimator	后端优化。融合 IMU + 视觉 + UWB 约束。**UWB 功能已验证有效**。
feature_tracker	src/VINS-Mono/feature_tracker	前端跟踪。单目视觉光流跟踪。
benchmark_publisher	src/VINS-Mono/benchmark_publisher	发布真值 (Ground Truth)。
**Scripts (新增)**	/home/cheng/catkin_ws/	自动化脚本群 (`run_viz_eval.sh`, `benchmark_10x.py` 等)。

3. 数据集确认 (Dataset)
主用数据集: nya_01.bag（位于 /home/cheng/nya_01 或容器内 /data/nya_01.bag）。
**Topic 列表**: `/imu/imu`, `/left/image_raw`, `/right/image_raw`。 (Launch 文件已修正匹配此列表)

4. 当前开发状态 (截至 2025-12-24)
当前阶段: Phase 11 & 12 (System Stability & Automated Benchmarking) Completed.
核心精度: 
- **Strict Gate (0.5m) + UWB**: ATE ~0.53m (Mean).
- Pure VINS (No UWB): ATE ~1.10m.
- **结论**: UWB 即使在严苛过滤下也提供了 ~50% 的精度提升。

5. 重大问题修复记录 (Bug Fixes)
A. UWB Logic Bug (Scale Collapse)
- **现象**: ATE 异常高 (3.6m)，Scale 漂移至 ~0.11 (Scale Collapse)。
- **原因**: `estimator.cpp` 中 `optimization()` 函数错误地将 `uwb_initialized` 重置为 `false`，导致首次优化后 UWB 被彻底禁用，系统退化为无尺度的单目 VIO。
- **修复**: 删除 `estimator.cpp` 中错误的 `uwb_initialized = false;` 代码。验证后 ATE 恢复至 ~0.51m。

B. Rosbag Play 话题不匹配 (Launch Failure)
- **现象**: 运行自动化脚本时 `rosbag play` 直接退出，提示 "No messages to play".
- **原因**: Launch 文件请求 `/imu/data` 等通用名，但 Bag 文件实际为 `/imu/imu` 等特定名。
- **修复**: 修正 `run_ntuviral.launch` 中的 `--topic` 参数以匹配 Bag 文件。

C. 评估脚本兼容性 (Python 2/3)
- **现象**: Docker 内仅有 Python 3.5 (报错 `f-string`) 或 Python 2 (无 Scipy)。
- **修复**: 重构 `evaluate_viral.py` 去除 Scipy 依赖，手动实现四元数运算，移除 f-string，使其兼容 Python 2。

D. 系统抖动 (Jitter)
- **现象**: 初始阶段 VINS 轨迹剧烈抖动。
- **原因**: `rosbag play` 倍速设置为 `2.0x`，导致 IMU 帧率过高/不稳。
- **修复**: 回退至 `1.0x` 播放速度。

6. 关键脚本说明 (New)
A. `run_viz_eval.sh`
- **功能**: 一键运行 VINS + Rviz + 自动评估。
- **命令**: `docker exec -it vins_container /bin/bash -c "/root/catkin_ws/run_viz_eval.sh"`
- **特性**: 包含 `killall rosmaster` 自动清理残留进程。

B. `benchmark_10x.py`
- **功能**: 连续运行 10 次 Benchmark，统计平均 ATE。
- **特性**: 自动调用 `reproduce_ate.sh` (Headless 模式)，解析输出。

7. 开发守则 (AI 强制执行)
- **UWB 必须开启**: 实验证明 UWB 对精度至关重要。
- **Launch 文件**: 必须确保 Topic 名称与 Bag 文件一致。
- **Playback Speed**: 保持 `1.0x` 以确保初始化稳定。
- **Docker 路径**: 脚本中必须使用 `/root/catkin_ws` 而非宿主机路径。

8. Runtime Environment
(同 v1.0，Docker 环境配置保持不变)
