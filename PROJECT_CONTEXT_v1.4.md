
VINS-Mono (NTU VIRAL 扩展版) 项目环境上下文 v1.4
AI 助手长期记忆文件（更新于 2025-12-25）

1. 项目基本信息
项目名称: VINS-Mono (NTU VIRAL 数据集适配版)
核心框架: ROS 1 (Kinetic/Melodic)
构建系统: Catkin
工作空间根目录: /home/cheng/catkin_ws
主要源码目录: /home/cheng/catkin_ws/src/VINS-Mono
Docker 容器: vins_container

2. 核心架构与模块
软件包	路径	核心职责
vins_estimator	src/VINS-Mono/vins_estimator	后端优化。融合 IMU + 视觉 + UWB 约束。**已集成鲁棒性增强 (HuberLoss)**。
feature_tracker	src/VINS-Mono/feature_tracker	前端跟踪。单目视觉光流跟踪。
benchmark_publisher	src/VINS-Mono/benchmark_publisher	发布真值 (Ground Truth)。
**Scripts (增强)**	/home/cheng/catkin_ws/	自动化测试组件 (`benchmark_10x.py`, `reproduce_ate.sh`)。

3. 数据集确认 (Dataset)
主用数据集: nya_01.bag（位于 /home/cheng/nya_01 或容器内 /data/nya_01.bag）。
**Topic 列表**: `/imu/imu`, `/left/image_raw`, `/right/image_raw`。

4. 当前开发状态 (截至 2025-12-25)
当前阶段: Phase 12 (System Robustness & Verification) COMPLETED.
核心指标: 
- **10-Run Benchmark**: 10次连续运行 ATE 均值 ~0.50m (Min 0.45m, Max 0.60m)。无失败案例。
- **稳定性**: 系统已具备从大幅漂移 (Drift > 20m) 和中途重置 (Mid-run Reset) 中自动恢复的能力。

5. 重大问题修复记录 (Bug Fixes) - 新增项
... (A-D 见 v1.3) ...

E. Benchmark Consistency (Process Pollution)
- **现象**: 第一次运行成功 (0.5m)，后续运行失败 (3.4m)。Docker 环境内残留了僵尸进程。
- **修复**: 在 `reproduce_ate.sh` 中引入激进的清理逻辑，使用 `pkill -f` (配合正则保护) 强制杀死 `rosmaster` 等核心进程。

F. System Robustness (Drift & Reset Recovery)
- **现象**: 偶发的 VIO 剧烈漂移或中途重置导致 UWB 被拒绝，系统最终发散 (ATE > 3m)。
- **原因**: 
  1. `GATE_THRESHOLD` (10m) 过于严格，在剧烈漂移时拒绝了救命的 UWB 数据。
  2. `CauchyLoss` 在大误差 (>10m) 时梯度消失，导致优化器无法拉回轨迹。
  3. 对齐阈值 (RMSE 0.5m) 过于严格，导致重置后的紧急对齐失败。
- **修复**: 
  1. **Disable Gate**: 将 Gate 阈值放宽至 100m (`estimator_node.cpp`)。
  2. **Robust Loss**: 为 UWB 引入 `HuberLoss(20.0)`，保证大误差下的强梯度恢复能力 (`estimator.cpp`)。
  3. **Relax Alignment**: 将对齐阈值放宽至 1.0m，优先保证系统闭环 (`initial_uwb_alignment.cpp`)。

6. 关键脚本说明
A. `reproduce_ate.sh`
- **功能**: Docker 内单次复现脚本。
- **增强**: 集成 `pkill` 清理逻辑，确保环境绝对净室 (Clean Room)。

B. `benchmark_10x.py`
- **功能**: 统计验证脚本。
- **状态**: 稳定运行，支持自动日志捕获 (`bench_run_X.log`) 和对齐状态监控。

7. 开发守则 (AI 强制执行)
- **UWB 必须开启**: 实验证明 UWB 对精度至关重要。
- **Loss Function**: 保持 UWB 使用 `HuberLoss` 以应对非线性漂移。
- **Process Cleanup**: 任何 Benchmark 脚本必须包含 `pkill` 清理步骤。

8. Runtime Environment
(同 v1.0，Docker 环境配置保持不变)
