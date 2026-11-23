# 双臂协同搬运：Windows MATLAB 30 cm 候选实现

此目录将原 Python 逐次线性化动力学 MPC 移植为 MATLAB R2023b。目标按用户要求设为直接上升 30 cm、持续 60 s，并保留双臂配对计划、HOLD 回退和轨迹保护。Ubuntu Gazebo 与原 `dualarm_hold_controller` C++ 执行插件仍作为仿真底层。MATLAB 控制进程不调用 Python。

**状态：数学核心、30 cm 名义预检和完整模拟状态机已验证；真实 ROS 2 自定义消息发布与 Gazebo 30 cm 动态执行尚未验证。** Windows MATLAB 已安装 ROS Toolbox，但 `ros2genmsg` 因未配置受支持的 C++ 编译器而停止。操作见 [RUN_30CM_WINDOWS_zh.md](RUN_30CM_WINDOWS_zh.md)。

## 代码与作用

| 文件 | 作用 |
|---|---|
| `lift_30cm.m` | 预检或有保护的 30 cm 执行入口；执行需要显式确认支撑移除和空间余量 |
| `+dualarm/Lift30Runner.m` | 当前状态建模、QP 求解、计划提交、60 s 轨迹、日志、回退与终点验收 |
| `+dualarm/RosTransport.m` | ROS 2 订阅和自定义力矩计划/提交发布 |
| `+dualarm/MpcStateCache.m` | `/clock`、关节、棍子及控制器状态的时间戳与顺序整理 |
| `+dualarm/submitPairedPlan.m`、`stopPaired.m` | 双臂 READY→COMMIT→ACTIVE 与 STOP→HOLD 协议 |
| `+dualarm/checkLiftGuard.m`、`composeTorque.m` | 轨迹、状态、抓取关系和命令限幅保护 |
| `+dualarm/loadArm.m`、`armGeometry.m`、`armEvaluate.m` | URDF、FK、Jacobian、质量、重力、科氏和阻尼；无需 Robotics System Toolbox |
| `+dualarm/rodInertia.m`、`localModel.m` | 棍子惯量与 18 维双臂刚性抓取约束动力学 |
| `+dualarm/solveMPC.m` | 相同的 36 维状态、12 维力矩、N=10、dt=0.05 的状态消元 QP |
| `+dualarm/liftReference.m`、`evalLiftReference.m` | 61 点 IK/PCHIP、30 cm/60 s 五次参考 |
| `+dualarm/frictionFeedforward.m`、`trackingBiasStep.m` | stage4.3 的摩擦近似、0.8 N·m 启动余量和残差估计 |
| `+dualarm/reanchorLift.m` | stage4.4 的沉降后起点与抓取变换重捕获 |
| `+dualarm/captureRobotURDF.m`、`makeStamp.m` 等 | 从 ROS `robot_description` 读取当前 URDF，构造消息时间戳 |
| `preview_nominal_lift.m` | 只读 61 点 IK + 1201 点名义 QP 全路径预检 |
| `smoke_ros2_readonly.m` | MATLAB 侧只读通信验证，不发布力矩 |
| `msg_source/dualarm_mpc_msgs/msg/*.msg` | 原 ROS 2 消息定义的副本，供 MATLAB `ros2genmsg` 使用 |
| `model_data/rod_with_pose.sdf` | MATLAB 侧棍子质量和惯量；运行前与 Ubuntu 实际 SDF 核对 |

`solveMPC.m` 保留原 QP 的目标、约束与状态消元。无约束最优解可行时直接使用；否则调用 MATLAB Optimization Toolbox `quadprog`，数值迭代器与 Python 的 OSQP 不同。数学模型和参数见 [MPC_30cm_论文模型_zh.md](MPC_30cm_论文模型_zh.md)。

## 已完成的离线验证

- Windows MATLAB R2023b Update 5：左右臂 FK、Jacobian、质量矩阵、重力与原 Python 数值模型差 ≤5.4e-15；闭链 `F,g,A,error,L,l` 最大差 ≤3.2e-14。
- 五个时刻的 IK 关节参考最大差 ≤1.2e-13 rad；静止参考首份力矩最大差约 7.7e-8 N·m。
- 30 cm/60 s：61 点 IK、1201 个名义 QP 点通过，无执行命令；最大关节变化 1.00917 rad、峰值名义速度 0.03125 rad/s、最小缩放 Jacobian 奇异值 0.11338。
- 离线约束分支调用 `quadprog`：约束残差 1.33e-15，首步力矩变化 0.227 N·m（限值 0.9 N·m）。模拟消息缓存、配对提交与停止回 HOLD 已检查。
- 完整 30 cm/60 s 的完美跟踪模拟运行通过，930 份配对计划后输出 `MATLAB_LIFT_30CM_PASS`，报告明确带 `test_mode=true`；这只检验 MATLAB 流程，不代表 Gazebo 实验成功。
- Windows 热运行中位耗时：静止模型约 3.45 ms、QP 约 3.65 ms；非零关节速度模型约 13.55 ms。Ubuntu 虚拟机与 Windows↔Ubuntu ROS 2 往返仍须实测。

`selftest_*.m` 及 `benchmark_core.m` 用于 MATLAB 离线验证；`compare_*.py` 只用于开发时交叉比对，MATLAB 控制进程运行时不调用它们。

## 当前阻断与运行要求

1. 为 Windows MATLAB 安装受支持的 C++ 编译器，运行 `mex -setup cpp`，再运行 `ros2genmsg('D:/myChatGPT/dualarm_mpc_matlab/msg_source')`。工具箱已校验消息定义，但尚未生成接口。
2. Ubuntu 启动正式 Gazebo 持棍场景；确认控制器版本、双臂 `enable_mpc=true`、支撑移除、模型世界原点、棍子稳定，以及旧 Python 力矩计划发布者已退出。
3. Windows MATLAB 先运行 `smoke_ros2_readonly` 和 `lift_30cm('preview',...)`。只有自定义消息和跨机状态真实可用、全路径预检通过，才执行 30 cm。
4. MATLAB 执行脚本仍须在 Gazebo 中验证配对计划、运动保护、接触滑移、求解/消息延迟及 HOLD 回退。原 Python 2 cm 运动曾因棍子误差超过 3 mm 停止；新的 30 cm 数学预检不能替代实际动态验收。
