# Windows MATLAB → Ubuntu Gazebo：30 cm 协同搬运操作顺序

本说明针对 MATLAB R2023b Update 5、Windows 上的 ROS Toolbox/Optimization Toolbox、Ubuntu 22.04 的 ROS 2 Humble/Gazebo。30 cm 数学核心已离线验证；**自定义消息编译、跨机实测与 Gazebo 动态搬运仍待完成**。代码只控制虚拟仿真，不连接实机。

## 1. 生成 MATLAB 自定义消息

Windows MATLAB 当前 `ros2genmsg` 因未配置受支持的 C++ 编译器而停止。通过 MATLAB“主页 → 附加功能”安装官方 `MATLAB Support for MinGW-w64 C/C++ Compiler`，然后在 MATLAB 中执行：

```matlab
mex -setup cpp
addpath('D:/myChatGPT/dualarm_mpc_matlab')
setup_messages
```

按生成器输出把 `matlab_msg_gen.../install/m` 目录加入 MATLAB 路径，并检查 `ros2 msg list` 可见 `dualarm_mpc_msgs/TorquePlan`、`PlanCommit`、`ArmStatus`。生成后重建 MATLAB ROS 2 节点。不要从不匹配的旧消息定义生成接口。官方 `ros2genmsg` 要求有效的 C++ 编译器。

## 2. Ubuntu 启动并确认正式场景

按已验证步骤启动 `simulate_dualarm_effort.launch.py`、Gazebo Play、加载支撑与 `transport_rod`、四指闭合到约 0.01 m、只移除 `rod_support`。等待至少 5 秒仿真时间，并在 Ubuntu 检查：

```bash
ign model --list
ign model -m dualarm_transport --pose
ros2 control list_controllers
ros2 param get /left_arm_hold_controller controller_revision
ros2 param get /right_arm_hold_controller controller_revision
ros2 param get /left_arm_hold_controller enable_mpc
ros2 param get /right_arm_hold_controller enable_mpc
ros2 topic echo --once /transport_rod/pose
ros2 topic info -v /dualarm_mpc/torque_plan
```

应有 `transport_rod`、没有 `rod_support`；机器人模型在 Gazebo 世界原点，五个控制器 active，两个版本均 `stage3_hover_v1`、两个开关均 true，棍子 Pose 连续。开始 MATLAB 预检前，`torque_plan` 没有其他发布者，旧 Python `lift_2cm.py` 等发送程序已退出。检查棍子上方 30 cm 全程以及两臂运动包络的障碍物余量；代码仍没有自动网格碰撞检测。

## 3. Windows MATLAB 只读通信和完整预检

在 MATLAB 命令窗口（将域 ID `0` 改成你通信测试使用的值）：

```matlab
addpath('D:/myChatGPT/dualarm_mpc_matlab')
smoke_ros2_readonly(0)
report = lift_30cm('preview',[],[],0)
```

`smoke_ros2_readonly` 只订阅 `/clock`、`/joint_states`、棍子 Pose 和左右状态。`lift_30cm('preview',...)` 从运行中的 `/robot_state_publisher` 读取 `robot_description`，使用本目录 `model_data/rod_with_pose.sdf`，对本轮 q/棍子起点做 61 点 IK 和 1201 点名义 QP；命令发布者不创建、发送计划数为零。需要确认本地 SDF 与 Ubuntu 正在加载的棍子质量 0.15 kg、惯量对角 `[2.25e-5,0.01801125,0.01801125]` 一致。预检成功应包含 `MATLAB_NOMINAL_LIFT_PRECHECK_PASS`、`height_m=0.30`、`duration_s=60`、`actuator_commands_sent=0`。

## 4. 直接 30 cm 仿真执行

在当前场景仍稳定、上一步全部通过时运行：

```matlab
result = lift_30cm('execute',[],[],0,true,true)
```

末尾两个 `true` 表示操作者已确认支撑移除及 30 cm 路径空间余量。程序会重新预检，配对发送同一 12 维力矩计划、2 秒平滑接管、1 秒静止 MPC、从稳定后的棍子位置用 60 秒仿真时间上升 30 cm、终点保持 2 秒、返回 HOLD 再观察 2 秒。每份计划有效期 0.12 s；任何状态陈旧、求解失败、双臂计划不同步、关节或棍子误差越界都会请求停止。

成功必须出现 `MATLAB_LIFT_30CM_PASS` 且 `test_mode=false`、实测抬升 0.298–0.302 m、末端稳定、双臂 HOLD。CSV 写在 MATLAB 当前工作目录，名为 `matlab_lift30_*.csv`。若 MATLAB 输出 `MATLAB_LIFT_30CM_STOPPED`、`HOLD not confirmed`，或 Gazebo 中持续下沉/滑移/碰撞，请暂停 Gazebo 并保留首个错误和 CSV；不要增大阈值继续试。

**运行状态：**截至编写此说明，Windows MATLAB 尚缺可用于 `ros2genmsg` 的 C++ 编译器，`lift_30cm` 的真实 ROS 2 消息收发与仿真控制未验证。完成第 1 步后，先把消息生成结果和只读通信结果交给 Codex 核对，再执行第 4 步。
