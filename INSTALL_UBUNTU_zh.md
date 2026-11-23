# Ubuntu 虚拟机部署前置条件

当前 Ubuntu 虚拟机 `command -v matlab` 没有输出；Windows MATLAB R2023b Update 5 已安装 Optimization Toolbox，但未安装 ROS Toolbox 和 Robotics System Toolbox。本目录的离线数学核心不依赖 Robotics System Toolbox；实时 ROS 2 通信需要 ROS Toolbox。

## 安装前确认

在 Ubuntu 终端读取系统资源：

```bash
uname -m
free -h
df -h ~
```

MathWorks 的 R2023b Linux 系统要求列出 Ubuntu 22.04 LTS，最低 8 GB 内存，推荐 16 GB。下载/安装 Linux 版 R2023b 时，选择 MATLAB、Optimization Toolbox、ROS Toolbox；需确认当前许可证包含这些产品。此工作区不修改 MATLAB 安装目录或许可证。

安装后验证：

```bash
command -v matlab
matlab -batch "ver; fprintf('ros2node=%d quadprog=%d lsqnonlin=%d\\n',exist('ros2node','file'),exist('quadprog','file'),exist('lsqnonlin','file'))"
```

三项函数应可用。若 `matlab` 未加入 PATH，用实际安装路径的 `bin/matlab` 执行。

## 自定义 ROS 2 消息

Ubuntu 现有 `~/ros_workspace/src/dualarm_mpc_msgs` 定义了 `TorquePlan`、`PlanCommit`、`ArmStatus`。为 MATLAB 消息生成准备单独目录（不修改 ROS 工作区）：

```bash
mkdir -p ~/matlab_ros_msgs
cp -a ~/ros_workspace/src/dualarm_mpc_msgs ~/matlab_ros_msgs/
```

在 Ubuntu MATLAB 中执行 `ros2genmsg(fullfile(getenv('HOME'),'matlab_ros_msgs'))`，按其输出把生成目录加入 MATLAB 路径并验证 `ros2 msg list` 能看到三类自定义消息。实际生成路径由 MATLAB 打印，不预设目录名。

## 数学核心复制与验证

将整个 `dualarm_mpc_matlab` 目录复制到 Ubuntu 某个位置，例如 `~/ros_workspace/matlab_mpc`。脚本使用 `addpath` 将目录加入 MATLAB 路径。先用 `selftest_model.m`、`selftest_local.m`、`selftest_reference.m`、`selftest_bias.m` 及 `selftest_preview.m` 做离线检查，再用 `smoke_ros2_readonly.m` 验证五类状态话题可收。参考 `selftest_*.m` 的路径目前指向 Windows 工作区，复制后需改为 Ubuntu 的 `/tmp/dualarm_effort.urdf` 和 `~/ros_workspace/src/dualarm_transport/models/rod_with_pose.sdf`。

现阶段 `smoke_ros2_readonly.m` 没有发布者，无法发送力矩。实时配对计划/停止/故障回退尚未移植，不能用离线预检结果直接控制棍子。
