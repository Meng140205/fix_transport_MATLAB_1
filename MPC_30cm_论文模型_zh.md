# 固定基座双机械臂协同搬运的局部动力学 MPC 方法

## 摘要

本文档按当前 `dualarm_mpc_matlab` 与对应 Python stage4.3/4.4 的实际计算描述固定基座双 EC66 搬运 0.15 kg 棍子的模型和控制器。控制输入为双臂 12 个关节的完整力矩；每个控制周期用实测关节及棍子状态重新建立局部线性化闭链动力学，求解有限时域凸二次规划。目标为棍子沿世界竖直方向在 60 s 内平滑上升 0.30 m，并保持水平位置、姿态及两侧抓取几何。此方法是**逐次局部线性化 MPC**，不是全时域非线性最优控制，也未包含物理接触摩擦锥或网格碰撞约束。

## 1. 系统与坐标

令 `i∈{L,R}` 表示左右 EC66。各臂关节位置、速度、加速度为 `q_i,dq_i,ddq_i∈R^6`，输入 `τ_i∈R^6`。棍子世界位姿 `T_WO∈SE(3)`，质心线速度 `v_O∈R^3`，世界角速度 `ω_O∈R^3`。两侧夹爪本体位姿 `T_WG_i(q_i)`。抓取静止阶段从实测位姿确定：

\[
T_{OG_i}=T_{WO}(t_0)^{-1}T_{WG_i}\!\left(q_i(t_0)\right).
\tag{1}
\]

其中 `T_OG_i` 在本轮 MPC 中作为固定刚性抓取几何。接触沉降停止后可在正式上升起点重新捕获一次，限值：棍子位移 ≤2 mm、姿态变化 ≤0.5°、关节变化 ≤0.001 rad、棍子线速度 ≤0.0002 m/s。若超过，拒绝开始上升。

机器人描述来自 `dualarm_effort.urdf.xacro` 展开的 URDF。各臂固定基座间距 1 m；手指在本轮实际闭合位置冻结并计入末端惯量。棍子 SDF 质量 `m_O=0.15 kg`，尺寸 `1.2×0.03×0.03 m`，质心坐标系惯量 `I_O=diag(2.25×10^-5,0.01801125,0.01801125) kg·m²`；世界惯量 `I_W=R_WO I_O R_WOᵀ`。重力加速度取 `9.81 m/s²`。

## 2. 双臂和棍子动力学

各臂满足关节空间欧拉–拉格朗日形式：

\[
M_i(q_i)\ddot q_i+C_i(q_i,\dot q_i)\dot q_i
+g_i(q_i)+D_i\dot q_i+\hat f_i
=\tau_i+J_i(q_i)^{\mathsf T}\lambda_i.
\tag{2}
\]

`M_i`、`g_i`、末端位姿与 Jacobian 由 URDF 连杆质量、质心及惯量计算；MATLAB 实现用质量矩阵对关节角的中心差分构造科氏项。棍子平动惯量为 `m_O I_3`，转动惯量为 `I_W`；角速度偏置包含 `ω_O×(I_W ω_O)`。棍子重力只计入一次。

将 `y=[q_L;q_R;p_O;θ_O]∈R^18`、`v=[dq_L;dq_R;v_O;ω_O]∈R^18`、`u=[τ_L;τ_R]∈R^12`，定义两侧夹爪刚性抓取的速度矩阵：

\[
A=
\begin{bmatrix}J_L&0&-G_L\\0&J_R&-G_R\end{bmatrix},
\qquad
G_i=
\begin{bmatrix}I_3&-[r_i]_\times\\0&I_3\end{bmatrix}.
\tag{3}
\]

`r_i` 为棍子中心指向第 `i` 个期望夹爪本体原点的世界向量，`[r]×` 为叉乘矩阵。`J_i` 统一采用 **[线速度；角速度]** 行顺序。令抓取位姿误差 `e∈R^12` 包括两侧平移误差和旋转对数误差，则局部约束动力学写作：

\[
\begin{bmatrix}M&-A^{\mathsf T}\\A&0\end{bmatrix}
\begin{bmatrix}\ddot y\\\lambda\end{bmatrix}
=
\begin{bmatrix}Bu-h\\-\dot A v-20e-8Av\end{bmatrix}.
\tag{4}
\]

这一步利用位置与速度误差反馈抑制闭链数值漂移。求解块矩阵得到当前工作点的仿射关系：

\[
\ddot y=F(q,\dot q,T_{WO},V_O)u+\bar g(q,\dot q,T_{WO},V_O).
\tag{5}
\]

每个 MPC 更新周期重算 `F,g_bar`，在本次 10 步预测时域内固定。接触约束反力 `λ=L u+l` 用于代价正则；没有加入摩擦锥、法向力上下限，也没有实测夹持力反馈。

### 2.1 摩擦与模型残差

对 12 个关节，预测偏置包含：

\[
\hat f_j=
\mu_j\tanh\!\left(\frac{\dot q^{\rm ref}_j}{0.0002}\right)
+m_j\tanh\!\left(\frac{\dot q^{\rm ref}_j}{0.0002}\right)
\exp\!\left[-\left(\frac{\dot q_j}{0.005}\right)^2\right]
+\hat b_j.
\tag{6}
\]

URDF 中每关节 `μ_j=3.5 N·m`、黏滞阻尼 `D_j=1.5 N·m·s/rad`。左右第 2、4 关节 `m_j=0.8 N·m`，其余为零。残差按仿真时间积分：

\[
\hat b_j^{+}=
\operatorname{clip}\!\left(
\hat b_j+8(q^{\rm ref}_j-q_j)\Delta t,\,-0.5,\,0.5
\right)\ {\rm N\,m}.
\tag{7}
\]

仅在运动阶段并收到新关节时间戳时更新。上述项在构造式 (4) 的 `h` 时加入，最终力矩仍满足 QP 限制。它们是当前实验的经验补偿，不能解释为已经辨识出的真实接触摩擦。

## 3. 参考轨迹与预测模型

以接触稳定后测得的棍子高度 `z_0` 为起点，令 `s=clip((t-t_lift)/60,0,1)`、`φ(s)=10s³-15s⁴+6s⁵`：

\[
\begin{aligned}
s&=\operatorname{clip}\!\left(\frac{t-t_{\rm lift}}{60},0,1\right),&
\phi(s)&=10s^3-15s^4+6s^5,\\
p_O^d(t)&=p_O(t_{\rm lift})+[0,0,0.30\phi(s)]^{\mathsf T},&
R_O^d(t)&=R_O(t_{\rm lift}).
\end{aligned}
\tag{8}
\]

起止参考速度、加速度均为零。沿 0–30 cm 在 61 个高度点求双臂六维 IK，保证对应夹爪的世界位置和姿态与刚性抓取参考相符；用保形三次样条 PCHIP 生成 `q_ref(t)`，由样条导数求 `dq_ref(t)`。离线参考路径的最大关节变化约 `1.009 rad`、峰值关节速度约 `0.03125 rad/s`、缩放 Jacobian 最小奇异值约 `0.11338`。

令局部预测状态 `x=[δy;v]∈R^36`，在 `Δt=0.05 s` 下使用常加速度离散化：

\[
x_{k+1}=
\begin{bmatrix}I&\Delta t I\\0&I\end{bmatrix}x_k+
\begin{bmatrix}\tfrac12\Delta t^2F\\\Delta t F\end{bmatrix}u_k+
\begin{bmatrix}\tfrac12\Delta t^2\bar g\\\Delta t\bar g\end{bmatrix}.
\tag{9}
\]

优化时域 `N=10`，长度 0.5 s。第 `k` 步目标按 `t+kΔt` 取值，因此时域内的上升参考随时间变化。

## 4. 有限时域优化

决策量为未来 10 步的 `U=[u_0;...;u_9]∈R^120`。状态消元 `X=SU+d` 后优化：

\[
\min_U\ \sum_{k=0}^{10}\|x_k-x_k^{\rm ref}\|_Q^2+
\sum_{k=0}^{9}\|u_k-u_{\rm previous}\|_R^2+
10^4\sum_{k=0}^{10}\|A\delta y_k+e\|^2+
10^{-4}\sum_{k=0}^{9}\|\lambda_k\|^2.
\tag{10}
\]

其中 `R=diag(0.5/τ_limit,j²)`；参考项按 `Q` 加权。左右关节位置权重均为 200、速度均为 20；棍子平移各轴 100000、姿态各轴 30000、线速度各轴 600、角速度各轴 100。实际 MATLAB 使用相同 Hessian/线性项与状态消元；若无约束最优解满足所有边界则直接采用，否则调用 `quadprog` 解相同的凸 QP。Python 原实现的受约束分支使用 OSQP，因此数值迭代器不同，优化问题相同。

| 约束 | 数值 |
|---|---:|
| 关节位置 | URDF 上下限留 0.01 rad；每次预测相对当前 ±0.15 rad |
| 棍子局部位置/姿态 | ±0.03 m / ±0.10 rad |
| 预测关节速度 | ±0.20 rad/s |
| 棍子预测线/角速度 | ±0.05 m/s / ±0.10 rad/s |
| 关节 1–3、4–6 绝对力矩 | ±150、±28 N·m |
| 相邻力矩变化率 | ≤18 N·m/s，即每步 0.9 N·m；首步相对当前控制器写入力矩 |

MATLAB 离线预检已在保存的 URDF/SDF 与代表性起点上完成 61 点 IK、1201 点名义 QP，最大约束残差为零，**没有发送执行命令**。这验证名义数学路径，不代表持棍接触、网格碰撞与真实消息延迟已通过。

## 5. 配对执行与保护

保持控制器的基线力矩从开始时实际输出构造，2 秒内按同一五次多项式 `α(t)` 混合：

\[
\tau_{\rm cmd}(t)=[1-\alpha(t)]\tau_H(t)+
\alpha(t)\tau_{\rm MPC}(t),\qquad
\alpha(t)=10s^3-15s^4+6s^5,\quad
s=\operatorname{clip}\!\left(\frac{t-t_0}{2},0,1\right).
\tag{11}
\]

两份常驻 C++ 控制器各占本臂 6 个 effort 接口。MATLAB 发送包含同一 session ID、计划 ID、状态/起始/失效仿真时间及 12 个**完整**力矩的 `TorquePlan`；两臂均反馈 READY 后发送 `PlanCommit`，并验证两臂 ACTIVE 与计划 ID 一致。每份计划有效期 0.12 s。停止请求或计划过期后，两臂回到 HOLD；MATLAB 必须观察两侧 `mode=HOLD`、`active_plan_id=0`。C++ 控制器不因 MATLAB 移植而改动。

在线保护沿用原方案：实际关节跟踪误差 0.02 rad、关节速度 0.10 rad/s、棍子三维位置误差 3 mm、姿态误差 1°、四指位置变化 0.5 mm、抓取位姿误差与数据陈旧/求解失败等。30 cm 终点验收还要求实测升高在目标 ±2 mm、关节速度 ≤0.01 rad/s、棍子线/角速度对应分量 ≤0.005，且 HOLD 持续稳定。

## 6. 方法边界与验收状态

物体由 Gazebo 接触物理夹住；式 (3)–(4) 用理想刚性抓取近似该接触。当前 MPC 没有自动网格碰撞检查、摩擦锥、夹持力传感器或完整非线性时域求解。先前 Python stage4.3 的 2 cm 运动曾因棍子相对夹爪沉降与后续位置误差超过 3 mm 中止。stage4.4 的运动起点重捕获只在沉降有界且趋稳时使用，不能掩盖持续滑移。

截至本文件编写，MATLAB 30 cm 版本的数学核心及完美跟踪模拟状态机已验证，模拟输出为 930 份配对计划、`MATLAB_LIFT_30CM_PASS`、`test_mode=true`；Windows MATLAB 端的自定义消息接口仍需 C++ 编译器生成，Gazebo 中 30 cm 实际协同搬运尚未完成。实验结论须以真实控制器 ACTIVE、棍子轨迹、最终 HOLD 及 `test_mode=false` 日志为准。
