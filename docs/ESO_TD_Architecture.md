# PX4 多旋翼 ESO+TD 姿态控制架构文档（v2）

## 1. 平台概述

| 项目 | 说明 |
|------|------|
| **飞控固件** | PX4 Autopilot (v1.15+, 自定义分支) |
| **目标机型** | 多旋翼 (Multicopter)，仿真使用 x500 四旋翼 |
| **仿真环境** | PX4 SITL + Gazebo (`make px4_sitl gz_x500`) |
| **地面站** | QGroundControl (QGC) |
| **控制频率** | 姿态环 ~250 Hz，角速度环 ~250 Hz |
| **操作系统** | Linux (Ubuntu) |

---

## 2. 整体控制架构

```
                    ┌─────────────────────────────────────────────────────────┐
                    │               AttitudeControl::update()                 │
                    │                                                         │
  遥控/位置控制器    │   ┌──────────┐                                          │
  ───→ qd ─────────┼──→│ Euler转换 │──→ euler_d ──→ ┌──────────────┐        │
  (期望四元数)       │   └──────────┘                 │  AttitudeTd  │        │
                    │                                │  (4阶TD)     │        │
                    │                                │              │        │
                    │                                │ euler_d → x1 │──┐     │
                    │                                │           x2 │──┤     │
                    │                                │           x3 │──┤     │
                    │                                │           x4 │  │     │
                    │                                └──────────────┘  │     │
                    │                                                  │     │
                    │   ┌─────── TD 输出解析 ────────────────────────┐ │     │
                    │   │                                            │ │     │
                    │   │  x1 ──→ Quatf(Eulerf(x1)) = qd           │◄┘     │
                    │   │         用于四元数P控制的平滑期望姿态        │       │
                    │   │                                            │       │
                    │   │  x1 ──→ W矩阵构造的参考角度(φ,θ)          │       │
                    │   │                                            │       │
                    │   │  x2 ──→ W × x2 = td_rate_sp (机体角速度)   │       │
                    │   │         ESO轨迹前馈                        │       │
                    │   │                                            │       │
                    │   │  x3 ──→ W×x3 + Ẇ×x2 = td_rate_accel      │       │
                    │   │         ESO加速度前馈 (机体角加速度)        │       │
                    │   └────────────────────────────────────────────┘       │
                    │                                                         │
                    │   ┌─────── 四元数P控制 ───────────────────────┐        │
                    │   │  qe = q⁻¹ × qd(来自x1)                   │        │
                    │   │  eq = 2 × qe.canonical().imag()           │        │
                    │   │  rate_setpoint = Kp ⊙ eq  (P修正)         │        │
                    │   └───────────────────────────────────────────┘        │
                    │                                                         │
                    │  输出:                                                  │
                    │    return rate_setpoint    ──→ PID路径 + ESO的rate_sp  │
                    │    _td_rate_sp_body        ──→ ESO的td_x2              │
                    │    _td_rate_accel_body     ──→ ESO的td_x3              │
                    └─────────────────────────────────────────────────────────┘
                                         │
                                         ↓
                    ┌─────────────────────────────────────────────────────────┐
                    │              RateControl::update()                      │
                    │                                                         │
                    │  输入: rate(陀螺仪), rate_sp(P修正),                    │
                    │        td_rate_sp(W×x2), td_rate_accel(W×x3+Ẇ×x2)     │
                    │                                                         │
                    │  ┌── ESO路径 ──────────────────────────────────┐       │
                    │  │  ESO闭环: run(gyro) → z1, z2, z_inertia    │       │
                    │  │  α = z_inertia + z2                        │       │
                    │  │                                             │       │
                    │  │  Tv1 = P1×(td_x2 + rate_sp - z1) + td_x3  │       │
                    │  │  Tv2 = P2×(td_x3 - α)                     │       │
                    │  │  Ta1 = P2×(Tv1 - α) + Tv2                 │       │
                    │  │  torque_eso = (z_inertia + T×Ta1) / b      │       │
                    │  │                                             │       │
                    │  │  ESO开环: updateControlInput(torque)        │       │
                    │  └─────────────────────────────────────────────┘       │
                    │                                                         │
                    │  ┌── PID路径 (并行热备) ────────────────────────┐       │
                    │  │  torque_pid = Kp×err + I - Kd×α + Kff×rsp  │       │
                    │  └─────────────────────────────────────────────┘       │
                    │                                                         │
                    │  模式选择器(RATE_CTRL_MODE) → torque_setpoint           │
                    └─────────────────────────────────────────────────────────┘
                                         │
                                         ↓
                                  控制分配器 → 电机PWM
```

---

## 3. TD 信号解析详解（核心）

### 3.1 AttitudeTd 4阶跟踪微分器

**文件:** `AttitudeTd.hpp` / `AttitudeTd.cpp`

**输入:** `euler_d = [φ_d, θ_d, ψ_d]`（从 `_attitude_setpoint_q` 转换的期望欧拉角）

**4阶积分链:**
```
x1' = x2          角度(rad)      → 角速度
x2' = x3          角速度(rad/s)  → 角加速度
x3' = x4          角加速度(rad/s²)→ jerk
x4' = T5          jerk(rad/s³)   → snap(驱动信号)
```

### 3.2 TD 每层的跟踪机制

每层 k 的控制信号 T_{k+1} 由非线性增益 `smoothKp` 产生：

```
e_k = target_k - x_k                        (标量: |e_k|)
T_{k+1} = n_k × smoothKp(|e_k|, P_k, r_k)  (方向 × 幅度)

smoothKp(err, P, r) = r × tanh(P/r × err)
```

**非线性特性:**
- 小误差 (`|e| << r/P`): `smoothKp ≈ P × err` — 线性比例控制
- 大误差 (`|e| >> r/P`): `smoothKp → ±r` — 饱和限幅

**每层的导数也被计算** (`smoothKp1`)，用于构造下一层的目标导数 `T_dot`。这使得4层之间的指令自洽：`T3 = d(T2)/dt` 的近似。

### 3.3 TD 各层信号的物理含义与参数

| 层 | 状态 | 跟踪目标 | 输出含义 | P增益 | 饱和r | 代码 |
|----|------|---------|---------|-------|-------|------|
| 1 | x1 → x2 | euler_d → x1 | T2: 期望角速度 | P1=8 | r1=10 rad/s | `smoothKp1(e1, P1, r1)` |
| 2 | x2 → x3 | T2 → x2 | T3: 期望角加速度 | P2=12 | r2=50 rad/s² | `smoothKp1(e2, P2, r2)` |
| 3 | x3 → x4 | T3 → x3 | T4: 期望jerk | P3=12 | r3=200 rad/s³ | `smoothKp1(e3, P3, r3)` |
| 4 | x4 → T5 | T4 → x4 | T5: snap(驱动) | P4=12 | r4=500 rad/s⁴ | `smoothKp0(e4, P4, r4)` |

**注意:** 第4层只计算0阶增益（不需要导数），因为T5是终端驱动信号。

### 3.4 TD 初始化

```cpp
if (landed || !_initialized) {
    _x1 = euler_d;    // x1直接设为当前期望角，无跳变
    _x2.zero();       // 速度/加速度/jerk 全部清零
    _x3.zero();
    _x4.zero();
    _T5.zero();
    _initialized = true;
}
```

**设计原因:** 起飞前 x1=euler_d 保证四元数P控制的 qd=Quatf(x1) 与实际姿态匹配，不产生虚假误差。

---

## 4. TD 输出的坐标变换

TD 输出 x1/x2/x3 是**欧拉角系**的信号。ESO/PID 在**机体系**工作。需要 W 矩阵转换。

### 4.1 W 矩阵定义

将欧拉角速率 `[φ̇, θ̇, ψ̇]` 转换为机体角速度 `[p, q, r]`：

```
    ┌   ┐     ┌                    ┐ ┌    ┐
    │ p │     │  1      0    -sinθ │ │ φ̇ │
    │ q │  =  │  0    cosφ  sinφcosθ│ │ θ̇ │
    │ r │     │  0   -sinφ  cosφcosθ│ │ ψ̇ │
    └   ┘     └                    ┘ └    ┘
```

**关键:** W 矩阵的角度参数来自 **TD 的 x1**（不是测量角），保证 W 和 x2/x3 自洽。

```cpp
const float phi   = x1(0);    // ← 来自TD x1，不是IMU测量
const float theta = x1(1);
```

### 4.2 信号①: td_rate_sp = W × x2

```cpp
body_rate_ff(0) = pd                    - st    * rd;       // p = φ̇ - sinθ·ψ̇
body_rate_ff(1) =       cp * qd_e       + sp * ct * rd;     // q = cosφ·θ̇ + sinφ·cosθ·ψ̇
body_rate_ff(2) =     - sp * qd_e       + cp * ct * rd;     // r = -sinφ·θ̇ + cosφ·cosθ·ψ̇

_td_rate_sp_body = body_rate_ff;   // → ESO 的 td_x2
```

**物理含义:** TD 轨迹规划的机体角速度。是 x1 的严格时间导数在机体系的投影。

### 4.3 信号②: td_rate_accel = W × x3 + Ẇ × x2

```cpp
// α_d = d(W×x2)/dt = W×x3 + Ẇ×x2   (链式法则)
_td_rate_accel_body(0) = pd3 - st*rd3 + (-ct*rd)*qd_e;
_td_rate_accel_body(1) = cp*qd3 + sp*ct*rd3
                        + pd*(-sp*qd_e + cp*ct*rd) + qd_e*(-sp*st*rd);
_td_rate_accel_body(2) = -sp*qd3 + cp*ct*rd3
                        + pd*(-cp*qd_e - sp*ct*rd) + qd_e*(-cp*st*rd);
```

**物理含义:** TD 轨迹规划的机体角加速度。是 `td_rate_sp` 的严格时间导数。

**Ẇ×x2 项的展开:**
- Ẇ 依赖 x1 的变化率（即 x2），所以 Ẇ×x2 包含 x2 的耦合项
- Roll 轴: `(-cosθ·ψ̇)·θ̇`
- Pitch 轴: `φ̇·(-sinφ·θ̇ + cosφ·cosθ·ψ̇) + θ̇·(-sinφ·sinθ·ψ̇)`
- Yaw 轴: `φ̇·(-cosφ·θ̇ - sinφ·cosθ·ψ̇) + θ̇·(-cosφ·sinθ·ψ̇)`

### 4.4 信号③: rate_setpoint = Kp ⊙ quat_error(q, Quatf(x1))

```cpp
Quatf qd = Quatf(Eulerf(x1(0), x1(1), x1(2)));  // ← 用TD x1构造期望四元数
const Quatf qe = q.inversed() * qd;               // 四元数误差
const Vector3f eq = 2.f * qe.canonical().imag();   // 小角度近似
Vector3f rate_setpoint = eq.emult(_proportional_gain);  // P修正
```

**物理含义:** 当前实际姿态 `q` 与 TD 平滑期望姿态 `x1` 之间的误差，经 P 增益放大后产生的修正角速度。

**关键设计:** qd 来自 **x1**（不是原始 `_attitude_setpoint_q`），这使得：
- rate_setpoint 和 td_rate_sp 来自同一条 TD 轨迹
- 姿态P控制看到的是平滑后的期望，不会因遥控器突变产生尖峰

### 4.5 三个信号的自洽性

```
td_rate_sp    = d(Quatf(x1))/dt  在机体系的投影    = W(x1) × x2
td_rate_accel = d(td_rate_sp)/dt                   = W(x1) × x3 + Ẇ(x1,x2) × x2
rate_setpoint = Kp × error(q, Quatf(x1))           = 当前姿态对x1轨迹的P修正
```

**全部基于 x1 轨迹:** 三个信号使用的期望姿态都是 TD 的 x1，W 矩阵的角度也来自 x1，保证了物理自洽。

---

## 5. ESO 控制律中 TD 信号的使用

### 5.1 控制律结构 (经典ADRC二阶状态反馈)

```
         ┌──────────────────────────────────────────────────────┐
         │          ESO 控制律 (每轴, 经典ADRC形式)              │
         │                                                      │
td_x2 ──→│  rate_err  = (td_x2 + rate_sp) - z1                 │
rate_sp ──→│             ~~~~~~~   ~~~~~~~    ~~                  │
         │             轨迹速度  P修正     ESO估计角速度         │
         │                                                      │
td_x3 ──→│  accel_err = td_x3 - α                              │
         │                        ~                             │
         │                    ESO估计总加速度(z_inertia+z2)     │
         │                                                      │
         │  Ta1 = P1×P2 × rate_err + (P1+P2) × accel_err       │
         │        ~~~~~~             ~~~~~~~~                   │
         │       速度误差增益        加速度误差增益(单次!)       │
         │                                                      │
         │  torque = (z_inertia + T × Ta1) / b                  │
         │                                                      │
         │  Anti-windup: 控制分配饱和时冻结ESO β₂自适应         │
         └──────────────────────────────────────────────────────┘
```

**vs 旧版结构 (已废弃):**
```
旧: Ta1 = P2×(Tv1-α) + P2×(td_x3-α)  → α被2×P2放大 (双重放大bug)
新: Ta1 = P1×P2×rate_err + (P1+P2)×accel_err  → α被(P1+P2)放大 (标准极点配置)
```

### 5.2 各信号在控制律中的角色

| 信号 | 注入位置 | 角色 |
|------|---------|------|
| **td_x2** (W×x2) | rate_err 的期望速度项 | 轨迹前馈: "我希望以这个角速度转" |
| **rate_sp** (Kp×err) | rate_err 的修正项 | 反馈修正: "当前姿态偏了，额外转一点修回来" |
| **td_x3** (W×x3+Ẇ×x2) | accel_err 的期望加速度 | 轨迹前馈: "角加速度应该是这么大" |
| **z1** (ESO角速度估计) | rate_err 的反馈项 | 状态反馈: "ESO认为实际角速度是这么大" |
| **α** (z_inertia+z2) | accel_err 的反馈项 | 扰动补偿: "当前总角加速度(含扰动)估计" |

### 5.3 增益链分析

```
Ta1 = P1×P2 × rate_err + (P1+P2) × accel_err
```

**特征多项式:** `s² + (P1+P2)s + P1×P2 = (s+P1)(s+P2)`
**闭环极点:** `-P1`, `-P2` (独立可调，稳定性有保证)

**总增益链（rate_err → torque）:**
```
torque = T/b × Ta1
       ≈ T/b × P1 × P2 × rate_err    (忽略加速度项)
       = 0.07/120 × 5 × 3 × rate_err
       = 0.00875 × rate_err
```

**动态增益（α估计误差放大）:**
```
α误差 → Ta1: -(P1+P2) = -8 倍  (旧版: -2×P2 = -6 倍, 但结构不正确)
α误差 → torque: -(P1+P2) × T/b = -0.00467 倍
```

### 5.4 Anti-Windup 机制

```
控制分配器饱和(正/负) → setSaturation(true) → ESO:
  ① 冻结 β₂ 自适应累积时间 (err_continues_time = 0)
  ② β₂_scale 回归 1.0 (不再放大)
  ③ z2 只靠基础 β₂ 校正，不会在饱和下加速膨胀

类似 PX4 PID 的 anti-windup:
  PID: 饱和时截断 rate_error 方向，阻止积分继续推深
  ESO: 饱和时冻结自适应增益，阻止 z2 继续放大
```

---

## 6. ESO 观测器详解

**文件:** `eso_angular_rate.hpp` / `eso_angular_rate.cpp`

### 6.1 状态

| 状态 | 含义 | 更新方式 |
|------|------|---------|
| z1 | 角速度估计 (rad/s) | 开环: `z1' = z_inertia + z2`; 闭环: `z1 += β1×err` |
| z2 | 外部扰动 (rad/s²) | 闭环: `z2 += β2×(err - last_err)` |
| z_inertia | 执行器响应 (rad/s²) | 开环: `z_inertia += dt/T × (b×u - z_inertia)` |

### 6.2 每周期时序

```
① run(gyro, dt, landed)         ← 闭环校正
   err = gyro - his_z1[0]       ← 8步延迟匹配
   z1 += β1 × err               ← 角速度校正
   z2 += β2 × (err-last_err)    ← 扰动校正

② 控制律计算                     ← 用 z1, z2, z_inertia
   Tv1 → Tv2 → Ta1 → torque

③ updateControlInput(torque)     ← 开环预测
   z_inertia += dt/T×(b×u - z_inertia)
   z1 += dt×(z_inertia + z2)    ← 为下周期做预测
```

### 6.3 8步延迟补偿

```
his_z1[0] = t-8步的z1预测值
his_z1[1] = t-7步的z1预测值
...
his_z1[7] = 当前z1

err = gyro_now - his_z1[0]    (陀螺仪有~8步滤波延迟)
```

校正时不仅更新 z1，还**回溯更新全部历史值**：
```
for k = 0..6:
    his_z1[k] = his_z1[k+1] + z1_correction + k×dt×z2_correction
```

### 6.4 自适应增益

```
z2_err 持续同方向 → err_continues_time 累加 → β2_scale = 1 + ζ₂×t³ → β2放大
最大5倍放大，t上限2秒
```

### 6.5 着陆初始化

```cpp
if (landed) {
    _z1 = v;              // 跟踪陀螺仪测量值，起飞无跳变
    _z2 = 0;              // 扰动清零
    _z_inertia = 0;       // 执行器状态清零
    for all: _his_z1[i] = v;  // 历史队列也初始化为测量值
}
```

---

## 7. 参数总表

### 7.1 TD 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| RATE_TD_P1 | 8.0 | x1跟踪euler_d的增益。越大→x1跟踪越快 |
| RATE_TD_P2 | 12.0 | x2跟踪目标速率的增益 |
| RATE_TD_P3 | 12.0 | x3跟踪目标加速度的增益。P4硬编码=P3 |

**硬编码饱和半径:** r1=10, r2=50, r3=200, r4=500

### 7.2 ESO 控制律反馈增益

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| RATE_FEED_R/P/Y_P1 | **5.0** | 速度层增益。rate_err放大倍数 |
| RATE_FEED_R/P/Y_P2 | **3.0** | 加速度层增益。**出现两次**(Ta1中P2²效应) |

### 7.3 ESO 观测器增益

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| RATE_ESO*_BETA1 | 0.6 | z1校正增益。越大→跟踪陀螺仪越快 |
| RATE_ESO*_BETA2 | 7.0 | z2校正增益。越大→扰动跟踪越快 |
| RATE_ESO*_CETA1 | 0.0 | β1自适应(禁用) |
| RATE_ESO*_CETA2 | 5000 | β2自适应系数 |

### 7.4 执行器模型参数

| 参数名 | Roll | Pitch | Yaw | 说明 |
|--------|------|-------|-----|------|
| RATE_ACT_T_* | 0.07 | 0.07 | 0.07 | 执行器时间常数(s) |
| RATE_ACT_B_* | 120 | 100 | 100 | 控制增益(rad/s²/norm) |

### 7.5 模式选择

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| RATE_CTRL_MODE | 0 | 0=全PID, 1=Roll ESO, 4=R+P ESO, 5=全ESO |

---

## 8. 关键文件索引

| 文件 | 功能 |
|------|------|
| `AttitudeTd.hpp/cpp` | 4阶TD: euler_d→x1(角度)/x2(速率)/x3(加速度)/x4(jerk) |
| `AttitudeControl.hpp/cpp` | 姿态环: TD跟踪 + W变换 + 四元数P控制 |
| `eso_angular_rate.hpp/cpp` | ESO: 角速度/扰动/执行器状态观测 |
| `rate_control.hpp/cpp` | 角速度环: ESO控制律 + PID并行 + 模式选择 |
| `mc_att_control_main.cpp` | 主循环: 参数加载 + 模块调度 |
| `mc_rate_control_params.c` | 参数定义与默认值 |

---

## 9. 操作流程

### 9.1 编译与运行
```bash
make px4_sitl_default          # 编译
make px4_sitl gz_x500          # 启动SITL仿真
```

### 9.2 模式切换（PX4 shell）
```
param set RATE_CTRL_MODE 0     # 全PID (安全)
param set RATE_CTRL_MODE 5     # 全ESO
```

### 9.3 参数调节
```
# TD — 控制轨迹平滑度
param set RATE_TD_P1 8.0       # 角度跟踪(越大→x1跟踪越快，x2越大)
param set RATE_TD_P2 12.0      # 速度跟踪
param set RATE_TD_P3 12.0      # 加速度跟踪

# ESO控制律 — 控制响应速度(⚠ P2出现两次，慎调)
param set RATE_FEED_R_P1 5.0   # 速度层增益
param set RATE_FEED_R_P2 3.0   # 加速度层增益

# ESO观测器 — 控制状态估计速度
param set RATE_ESOR_BETA1 0.6  # z1校正
param set RATE_ESOR_BETA2 7.0  # z2校正

# 执行器模型 — 控制torque输出缩放
param set RATE_ACT_T_R 0.07    # 时间常数
param set RATE_ACT_B_R 120.0   # 控制增益
```

---

## 10. 当前状态

### 已完成
- 4阶TD数值稳定 (饱和半径防止发散)
- TD初始化正确 (着陆时x1=euler_d)
- ESO初始化正确 (着陆时z1=测量值)
- PID模式正常飞行
- td_x2/td_x3物理自洽 (W变换+Ẇ项)
- 姿态P控制使用TD x1 (与x2/x3同源)
- P修正独立注入ESO Tv1层
- ESO反馈增益降低 (P1=5, P2=3)

### 待验证
- ESO模式起飞+悬停稳定性
- 扰动抑制效果
- 参数精细调优
