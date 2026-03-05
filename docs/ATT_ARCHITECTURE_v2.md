# PX4 多旋翼姿态控制系统架构文档

> 生成日期: 2026-03-05 | 基于当前代码实际状态
> 范围: 姿态外环 (AttitudeControl) + 角速度内环 (RateControl) + TD + ESO

---

## 一、系统总览

### 1.1 控制链路全景

```
vehicle_attitude_setpoint (q_d, thrust)      ← 位置控制器或手动模式生成
        │
        ▼
┌───────────────────────────────────────────────────────────────────┐
│           MulticopterAttitudeControl::Run()                       │
│           文件: mc_att_control_main.cpp                           │
│           触发: vehicle_angular_velocity (400 Hz)                 │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │ 外环: AttitudeControl::update()          100 Hz (10ms分频)  │  │
│  │   ├─ 4阶TD: 参考信号平滑 + 角速度/角加速度微分估计         │  │
│  │   ├─ 四元数误差 → P控制 → rate_setpoint                    │  │
│  │   └─ W矩阵 → LPF → Ramp → td_rate_sp, td_rate_accel       │  │
│  └────────────────────────┬────────────────────────────────────┘  │
│                           │                                       │
│  ┌────────────────────────▼────────────────────────────────────┐  │
│  │ 内环: RateControl::update()              400 Hz             │  │
│  │   ├─ PID路径:   Kp·e + Ki·∫e - Kd·α + Kff·rate_sp         │  │
│  │   ├─ ADRC路径:  ESO闭环→控制律→执行器反解→TD前馈           │  │
│  │   └─ 模式选择器 (RATE_CTRL_MODE) → torque_setpoint          │  │
│  └────────────────────────┬────────────────────────────────────┘  │
│                           │                                       │
│                           ▼                                       │
│            vehicle_torque_setpoint + vehicle_thrust_setpoint       │
└───────────────────────────┬───────────────────────────────────────┘
                            │
                            ▼
                   Control Allocator → actuator_motors
```

### 1.2 频率结构

| 组件 | 频率 | 触发方式 |
|------|------|----------|
| Gyro 发布 (`vehicle_angular_velocity`) | 400 Hz | `IMU_GYRO_RATEMAX` 参数 |
| **主循环 `Run()`** | 400 Hz | `SubscriptionCallbackWorkItem` |
| **RateControl + ESO** | 400 Hz | 每次 `Run()` 都执行 |
| **AttitudeControl + TD** | **100 Hz** | `hrt_elapsed_time >= 10ms` 分频 |
| PositionControl (上游) | ~50 Hz | 独立模块 |

### 1.3 带宽层级 (严格递减)

```
ESO 观测器 (~15-20 Hz) >> Rate 内环 (~3 rad/s) >> Attitude 外环 (~0.6 Hz) ≈ TD (~0.5 Hz)
```

---

## 二、mc_att_control 主循环

### 2.1 模块信息

| 项目 | 值 |
|------|------|
| **文件** | `src/modules/mc_att_control/mc_att_control_main.cpp` / `.hpp` |
| **类名** | `MulticopterAttitudeControl` |
| **基类** | `ModuleBase`, `ModuleParams`, `px4::WorkItem` |
| **触发源** | `vehicle_angular_velocity` 订阅回调 (400Hz) |
| **工作队列** | `px4::wq_configurations::rate_ctrl` |

### 2.2 Run() 分频机制

```cpp
// 400 Hz: 主循环触发 (每次 gyro 数据到达)
if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

    // === 100 Hz: 姿态外环 (含 TD) ===
    if (_vehicle_attitude_sub.update(&v_att)) {
        if (run_att_ctrl) {
            if (hrt_elapsed_time(&_last_att_update) >= 10000) {  // 10ms = 100Hz
                _last_att_update = now_att;
                att_dt = dt * 4.0f;  // ~10ms

                rates_sp = _attitude_control.update(q, att_dt, landed);
                _rates_setpoint = rates_sp;
            }
            // 非更新周期: _rates_setpoint 保持上次计算值
        }
    }

    // === 400 Hz: 角速度内环 (含 ESO) ===
    if (flag_control_rates_enabled) {
        torque = _rate_control.update(rates, _rates_setpoint, angular_accel,
                                      dt, landed, td_rate_sp, td_rate_accel,
                                      angular_velocity.timestamp_sample);
    }
}
```

### 2.3 模式分支

| 条件 | 行为 |
|------|------|
| Manual + attitude_enabled + 无位置/速度控制 | **Stabilized**: `generate_attitude_setpoint()` 从遥控杆生成 q_d |
| attitude_enabled + 自动/辅助 | 读取外部 `vehicle_attitude_setpoint` (来自位置控制器) |
| Manual + !attitude_enabled | **ACRO**: 遥控杆 → superexpo → 直接输出 rate_sp |
| rates_enabled | 运行 `RateControl::update()` → torque_setpoint |

### 2.4 Stabilized 手动模式姿态生成

`generate_attitude_setpoint()` 流程:

```
RC stick roll/pitch
    → AlphaFilter (τ = MC_MAN_TILT_TAU = 0.4s)     ← 低通防止 RC 阶跃激发 TD
    → 倾斜向量 v = (roll × tilt_max, -pitch × tilt_max), 限幅 MPC_MAN_TILT_MAX
    → q_sp_rp = AxisAngle(v(0), v(1), 0)            ← 纯倾斜四元数
    → StickYaw: yaw_rate → 积分 → yaw_setpoint      ← 偏航速率限制
    → (VTOL: correctTiltSetpointForYawError)         ← yaw 误差对倾斜补偿
    → q_sp = q_sp_yaw × q_sp_rp                     ← 合成最终姿态设定值
    → thrust = throttle_curve(stick)
    → 发布 vehicle_attitude_setpoint
```

### 2.5 饱和反馈链路

```
control_allocator_status
    │
    ├─► sat_positive[3], sat_negative[3]
    │
    ├─► RateControl::setSaturationStatus()
    │       ├─► ESO::setSaturation(pos, neg)  → z2 方向截断 + β₂ 冻结
    │       └─► PID updateIntegral()           → 积分方向截断
    │
    └─► ESO::updateControlInput(torque_actual)  → 实际力矩更新执行器模型
```

---

## 三、外环: AttitudeControl (100 Hz)

### 文件
- `AttitudeControl/AttitudeControl.hpp` / `.cpp` — 类定义与实现
- `AttitudeControl/AttitudeControlMath.hpp` — VTOL yaw 补偿辅助函数

### 算法来源
> Nonlinear Quadrocopter Attitude Control (2013), ETH Zurich IDSC

### update() 9步流程

```
                     ┌──────────────────┐
   q_d (setpoint)    │  Step 1: TD      │
   ──────────────────►  input=Euler(q_d) │ ← 纯参考信号,不含测量态 q
                     │  4th-order        │
                     │  P1=3, P2=4, P3=5│
                     └──┬───┬───┬───────┘
                        │   │   │
                       x1  x2  x3
                        │   │   │
               ┌────────┘   │   │
               ▼            ▼   ▼
    ┌──────────────┐  ┌────────────────────────────┐
    │ Step 2:      │  │ Step 6: W 矩阵             │
    │ qd=Quat(x1)  │  │  body_ω = W × x2           │
    │ (平滑四元数) │  │  body_α = W × x3 + Ẇ × x2 │
    └──────┬───────┘  └──────┬──────────┬───────────┘
           │                 │          │
           ▼                 ▼          ▼
    ┌──────────────┐  ┌─────────┐ ┌─────────┐
    │ Step 3-4:    │  │Step 7:  │ │Step 7:  │
    │ Yaw优先级    │  │LPF 20Hz│ │LPF 20Hz│
    │ 四元数误差P  │  └────┬────┘ └────┬────┘
    │              │       │           │
    │ rate_p =     │  ┌────┴────┐ ┌───┴─────┐
    │   Kp ⊙ eq   │  │Step 8:  │ │Step 8:  │
    └──────┬───────┘  │Ramp 2s  │ │Ramp 2s  │
           │          └────┬────┘ └────┬────┘
    ┌──────▼───────┐       │           │
    │ Step 5:      │  ┌────▼────┐ ┌───▼──────┐
    │ rate限幅     │  │Step 9:  │ │Step 9:   │
    └──────┬───────┘  │±3 rad/s │ │±10 rad/s²│
           │          └────┬────┘ └────┬─────┘
           ▼               ▼           ▼
       rate_sp        td_rate_sp   td_rate_accel
      (P校正输出)     (前馈 ω)     (前馈 α)
           └───────┬───────┴───────────┘
                   ▼
          传递给 RateControl::update()
```

### 各步骤详解

**Step 1 — TD 跟踪微分器** (详见第四节):
- 输入: `Euler(q_d)` — 纯参考信号，**不含测量姿态 q**
- 输出: x1 (平滑角度), x2 (角速率), x3 (角加速度)

**Step 2 — 平滑四元数构造**:
- `qd = Quaternion(Euler(x1(0), x1(1), x1(2)))`
- 当 TD 收敛: x1 → euler_d, 故 qd ≈ q_setpoint

**Step 3 — Yaw 优先级分离**:
- 提取 tilt 分量 (dcm_z): `qd_red` = 纯 tilt, `qd_dyaw` = 纯 yaw
- Yaw 降权: `qd = qd_red × Quat(cos(yaw_w·acos(qd_dyaw(0))), 0, 0, sin(yaw_w·asin(qd_dyaw(3))))`
- `MC_YAW_WEIGHT=0.4` → Yaw 控制力矩弱, 降权避免抢占 Roll/Pitch

**Step 4 — P 控制**:
- 四元数误差: `qe = q.inversed() × qd`
- 向量化: `eq = 2 × qe.canonical().imag()` (≈ 小角度误差)
- `rate_p = eq ⊙ [MC_ROLL_P, MC_PITCH_P, MC_YAW_P/yaw_w]`
- 叠加 yaw 速率前馈: `rate_p += q⁻¹.dcm_z() × yawspeed_setpoint`

**Step 5 — Rate 限幅**: `clamp(rate_p, ±[ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX])`

**Step 6 — W 矩阵 (Euler → Body)**:
```
W = [ 1      0       -sinθ     ]
    [ 0    cosφ    sinφ·cosθ   ]
    [ 0   -sinφ    cosφ·cosθ   ]

body_ω = W × x2,   body_α = W × x3 + Ẇ × x2
```

**Step 7 — 20Hz LPF**: `y += alpha × (x - y)`, `alpha = dt/(dt + 1/(2π×20))`

**Step 8 — 2s 起飞 Ramp**: `ff_alpha = clamp(t_since_takeoff / 2.0, 0, 1)`, 着陆时清零

**Step 9 — 硬限幅 + 输出**:
- `td_rate_sp = clamp(ff_alpha × filtered_body_ω, ±3.0 rad/s)`
- `td_rate_accel = clamp(ff_alpha × filtered_body_α, ±10.0 rad/s²)`

**关键设计原则**: TD **只做前馈**, rate_sp 来自 P 控制, td_rate_sp/accel 独立叠加到内环力矩。

---

## 四、4阶跟踪微分器 (AttitudeTd)

### 文件
- `AttitudeControl/AttitudeTd.hpp` / `.cpp`

### 积分链结构

```
输入: ref_d = Euler(q_setpoint)    ← 纯参考信号 (Euler 域, 3轴独立)

x1' = x2       (角度 → 角速度)
x2' = x3       (角速度 → 角加速度)
x3' = x4       (角加速度 → jerk)
x4' = T5       (jerk → snap, 驱动信号)

输出:
  x1 — 平滑欧拉角 [rad],     → 构造 q_smooth 用于姿态 P 控制
  x2 — 期望角速率 [rad/s],   → W矩阵转机体系 → 前馈给 ESO
  x3 — 期望角加速度 [rad/s²],→ W矩阵转机体系 → 前馈给 ESO
```

### 4层跟踪机制

```
Layer 1: e1 = ref_d - x1 ──► smoothKp1(|e1|, P1, r1=10)  ──► T2 (速度指令)
Layer 2: e2 = T2 - x2    ──► smoothKp1(|e2|, P2, r2=50)  ──► T3 (加速度指令)
Layer 3: e3 = T3 - x3    ──► smoothKp1(|e3|, P3, r3=200) ──► T4 (jerk 指令)
Layer 4: e4 = T4 - x4    ──► smoothKp0(|e4|, P4, r4=500) ──► T5 (snap)

欧拉积分: x1+=x2·dt, x2+=x3·dt, x3+=x4·dt, x4+=T5·dt
```

### 非线性增益函数

```
smoothKp0(e, P, r) = r × tanh(P/r × e)
```
- 线性区 (e ≪ r/P): ≈ P × e (正常比例增益)
- 饱和区 (e ≫ r/P): ≈ r (限幅防发散)

### 饱和半径 (防级联发散)

| 层 | r 值 | 物理限制 |
|----|------|----------|
| r1 | 10 rad/s | 角速度限制 |
| r2 | 50 rad/s² | 角加速度限制 |
| r3 | 200 rad/s³ | jerk 限制 |
| r4 | 500 rad/s⁴ | snap 限制 |

### 初始化与着陆保护
- 首次调用或着陆: `x1 = ref_d`, x2/x3/x4/T5 全部清零
- 防止起飞瞬间产生大跳变

---

## 五、内环: RateControl (400 Hz)

### 文件
- `src/lib/rate_control/rate_control.hpp` / `.cpp`

### 双路径架构

```
       rate_sp (来自姿态P)     td_rate_sp / td_rate_accel (来自TD)
            │                        │              │
  ┌─────────▼──────────┐   ┌────────▼──────────────▼────────────┐
  │   PID 路径         │   │   ESO/ADRC 路径                    │
  │                    │   │                                     │
  │ e = rate_sp - ω    │   │ ① ESO闭环: run(ω,dt,ts)→z1,z2,zi  │
  │                    │   │ ② rate_err = rate_sp - z1           │
  │ torque =           │   │   accel_err = -z2                   │
  │   Kp·e             │   │ ③ Ta1 = P1·P2·rerr + (P1+P2)·aerr │
  │ + Ki·∫e            │   │ ④ torque = (zi + T·Ta1) / b        │
  │ - Kd·angular_accel │   │ ⑤ += 0.15·td_x2 + 0.05·td_x3     │
  │ + Kff·rate_sp      │   │ ⑥ 限幅 [-1, 1]                    │
  └─────────┬──────────┘   └──────────────────┬─────────────────┘
            │                                  │
            ▼                                  ▼
  ┌───────────────────────────────────────────────────────┐
  │  模式选择器 (RATE_CTRL_MODE)                          │
  │  0: 全PID  1: R=ESO  2: P=ESO  3: Y=ESO              │
  │  4: RP=ESO  5: 全ESO                                  │
  └─────────────────────┬─────────────────────────────────┘
                        ▼
                  torque_setpoint
                        │
  ┌─────────────────────┼─────────────────────────────────┐
  │ ⑦ PID积分器更新 (所有模式, 三重保护)                 │
  │ ⑧ ESO开环预测 (所有模式, 保持ESO热备)               │
  │   updateControlInput(torque_actual, now_us)            │
  └─────────────────────┼─────────────────────────────────┘
                        ▼
              vehicle_torque_setpoint → 控制分配器
```

### ADRC 控制律数学

**误差定义** (TD 不进入误差, 只做前馈):
```
rate_err  = rate_sp - z1         (角速度误差)
accel_err = -z2                  (仅扰动, 不含 z_inertia 避免二次补偿)
```

**状态反馈**:
```
Ta1 = P1·P2 × rate_err + (P1+P2) × accel_err

特征多项式: s² + (P1+P2)·s + P1·P2 = (s+P1)(s+P2)
闭环极点: -P1, -P2 (独立可调)
当前: P1=P2=3 → 临界阻尼, ω_c ≈ 3 rad/s ≈ 0.5 Hz
```

**力矩输出**:
```
torque_adrc = (z_inertia + T × Ta1) / b      ← z_inertia 仅在此处补偿一次
torque_final = torque_adrc + 0.15·td_x2 + 0.05·td_x3
               (限幅到 [-1, 1])
```

### PID 积分器三重保护

| 机制 | 描述 |
|------|------|
| 饱和方向截断 | 控制分配器+sat → 误差只允许负向积分 |
| 非线性衰减 | `i_factor = 1 - (rate_err/400°)²`, 大误差时积分弱化 |
| 硬限幅 | `constrain(rate_i, ±lim_int)` |

---

## 六、ESO 扩展状态观测器

### 文件
- `src/lib/rate_control/eso_angular_rate.hpp` / `.cpp`
- 每轴一个实例: `acfly_eso_roll`, `acfly_eso_pitch`, `acfly_eso_yaw`

### 内部状态

| 状态 | 含义 | 单位 |
|------|------|------|
| `z1` | 角速度估计 | rad/s |
| `z2` | 外部扰动估计 | rad/s² |
| `z_inertia` | 执行器惯性响应 (一阶模型) | rad/s² |

### 每周期调用时序

```
① run(ω_gyro, dt, landed, timestamp_sample)     ← 闭环校正
    ├─ lookupZ1AtTimestamp(timestamp_sample)       从环形缓冲查找时间对齐的 z1
    ├─ err = ω_gyro - z1_at_sample                 观测误差
    ├─ 自适应增益 β₂ 调节 (基于误差持续性)
    ├─ z1 += β₁ × err × dt                        角速度校正
    └─ z2 += β₂_eff × err × dt                    扰动校正 (含方向截断)

② 控制律计算 torque_setpoint                      ← 在 RateControl 中

③ updateControlInput(torque_actual, now_us)        ← 开环预测
    ├─ z_inertia += dt/T × (b·u_actual - zi)       执行器一阶模型
    ├─ z1 += dt × (zi + z2)                         角速度预测
    └─ ring_buffer[head] = (now_us, z1)             存入环形缓冲
```

### 延迟补偿: 时间戳环形缓冲

```
16 槽时间戳环形缓冲 (ESO_HIS_BUF_LEN = 16)
400Hz 下覆盖最大 40ms 延迟

lookupZ1AtTimestamp(gyro_ts):
  → 遍历缓冲, 找 |t_buf - gyro_ts| 最小的条目
  → 返回该条目的 z1 作为 z1_at_sample
```

### Anti-Windup 三重机制

| 机制 | 描述 | 触发条件 |
|------|------|----------|
| **u_actual** | z_inertia 模型使用实际施加力矩 (非期望值) | 始终 |
| **z2 方向截断** | +sat → z2 只能减小; -sat → z2 只能增大 | `sat_positive` / `sat_negative` |
| **β₂ 冻结** | 饱和时自适应增益 scaling 冻结 | 任意方向饱和 |

### β₂ 自适应机制

```
基础: β₂_eff = β₂

自适应: 若误差同号持续 t 秒 → β₂_eff = β₂ × (1 + ceta2 × t³)
冻结: 若任意方向饱和 → β₂_eff 不再增长

ceta2 参数: Roll=500, Pitch/Yaw=5000
```

### 起飞保护
- **着陆时**: z1=ω_gyro, z2=0, z_inertia=0, 环形缓冲清零
- **起飞后 1s 内**: z2 强制清零, 仅跟踪 z1 (防止地效/重力补偿瞬态污染)

---

## 七、全参数说明

### 7.1 姿态外环参数

> 文件: `src/modules/mc_att_control/mc_att_control_params.c`

| 参数名 | 默认值 | 范围 | 单位 | 物理含义 |
|--------|--------|------|------|----------|
| `MC_ROLL_P` | **4.0** | 0–12 | — | Roll 姿态 P 增益 |
| `MC_PITCH_P` | **4.0** | 0–12 | — | Pitch 姿态 P 增益 |
| `MC_YAW_P` | **2.8** | 0–5 | — | Yaw 姿态 P 增益 |
| `MC_YAW_WEIGHT` | **0.4** | 0–1 | — | Yaw 优先级降权, 避免抢占 roll/pitch |
| `MC_ROLLRATE_MAX` | **220** | 0–1800 | deg/s | Roll 角速度上限 |
| `MC_PITCHRATE_MAX` | **220** | 0–1800 | deg/s | Pitch 角速度上限 |
| `MC_YAWRATE_MAX` | **200** | 0–1800 | deg/s | Yaw 角速度上限 |
| `MC_MAN_TILT_TAU` | **0.4** | 0–2 | s | 遥控倾斜输入低通时间常数 |

**数学关系**: `rate_sp = MC_ROLL_P × attitude_error`, `clamp(rate_sp, ±MC_ROLLRATE_MAX)`

**调参指导**:
- P 决定外环带宽 ≈ P/(2π) Hz, 当前 P=4.0 → ~0.6 Hz
- `MC_MAN_TILT_TAU=0.4` 将 RC 阶跃变为 ~0.4Hz 斜坡, 保护 TD

### 7.2 模式选择参数

| `RATE_CTRL_MODE` 值 | 含义 |
|---------------------|------|
| 0 | 全 PID (默认, ESO 热备但不输出) |
| 1 | Roll=ESO, Pitch/Yaw=PID |
| 2 | Pitch=ESO, Roll/Yaw=PID |
| 3 | Yaw=ESO, Roll/Pitch=PID |
| 4 | Roll+Pitch=ESO, Yaw=PID |
| 5 | 全 ESO |

ESO 和 PID 积分器在**所有模式下都运行**, 切换仅改变最终力矩来源。

### 7.3 ADRC 反馈增益 (闭环极点)

> 文件: `src/modules/mc_att_control/mc_rate_control_params.c`

| 参数名 | 默认值 | 轴 |
|--------|--------|-----|
| `RATE_FEED_R_P1` / `P2` | 3.0 / 3.0 | Roll |
| `RATE_FEED_P_P1` / `P2` | 3.0 / 3.0 | Pitch |
| `RATE_FEED_Y_P1` / `P2` | 3.0 / 3.0 | Yaw |

```
控制律: Ta1 = P1·P2·rate_err + (P1+P2)·accel_err
闭环: (s+P1)(s+P2) = 0, 极点: -P1, -P2
当前: P1=P2=3 → 临界阻尼, ω_c ≈ 3 rad/s
```

**核心约束**: ADRC 带宽 ≪ ESO 带宽 (至少 3× 以下), 当前 1:7 ✓

### 7.4 ESO 观测器增益

| 参数名 | 默认值 | 轴 | 含义 |
|--------|--------|-----|------|
| `RATE_ESO{R/P/Y}_BETA1` | 2.0 | 各轴 | z1 校正增益 β₁ |
| `RATE_ESO{R/P/Y}_BETA2` | 20.0 | 各轴 | z2 校正增益 β₂ |
| `RATE_ESO{R/P/Y}_CETA1` | 0.0 | 各轴 | β₁ 自适应 (禁用) |
| `RATE_ESOR_CETA2` | 500 | Roll | β₂ 自适应系数 |
| `RATE_ESO{P/Y}_CETA2` | 5000 | Pitch/Yaw | β₂ 自适应系数 |

**ESO 带宽**: β₁=2, β₂=20 → ~15-20 Hz

**调参指导**:
- β₂ 是**最关键参数**, 控制扰动跟踪速度
- 核心约束: ESO 带宽 ≥ 3× ADRC 带宽 (当前 40:1, 充裕)
- ceta2 越大 → 持续扰动时自适应越快; =0 则无自适应

### 7.5 执行器模型参数

| 参数名 | 默认值 | 单位 | 含义 |
|--------|--------|------|------|
| `RATE_ACT_T_{R/P/Y}` | 0.08 | s | 一阶惯性时间常数 T |
| `RATE_ACT_B_{R/P/Y}` | 8.0 | rad/s² | 控制增益 b |

```
执行器模型:  z_inertia' = (b × u_actual - z_inertia) / T
力矩输出:   torque = (z_inertia + T × Ta1) / b
```

**调参指导**:
- **b 过大是发散常见原因**: z_inertia 过快 → z2 反向补偿 → 正反馈爆炸
- 实际多旋翼: T ≈ 0.05–0.15s, b ≈ 5–15 rad/s²
- **建议从保守值开始** (T=0.08, b=8), 通过日志逐步辨识

### 7.6 TD 跟踪微分器参数

| 参数名 | 默认值 | 含义 |
|--------|--------|------|
| `RATE_TD_P1` | 3.0 | 欧拉角跟踪增益 (x1→angle) |
| `RATE_TD_P2` | 4.0 | 角速度跟踪增益 (x2→rate) |
| `RATE_TD_P3` | 5.0 | 角加速度跟踪增益 (x3→accel) |

```
TD 输入: Euler(q_setpoint) (纯参考)
TD 输出链路: Euler域 → W矩阵 → 机体系 → 20Hz LPF → 2s Ramp → 限幅
td_rate_sp    = clamp(ff_alpha × LPF(W × x2),    ±3.0 rad/s)
td_rate_accel = clamp(ff_alpha × LPF(W×x3+Ẇ×x2), ±10.0 rad/s²)
```

**调参**: P1 < P2 < P3 → 高阶导数跟踪更快。TD 只做前馈, 带宽不影响闭环稳定性。

### 7.7 TD 前馈增益 (硬编码)

> 位置: `src/lib/rate_control/rate_control.cpp`

| 常量 | 值 | 含义 |
|------|------|------|
| `k_ff_rate` | 0.15 | TD 角速度前馈增益 |
| `k_ff_acc` | 0.05 | TD 角加速度前馈增益 |

`torque_final = torque_adrc + k_ff_rate × td_x2 + k_ff_acc × td_x3`

### 7.8 ESO 内部硬编码常量

| 常量 | 值 | 位置 | 含义 |
|------|------|------|------|
| `ESO_HIS_BUF_LEN` | 16 | `eso_angular_rate.hpp` | 环形缓冲长度 (最大覆盖 40ms) |
| `Z2_LIMIT` | ±20.0 rad/s² | `eso_angular_rate.hpp` | z2 硬限幅 |
| 起飞冻结时间 | 1.0 s | `eso_angular_rate.cpp` | z2 起飞后 1s 强制清零 |

### 7.9 AttitudeControl 内部硬编码常量

| 常量 | 值 | 位置 | 含义 |
|------|------|------|------|
| LPF 截频 | 20 Hz | `AttitudeControl.cpp` | TD 输出低通 |
| Ramp 时间 | 2 s | `AttitudeControl.cpp` | 起飞前馈渐变 |
| td_rate_sp 限幅 | ±3.0 rad/s | `AttitudeControl.cpp` | 前馈角速度硬限 |
| td_rate_accel 限幅 | ±10.0 rad/s² | `AttitudeControl.cpp` | 前馈角加速度硬限 |
| ATT 分频阈值 | 10000 μs | `mc_att_control_main.cpp` | 100Hz 分频 |

### 7.10 全系统带宽层级总览

```
层级           带宽            参数来源                     约束
────────────────────────────────────────────────────────────────
ESO 观测器     ~15–20 Hz       β1=2, β2=20                 最快
Rate 内环      ~3 rad/s        P1=3, P2=3                  ≤ ESO/3
Attitude 外环  ~0.6 Hz         MC_ROLL_P=4                 ≤ Rate
TD 参考整形    ~0.5 Hz         TD_P1=3, P2=4, P3=5         ≈ Attitude
RC 输入滤波    ~0.4 Hz         MC_MAN_TILT_TAU=0.4         ≤ TD
────────────────────────────────────────────────────────────────
必须满足: ESO >> Rate >> Attitude ≈ TD ≈ RC input       ✓
```

---

## 八、文件清单

| 文件 | 职责 |
|------|------|
| `src/modules/mc_att_control/mc_att_control_main.cpp` | 主循环, 分频, 话题订阅/发布, 饱和反馈 |
| `src/modules/mc_att_control/mc_att_control.hpp` | 类定义, 参数声明, 成员变量 |
| `src/modules/mc_att_control/mc_att_control_params.c` | 姿态外环参数定义 |
| `src/modules/mc_att_control/mc_rate_control_params.c` | 角速度内环/ESO/TD 参数定义 |
| `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp/.cpp` | 姿态控制器 (TD + P控制 + W矩阵, 9步流程) |
| `src/modules/mc_att_control/AttitudeControl/AttitudeTd.hpp/.cpp` | 4阶跟踪微分器 |
| `src/modules/mc_att_control/AttitudeControl/AttitudeControlMath.hpp` | VTOL yaw 误差倾斜补偿 |
| `src/lib/rate_control/rate_control.hpp/.cpp` | 角速度控制器 (PID + ESO/ADRC 双路径, 模式选择) |
| `src/lib/rate_control/eso_angular_rate.hpp/.cpp` | ESO (环形缓冲, anti-windup, 自适应增益) |

---

## 九、已知问题 (2026-03-05 仿真)

### v1: ESO 立即发散
```
参数: P1=5, P2=3, β1=0.8, β2=4, T=0.07, b=20, z2_limit=50
      accel_err = -(z_inertia + z2)
现象: takeoff → z2 饱和 ±50 → zi 发散 → attitude failure
根因: ESO 带宽 ≈ 控制器带宽, b=20 过大导致 z_inertia 预测过快
```

### v2: z2 冻结期正常, 解冻后仍发散
```
参数: P1=3, P2=3, β1=2, β2=20, T=0.08, b=8, z2_limit=20
      accel_err = -z2 (去掉 z_inertia 二次补偿)
      起飞冻结 z2 1秒, k_ff_rate=0.15, k_ff_acc=0.05

地面: z2=0, zi=0 ✓ | 冻结期: z2=0 ✓ | 解冻后: z2 振荡发散 ✗
日志: z2=0.39→-0.60→0.50→-0.85→...→±17 → attitude failure

待排查:
  1. β₂=20 过高 → z2 校正过于激进产生振荡
  2. ceta2 自适应使 β₂_eff 解冻后迅速放大
  3. z_inertia 模型与仿真器执行器模型不匹配
  4. 延迟补偿 ring buffer 时间对齐不准确
```
