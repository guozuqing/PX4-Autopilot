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

## 三、外环: AttitudeControl (100 Hz)

### 文件
- `AttitudeControl.hpp` — 类定义
- `AttitudeControl.cpp` — `update()` 实现

### 数据流

```
                    ┌──────────────────┐
   q_d (setpoint)   │  Step 1: TD      │
   ─────────────────►  input = Euler(q_d) ← 纯参考信号,不含测量态
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
   │ q_smooth =   │  │  body_ω = W × x2           │
   │ Quat(x1)     │  │  body_α = W × x3 + Ẇ × x2 │
   └──────┬───────┘  └──────┬──────────┬───────────┘
          │                 │          │
          ▼                 ▼          ▼
   ┌──────────────┐  ┌─────────┐ ┌─────────┐
   │ Step 3-4:    │  │Step 7:  │ │Step 7:  │
   │ PX4 标准     │  │LPF 20Hz│ │LPF 20Hz│
   │ 四元数误差   │  └────┬────┘ └────┬────┘
   │              │       │           │
   │ e_q = 2·Im(  │  ┌────┴────┐ ┌───┴─────┐
   │   q⁻¹⊗q_s)  │  │Step 8:  │ │Step 8:  │
   │              │  │Ramp 2s  │ │Ramp 2s  │
   │ rate_sp =    │  └────┬────┘ └────┬────┘
   │   Kp ⊙ e_q  │       │           │
   └──────┬───────┘  ┌────┴────┐ ┌───┴──────┐
          │          │Step 9:  │ │Step 9:   │
          │          │Clamp    │ │Clamp     │
          │          │±3 rad/s │ │±10 rad/s²│
          │          └────┬────┘ └────┬─────┘
          │               │           │
          ▼               ▼           ▼
      rate_sp        td_rate_sp   td_rate_accel
     (P校正)         (前馈 ω)     (前馈 α)
          │               │           │
          └───────┬───────┴───────────┘
                  │
                  ▼  传递给 RateControl::update()
```

### TD 设计要点

| 属性 | 值 |
|------|------|
| **输入** | `Euler(q_d)` — 纯参考欧拉角, **不含测量态 q** |
| **输出 x1** | 平滑欧拉角 → 构造 `q_smooth` 用于姿态 P 控制 |
| **输出 x2** | 欧拉角速率 → W 矩阵转机体系 → LPF → Ramp → **td_rate_sp** |
| **输出 x3** | 欧拉角加速度 → (W×x3+Ẇ×x2) → LPF → Ramp → **td_rate_accel** |
| **关键原则** | TD **只做前馈**, **不进入误差项** |
| **保护措施** | 20Hz LPF + 2s 起飞 Ramp + 硬限幅 (±3, ±10) |
| **参数** | `RATE_TD_P1=3, P2=4, P3=5` → 带宽 ~3 Hz |
| **运行频率** | 100 Hz (分频) |

### W 矩阵 (Euler → Body)

```
W = [ 1      0       -sinθ     ]     body_ω = W × [φ̇, θ̇, ψ̇]ᵀ
    [ 0    cosφ    sinφ·cosθ   ]
    [ 0   -sinφ    cosφ·cosθ   ]
```

---

## 四、内环: RateControl (400 Hz)

### 文件
- `rate_control.hpp` — 类定义
- `rate_control.cpp` — `update()` 实现

### 双路径架构

```
               rate_sp          td_rate_sp    td_rate_accel
               (from att P)    (from TD)     (from TD)
                  │                │              │
    ┌─────────────┼────────────────┼──────────────┼─────────────────┐
    │             │                │              │                 │
    │    ┌────────▼────────┐  ┌───▼──────────────▼────────────┐    │
    │    │   PID 路径      │  │     ESO/ADRC 路径             │    │
    │    │                 │  │                                │    │
    │    │ e = rate_sp - ω │  │  ① ESO 闭环校正 (400Hz)       │    │
    │    │                 │  │     run(ω_gyro, dt, ts)       │    │
    │    │ torque_pid =    │  │     → z1 (est. ω)            │    │
    │    │   Kp·e          │  │     → z2 (est. disturbance)  │    │
    │    │ + Ki·∫e         │  │     → z_inertia              │    │
    │    │ - Kd·α_gyro     │  │                                │    │
    │    │ + Kff·rate_sp   │  │  ② 控制律 (TD 纯前馈)        │    │
    │    │                 │  │     rate_err  = rate_sp - z1  │    │
    │    │                 │  │     accel_err = -α            │    │
    │    │                 │  │     α = z_inertia + z2        │    │
    │    │                 │  │                                │    │
    │    │                 │  │     Ta1 = P1·P2·rate_err      │    │
    │    │                 │  │         + (P1+P2)·accel_err   │    │
    │    │                 │  │                                │    │
    │    │                 │  │     torque = (zi + T·Ta1) / b │    │
    │    │                 │  │                                │    │
    │    │                 │  │  ③ TD 前馈叠加                │    │
    │    │                 │  │     torque += 0.3·td_x2       │    │
    │    │                 │  │            + 0.1·td_x3        │    │
    │    │                 │  │                                │    │
    │    │                 │  │  ④ 限幅 [-1, 1]              │    │
    │    └────────┬────────┘  └───────────────┬────────────────┘    │
    │             │                           │                     │
    │             ▼                           ▼                     │
    │    ┌──────────────────────────────────────────────────┐       │
    │    │        模式选择器 (RATE_CTRL_MODE 参数)          │       │
    │    │                                                  │       │
    │    │  0: 全 PID                                      │       │
    │    │  1: Roll=ESO, Pitch/Yaw=PID                    │       │
    │    │  2: Pitch=ESO, Roll/Yaw=PID                    │       │
    │    │  3: Yaw=ESO, Roll/Pitch=PID                    │       │
    │    │  4: Roll+Pitch=ESO, Yaw=PID                    │       │
    │    │  5: 全 ESO                                      │       │
    │    └────────────────────┬─────────────────────────────┘       │
    │                         │                                     │
    │                         ▼                                     │
    │                   torque_setpoint                             │
    │                         │                                     │
    │    ┌────────────────────┼──────────────────────────────┐      │
    │    │  ⑦ PID 积分器更新 (所有模式下运行)              │      │
    │    │     三重保护: 饱和截断 + 非线性衰减 + 硬限幅     │      │
    │    └────────────────────┼──────────────────────────────┘      │
    │                         │                                     │
    │    ┌────────────────────┼──────────────────────────────┐      │
    │    │  ⑧ ESO 开环预测 (所有模式下运行,保持 ESO 热备) │      │
    │    │     updateControlInput(torque_actual, now_us)     │      │
    │    │     → z_inertia' = (b·u_actual - zi) / T         │      │
    │    │     → z1' = z1 + dt·(zi + z2)                    │      │
    │    │     → ring_buffer[head] = (now_us, z1)            │      │
    │    └────────────────────┼──────────────────────────────┘      │
    │                         │                                     │
    └─────────────────────────┼─────────────────────────────────────┘
                              │
                              ▼
                    vehicle_torque_setpoint → 控制分配器
```

### ADRC 控制律数学

**误差定义** (TD 不进入误差, 只做前馈):
```
rate_err  = rate_sp - z1          (角速度误差)
accel_err = -z2                   (仅扰动, 不含 z_inertia 避免二次补偿)
```

**状态反馈**:
```
Ta1 = P1·P2 · rate_err + (P1+P2) · accel_err

特征多项式: s² + (P1+P2)·s + P1·P2 = (s+P1)(s+P2)
闭环极点: -P1, -P2 (独立可调)
当前: P1=P2=3 → ω_c ≈ 3 rad/s ≈ 0.5 Hz
```

**力矩输出**:
```
torque_adrc = (z_inertia + T × Ta1) / b      ← z_inertia 仅在此处补偿一次
torque_final = torque_adrc + 0.15·td_x2 + 0.05·td_x3
               (限幅到 [-1, 1])
```

### TD 前馈系数

| 参数 | 值 | 作用 |
|------|------|------|
| `k_ff_rate` | 0.15 | 角速度前馈增益 (叠加到力矩) |
| `k_ff_acc` | 0.05 | 角加速度前馈增益 (叠加到力矩) |

---

## 五、ESO 扩展状态观测器

### 文件
- `eso_angular_rate.hpp` — 类定义
- `eso_angular_rate.cpp` — 实现

### 内部状态

| 状态 | 含义 | 单位 |
|------|------|------|
| `z1` | 角速度估计 | rad/s |
| `z2` | 外部扰动估计 | rad/s² |
| `z_inertia` | 执行器惯性响应 | rad/s² |

### 调用时序 (每个控制周期)

```
① run(ω_gyro, dt, landed, timestamp_sample)     ← 闭环校正
    │
    ├─ lookupZ1AtTimestamp(timestamp_sample)       查找时间对齐的 z1 预测
    ├─ err = ω_gyro - z1_at_sample                 观测误差
    ├─ 自适应增益 β₂ 调节                          基于误差持续性
    ├─ z2 校正 (含方向截断 anti-windup)
    └─ z1 校正

② 控制律计算 torque_setpoint                      ← 在 RateControl 中

③ updateControlInput(torque_actual, now_us)        ← 开环预测
    │
    ├─ z_inertia += dt/T · (b·u_actual - zi)       执行器模型
    ├─ z1 += dt · (zi + z2)                         角速度预测
    └─ ring_buffer[head] = (now_us, z1)             存入环形缓冲
```

### 延迟补偿: 时间戳环形缓冲

```
旧方案: 固定 8 步历史队列 (假设延迟 = 8 个控制周期)
新方案: 16 槽时间戳环形缓冲 (精确对齐 gyro timestamp_sample)

┌────────────────────────────────────────────────┐
│  Ring Buffer (ESO_HIS_BUF_LEN = 16)           │
│                                                │
│  [0] t=100us, z1=0.001                        │
│  [1] t=2600us, z1=0.003                       │
│  ...                                           │
│  [15] t=40000us, z1=0.012                     │
│                                                │
│  lookupZ1AtTimestamp(gyro_ts):                 │
│    → 找到 |t_buf - gyro_ts| 最小的 entry     │
│    → 返回该 entry 的 z1 作为 z1_at_sample     │
└────────────────────────────────────────────────┘
```

### Anti-Windup 三重机制

| 机制 | 描述 | 触发条件 |
|------|------|----------|
| **u_actual** | z_inertia 模型使用实际施加力矩 (非期望) | 始终 |
| **z2 方向截断** | +sat → z2 只能减; -sat → z2 只能增 | `sat_positive` / `sat_negative` |
| **β₂ 冻结** | 自适应增益 scaling 冻结 | 任意方向饱和 |

### 参数默认值 (2026-03-05)

| 参数 | Roll | Pitch | Yaw |
|------|------|-------|-----|
| `β₁` | 2.0 | 2.0 | 2.0 |
| `β₂` | 20.0 | 20.0 | 20.0 |
| `ceta1` | 0.0 | 0.0 | 0.0 |
| `ceta2` | 500 | 5000 | 5000 |
| `z2_limit` | ±20 rad/s² | ±20 rad/s² | ±20 rad/s² |
| `T` | 0.08 | 0.08 | 0.08 |
| `b` | 8.0 | 8.0 | 8.0 |
| 起飞冻结 | 1.0 s | 1.0 s | 1.0 s |

---

## 六、mc_att_control_main.cpp 主循环

### 分频机制

```cpp
// 400 Hz: 主循环触发
if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
    // ... dt 计算, 传感器读取 ...

    // 100 Hz: 姿态外环 (含 TD)
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

    // 400 Hz: 角速度内环 (含 ESO)
    if (flag_control_rates_enabled) {
        torque = _rate_control.update(rates, _rates_setpoint, angular_accel,
                                      dt, landed, td_rate_sp, td_rate_accel,
                                      angular_velocity.timestamp_sample);
    }
}
```

### 饱和反馈链路

```
control_allocator_status
    │
    ├─► sat_positive[3], sat_negative[3]
    │
    ├─► RateControl::setSaturationStatus()
    │       ├─► ESO::setSaturation(pos, neg)  → z2 方向截断 + β₂ 冻结
    │       └─► PID updateIntegral()           → 积分方向截断
    │
    └─► ESO::updateControlInput(torque_actual)  → 实际力矩更新模型
```

---

## 七、全参数说明

---

### 7.1 姿态外环参数 (`mc_att_control_params.c`)

> 文件: `src/modules/mc_att_control/mc_att_control_params.c`
> 所属模块: `Multicopter Attitude Control`

| 参数名 | 默认值 | 范围 | 单位 | 物理含义 |
|--------|--------|------|------|----------|
| `MC_ROLL_P` | **4.0** | 0–12 | — | Roll 姿态 P 增益。误差 1 rad 时输出 4 rad/s 角速度指令 |
| `MC_PITCH_P` | **4.0** | 0–12 | — | Pitch 姿态 P 增益，同上 |
| `MC_YAW_P` | **2.8** | 0–5 | — | Yaw 姿态 P 增益 |
| `MC_YAW_WEIGHT` | **0.4** | 0–1 | — | Yaw 优先级权重。多旋翼 yaw 控制力矩弱于 roll/pitch，降权避免抢占 |
| `MC_ROLLRATE_MAX` | **220** | 0–1800 | deg/s | Roll 角速度上限 |
| `MC_PITCHRATE_MAX` | **220** | 0–1800 | deg/s | Pitch 角速度上限 |
| `MC_YAWRATE_MAX` | **200** | 0–1800 | deg/s | Yaw 角速度上限 |
| `MC_MAN_TILT_TAU` | **0.4** | 0–2 | s | 遥控倾斜输入一阶低通滤波时间常数。0=关闭，0.4=平滑 RC 输入防止阶跃 |

**数学关系:**
```
rate_sp = MC_ROLL_P × attitude_error        (rad/s = 增益 × rad)
rate_sp = clamp(rate_sp, ±MC_ROLLRATE_MAX)
```

**调参指导:**
- `MC_ROLL_P` / `MC_PITCH_P` 决定外环带宽 ≈ P/(2π) Hz
- 当前 P=4.0 → 外环 ≈ 0.6 Hz (足够保守)
- `MC_MAN_TILT_TAU=0.4` 将 RC 阶跃输入变为 ~0.4Hz 斜坡，保护 TD 不被激发

---

### 7.2 模式选择参数

| 参数名 | 默认值 | 范围 | 说明 |
|--------|--------|------|------|
| `RATE_CTRL_MODE` | **0** | 0–5 | 控制器模式选择 |

| 值 | 含义 |
|----|------|
| 0 | 全 PID (默认, ESO 热备运行但不输出) |
| 1 | Roll=ESO, Pitch/Yaw=PID |
| 2 | Pitch=ESO, Roll/Yaw=PID |
| 3 | Yaw=ESO, Roll/Pitch=PID |
| 4 | Roll+Pitch=ESO, Yaw=PID |
| 5 | 全 ESO (Roll/Pitch/Yaw 均由 ADRC 控制) |

**说明:** ESO 在所有模式下都运行 (热备)。PID 积分器也在所有模式下更新。切换模式仅改变最终力矩输出的来源。

---

### 7.3 ADRC 反馈增益参数 (闭环极点)

> 文件: `src/modules/mc_att_control/mc_rate_control_params.c`

| 参数名 | 默认值 | 范围 | 对应轴 | 物理含义 |
|--------|--------|------|--------|----------|
| `RATE_FEED_R_P1` | **3.0** | 0.1–50 | Roll | Roll 闭环极点 P₁ |
| `RATE_FEED_R_P2` | **3.0** | 0.1–50 | Roll | Roll 闭环极点 P₂ |
| `RATE_FEED_P_P1` | **3.0** | 0.1–50 | Pitch | Pitch 闭环极点 P₁ |
| `RATE_FEED_P_P2` | **3.0** | 0.1–50 | Pitch | Pitch 闭环极点 P₂ |
| `RATE_FEED_Y_P1` | **3.0** | 0.1–50 | Yaw | Yaw 闭环极点 P₁ |
| `RATE_FEED_Y_P2` | **3.0** | 0.1–50 | Yaw | Yaw 闭环极点 P₂ |

**数学关系:**
```
控制律:
  rate_err  = rate_sp - z1
  accel_err = -z2                          ← 仅扰动，不含 z_inertia
  Ta1 = P1·P2 · rate_err + (P1+P2) · accel_err

闭环特征多项式:
  s² + (P1+P2)·s + P1·P2 = (s + P1)(s + P2) = 0

闭环极点:  -P1, -P2
等效带宽:  ω_c ≈ (P1+P2)/2 = 3 rad/s ≈ 0.5 Hz
```

**调参指导:**
- P1=P2=3 → 临界阻尼，带宽 ~0.5 Hz
- **核心约束:** ADRC 控制器带宽 ≪ ESO 观测器带宽 (至少 3× 以下)
- 当前: 控制器 ~3 rad/s vs ESO ~20 rad/s → 比值 ≈ 1:7 ✓
- 增大 P1/P2 → 控制更快更激进，但要求 ESO 更快
- P1≠P2 时为过阻尼 (两个不同实极点)，P1=P2 时最快无超调

---

### 7.4 ESO 观测器增益参数

| 参数名 | 默认值 | 范围 | 对应轴 | 物理含义 |
|--------|--------|------|--------|----------|
| `RATE_ESOR_BETA1` | **2.0** | 0.1–10 | Roll | z1 (角速度) 校正增益 β₁ |
| `RATE_ESOR_BETA2` | **20.0** | 1–50 | Roll | z2 (扰动) 校正增益 β₂ |
| `RATE_ESOR_CETA1` | **0.0** | 0–10000 | Roll | β₁ 自适应系数 (禁用=0) |
| `RATE_ESOR_CETA2` | **500** | 0–10000 | Roll | β₂ 自适应系数 |
| `RATE_ESOP_BETA1` | **2.0** | 0.1–10 | Pitch | (同 Roll) |
| `RATE_ESOP_BETA2` | **20.0** | 1–50 | Pitch | (同 Roll) |
| `RATE_ESOP_CETA1` | **0.0** | 0–10000 | Pitch | (禁用) |
| `RATE_ESOP_CETA2` | **5000** | 0–10000 | Pitch | β₂ 自适应系数 |
| `RATE_ESOY_BETA1` | **2.0** | 0.1–10 | Yaw | (同 Roll) |
| `RATE_ESOY_BETA2` | **20.0** | 1–50 | Yaw | (同 Roll) |
| `RATE_ESOY_CETA1` | **0.0** | 0–10000 | Yaw | (禁用) |
| `RATE_ESOY_CETA2` | **5000** | 0–10000 | Yaw | β₂ 自适应系数 |

**数学关系 (闭环校正 `run()`):**
```
err = ω_gyro - z1_at_sample              ← 时间对齐的观测误差

β₂ 自适应:
  若误差同号持续 → β₂_eff = β₂ × (1 + ceta2 × continuous_time)
  若饱和 → β₂_eff 冻结

z1 校正:  z1 += β₁ × err × dt
z2 校正:  z2 += β₂_eff × err × dt        ← 含方向截断 anti-windup
z2 硬限幅: z2 = clamp(z2, ±Z2_LIMIT)
```

**ESO 等效带宽:**
```
β₁ = 2, β₂ = 20 → ESO 带宽 ≈ 15–20 Hz
```

**调参指导:**
- β₁ 控制 z1 跟踪速度 (角速度估计)
- β₂ 控制 z2 跟踪速度 (扰动估计) — **这是最关键的参数**
- **核心约束:** ESO 带宽 ≥ 3× ADRC 控制器带宽
- 当前: ESO ~20 Hz vs ADRC ~0.5 Hz → 比值 ≈ 40:1 (充裕)
- ceta2 越大 → 扰动持续时自适应增益越快
- ceta2=0 → 无自适应，β₂ 恒定
- Roll/Pitch/Yaw 的 ceta2 不同: Roll=500, Pitch/Yaw=5000 (yaw 扰动变化更缓慢)

---

### 7.5 执行器模型参数

| 参数名 | 默认值 | 范围 | 单位 | 对应轴 | 物理含义 |
|--------|--------|------|------|--------|----------|
| `RATE_ACT_T_R` | **0.08** | 0.01–0.2 | s | Roll | 一阶惯性时间常数 T |
| `RATE_ACT_T_P` | **0.08** | 0.01–0.2 | s | Pitch | 一阶惯性时间常数 T |
| `RATE_ACT_T_Y` | **0.08** | 0.01–0.2 | s | Yaw | 一阶惯性时间常数 T |
| `RATE_ACT_B_R` | **8.0** | 1–250 | rad/s² | Roll | 控制增益 b |
| `RATE_ACT_B_P` | **8.0** | 1–250 | rad/s² | Pitch | 控制增益 b |
| `RATE_ACT_B_Y` | **8.0** | 1–250 | rad/s² | Yaw | 控制增益 b |

**数学关系 (执行器一阶模型):**
```
模型:     z_inertia' = (b × u_actual - z_inertia) / T
稳态:     z_inertia_ss = b × u

物理含义:
  T = 电机+螺旋桨 响应时间 (油门→转速→推力)
  b = 归一化力矩 → 角加速度 增益
  u_actual = 控制分配器实际输出 (已考虑饱和)
```

**力矩输出公式:**
```
torque = (z_inertia + T × Ta1) / b

z_inertia:  执行器当前惯性响应 (前馈补偿)
T × Ta1:    考虑执行器延迟的控制指令
÷ b:        角加速度 → 归一化力矩
```

**调参指导:**
- T 越大 → 模型认为电机越慢 → z_inertia 跟踪越慢
- b 越大 → 模型认为控制力越强 → z_inertia 预测值越大
- **b 过大是发散的常见原因**: z_inertia 预测过快 → ESO 认为加速度巨大 → z2 反向补偿 → 正反馈爆炸
- 实际多旋翼: T ≈ 0.05–0.15s, b ≈ 5–15 rad/s²
- **建议从保守值开始** (T=0.08, b=8), 通过飞行日志逐步辨识

---

### 7.6 TD 跟踪微分器参数

| 参数名 | 默认值 | 范围 | 物理含义 |
|--------|--------|------|----------|
| `RATE_TD_P1` | **3.0** | 1–50 | 欧拉角跟踪增益 (x1→angle) |
| `RATE_TD_P2` | **4.0** | 1–50 | 角速度跟踪增益 (x2→rate) |
| `RATE_TD_P3` | **5.0** | 1–50 | 角加速度跟踪增益 (x3→accel) |

**数学关系 (4阶 TD):**
```
TD 输入:  u = Euler(q_setpoint)           ← 纯参考信号
TD 输出:  x1 = 平滑角度 → 构造 q_smooth
          x2 = 平滑角速度 (Euler域)
          x3 = 平滑角加速度 (Euler域)

TD 运行频率: 100 Hz (分频)
TD 等效带宽: ≈ min(P1,P2,P3)/(2π) ≈ 0.5 Hz
```

**输出链路 (AttitudeControl.cpp):**
```
Euler 域 → W矩阵 → 机体系 → 20Hz LPF → 2s 起飞 Ramp → 硬限幅

td_rate_sp    = clamp(ff_alpha × LPF(W × x2),  ±3.0 rad/s)
td_rate_accel = clamp(ff_alpha × LPF(W×x3+Ẇ×x2), ±10.0 rad/s²)
```

**调参指导:**
- P1 < P2 < P3 → 高阶导数跟踪更快 (允许 jerk 更灵敏)
- 全部减小 → 更平滑但跟踪更慢
- TD 只做前馈, **不进入误差计算**, 所以 TD 带宽不影响闭环稳定性
- 硬限幅 ±3/±10 防止遥控阶跃输入产生巨大前馈

---

### 7.7 TD 前馈增益 (硬编码)

> 位置: `src/lib/rate_control/rate_control.cpp` 第 222–223 行

| 常量 | 值 | 物理含义 |
|------|------|----------|
| `k_ff_rate` | **0.15** | TD 角速度前馈增益 (叠加到力矩) |
| `k_ff_acc` | **0.05** | TD 角加速度前馈增益 (叠加到力矩) |

**数学关系:**
```
torque_final = torque_adrc + k_ff_rate × td_x2 + k_ff_acc × td_x3

典型量级:
  td_x2 ≈ ±0.5 rad/s  →  前馈力矩 ≈ ±0.075
  td_x3 ≈ ±2 rad/s²   →  前馈力矩 ≈ ±0.10
```

**调参指导:**
- 前馈加速参考跟踪, 减小 rate_err 暂态
- 过大 → 遥控输入产生 torque spike
- 过小 → 前馈效果不明显, ADRC 全靠反馈
- **建议范围:** k_ff_rate ∈ [0.05, 0.3], k_ff_acc ∈ [0.02, 0.15]

---

### 7.8 ESO 内部硬编码常量

> 位置: `src/lib/rate_control/eso_angular_rate.hpp`

| 常量 | 值 | 物理含义 |
|------|------|----------|
| `ESO_HIS_BUF_LEN` | **16** | 时间戳环形缓冲长度 (400Hz → 最大覆盖 40ms 延迟) |
| `Z2_LIMIT` | **20.0** | z2 扰动估计硬限幅 (±20 rad/s²) |
| 起飞冻结时间 | **1.0 s** | z2 在起飞后 1 秒内强制清零 |

**调参指导:**
- `Z2_LIMIT` 过大 → 起飞瞬间 z2 可能被大扰动拖走
- `Z2_LIMIT` 过小 → 正常飞行中扰动补偿能力不足
- **建议范围:** 10–30 rad/s² (依机型)
- 起飞冻结 1s 保护 ESO 免受地效/重力补偿瞬态污染

---

### 7.9 AttitudeControl 内部硬编码常量

> 位置: `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`

| 常量 | 值 | 物理含义 |
|------|------|----------|
| TD 输出 LPF 截频 | **20 Hz** | 机体系前馈 LPF (抑制 TD 高频噪声) |
| 前馈 Ramp 时间 | **2 s** | 起飞后前馈从 0 线性增到 100% |
| td_rate_sp 限幅 | **±3.0 rad/s** | 前馈角速度硬限 |
| td_rate_accel 限幅 | **±10.0 rad/s²** | 前馈角加速度硬限 |
| ATT 分频阈值 | **10000 μs** | AttitudeControl+TD 100Hz (400Hz 主循环 ÷ 4) |

---

### 7.10 全系统带宽层级总览

```
层级           带宽            参数来源                        约束
─────────────────────────────────────────────────────────────────────
ESO 观测器     ~15–20 Hz       β1=2, β2=20                    最快
Rate 内环      ~0.5 Hz         P1=3, P2=3                     ≤ ESO/3
Attitude 外环  ~0.6 Hz         MC_ROLL_P=4                    ≤ Rate
TD 参考整形    ~0.5 Hz         TD_P1=3, P2=4, P3=5            ≈ Attitude
RC 输入滤波    ~0.4 Hz         MC_MAN_TILT_TAU=0.4            ≤ TD
位置控制       ~50 Hz loop     (独立模块)                      最慢
─────────────────────────────────────────────────────────────────────

必须满足: ESO >> Rate >> Attitude ≈ TD ≈ RC input
当前:      20 Hz >> 0.5 Hz >> 0.6 Hz ≈ 0.5 Hz ≈ 0.4 Hz     ✓
```

---

## 八、文件清单

| 文件 | 职责 |
|------|------|
| `mc_att_control_main.cpp` | 主循环, 分频, 话题订阅/发布, 饱和反馈 |
| `mc_att_control.hpp` | 类定义, 参数声明, 成员变量 |
| `mc_att_control_params.c` | 姿态外环参数定义 |
| `mc_rate_control_params.c` | 角速度内环/ESO/TD 参数定义 |
| `AttitudeControl.hpp` | 姿态控制器类 (含 TD, P 控制, W 矩阵) |
| `AttitudeControl.cpp` | 姿态控制器实现 (9 步流程) |
| `AttitudeTd.hpp` | 4 阶跟踪微分器类 |
| `rate_control.hpp` | 角速度控制器类 (含 3×ESO, PID, 模式选择) |
| `rate_control.cpp` | 角速度控制器实现 (8 步流程) |
| `eso_angular_rate.hpp` | ESO 类 (含环形缓冲, anti-windup) |
| `eso_angular_rate.cpp` | ESO 实现 (闭环校正 + 开环预测) |

---

## 九、已知问题 (2026-03-05 仿真)

### 第一轮修改 (v1): ESO 立即发散
```
参数: P1=5, P2=3, β1=0.8, β2=4, T=0.07, b=20, z2_limit=50
      accel_err = -(z_inertia + z2)
现象: takeoff → z2 饱和 ±50 → zi 发散 → attitude failure
根因: ESO 带宽 ≈ 控制器带宽, b=20 过大导致 z_inertia 预测过快
```

### 第二轮修改 (v2): z2 冻结期正常, 解冻后仍发散
```
参数: P1=3, P2=3, β1=2, β2=20, T=0.08, b=8, z2_limit=20
      accel_err = -z2 (去掉 z_inertia 二次补偿)
      起飞冻结 z2 1秒, k_ff_rate=0.15, k_ff_acc=0.05

地面阶段: z2=0.00, zi=0.00, rerr~0.003 — 正常 ✓
冻结期 (takeoff +1s): z2=0.00, 力矩输出小 — 正常 ✓
解冻后 ~2s:
  z2 振荡 ±1 → ±4 → ±10 → ±17 → rerr > 1 → torque 饱和 → attitude failure

典型日志序列:
  z2=0.39 → -0.60 → 0.50 → -0.85 → 1.48 → -1.57 → ...→ ±17 → 爆炸

可能根因 (待排查):
  1. β₂=20 过高 → z2 校正过于激进产生振荡
  2. ceta2 自适应使 β₂_eff 在解冻后迅速放大
  3. z_inertia 模型与仿真器执行器模型不匹配
  4. 延迟补偿 ring buffer 时间对齐不准确
```
