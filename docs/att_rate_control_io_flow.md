# 多旋翼姿态与角速度控制器 — 输入、输出、坐标系与运行流程

## 1. 系统总览

合并后的 `mc_att_control` 模块在一个控制循环中同时完成姿态控制（外环）和角速度控制（内环）。

```
                          ┌──────────────────────────────────────────────────┐
  位置控制器               │            mc_att_control (合并后)                │
  mc_pos_control           │                                                  │
       │                   │   ┌──────────────┐     ┌───────────────────┐     │
       │ attitude_setpoint │   │  姿态控制(P)  │────▶│  角速度控制(PID/ESO)│     │
       └──────────────────▶│   │  AttitudeControl  │     │  RateControl        │     │
                           │   └──────────────┘     └───────────────────┘     │
  手动遥控器               │         │                       │                │
  manual_control_setpoint  │   rates_setpoint          torque_setpoint       │
       └──────────────────▶│   (内部变量)               (发布输出)            │
                           │                                  │                │
                           └──────────────────────────────────┼────────────────┘
                                                              │
                                                              ▼
                                                      控制分配器
                                                   control_allocator
                                                              │
                                                              ▼
                                                         电机输出
```

---

## 2. 坐标系定义

本模块涉及两个坐标系：

### 2.1 世界坐标系（World Frame / NED）

- **定义**：北-东-地（North-East-Down）
- **X轴**：指向正北
- **Y轴**：指向正东
- **Z轴**：指向地心（重力方向）
- **用途**：姿态四元数 `q` 描述从世界系到机体系的旋转；偏航角（yaw）和偏航角速度设定值 `yawspeed_setpoint` 在此坐标系中定义

### 2.2 机体坐标系（Body Frame / FRD）

- **定义**：前-右-下（Forward-Right-Down）
- **X轴**：机头方向（前）
- **Y轴**：机体右侧
- **Z轴**：机体下方
- **用途**：所有角速度、角加速度、角速度设定值、扭矩输出、推力输出均在此坐标系中表达

```
              机体坐标系 (Body Frame, FRD)

                    X (Forward/Roll)
                    ↑
                    │
                    │
                    │
     Y ◀───────────┼ (机体重心)
     (Right/Pitch)  │
                    │
                    ▼
                    Z (Down/Yaw)
```

**旋转正方向**（右手定则）：
- **Roll（滚转）**：绕 X 轴，右侧机翼下压为正
- **Pitch（俯仰）**：绕 Y 轴，机头上仰为正
- **Yaw（偏航）**：绕 Z 轴，机头右偏为正

---

## 3. 姿态控制器（AttitudeControl）

### 3.1 算法

基于四元数的非线性姿态控制，参考 ETH Zurich 论文：
*"Nonlinear Quadrocopter Attitude Control"* (Brescianini, Hehn, D'Andrea, 2013)

核心思想：利用四元数误差的虚部作为姿态误差，乘以比例增益得到角速度设定值。

### 3.2 输入

| 输入 | 类型 | 坐标系 | 来源 | 说明 |
|------|------|--------|------|------|
| `q` | `Quatf` (四元数) | World→Body | `vehicle_attitude` (EKF) | 当前机体姿态估计，描述从世界系到机体系的旋转 |
| `q_d` | `Quatf` (四元数) | World→Body | `vehicle_attitude_setpoint` | 期望姿态，由位置控制器或手动模式生成 |
| `yawspeed_setpoint` | `float` [rad/s] | **世界系 Z 轴** | `vehicle_attitude_setpoint` | 偏航角速度前馈，绕世界系 Z 轴的旋转速率 |
| `thrust_body` | `Vector3f` [归一化] | **机体系** | `vehicle_attitude_setpoint` | 推力设定值，通常仅 Z 分量有值 `[0, 0, -thrust]` |

### 3.3 内部计算步骤

```
① 计算简化期望姿态 q_d_red（忽略偏航，优先保证 Roll/Pitch）
    → q_d_red = 从当前推力方向到期望推力方向的最短旋转

② 提取偏航分量 q_d_dyaw，按 yaw_weight 缩放
    → 偏航权重 < 1 时，允许偏航跟踪让步给 Roll/Pitch

③ 重组期望姿态
    → q_d = q_d_red × Quatf(cos(w·θ/2), 0, 0, sin(w·θ/2))

④ 计算姿态误差四元数
    → q_e = q⁻¹ × q_d

⑤ 提取误差向量（四元数虚部 × 2）
    → e = 2 × q_e.canonical().imag()     // 机体系下的姿态误差 [rad]

⑥ 比例控制
    → rate_sp = e ⊙ P_gain               // 逐元素相乘

⑦ 偏航角速度前馈（世界系→机体系转换）
    → rate_sp += q⁻¹.dcm_z() × yawspeed_setpoint
    // q⁻¹.dcm_z() 将世界系 Z 轴转换到机体系表达

⑧ 限幅
    → rate_sp = constrain(rate_sp, -rate_limit, +rate_limit)
```

### 3.4 输出

| 输出 | 类型 | 坐标系 | 单位 | 说明 |
|------|------|--------|------|------|
| `rate_setpoint` | `Vector3f` [roll, pitch, yaw] | **机体系** | rad/s | 期望角速度，传递给角速度控制器 |

> **关键**：输出的 `rate_setpoint` 通过内部成员变量 `_rates_setpoint` 直接传递给角速度控制器，不经过 uORB。

---

## 4. 角速度控制器（RateControl）

### 4.1 算法

支持两种模式：
- **PID 模式**：传统比例-积分-微分控制
- **ESO/PID 混合模式**：扩展状态观测器 + 跟踪微分器 + 单参数控制律

可通过 `RATE_CTRL_MODE` 参数逐轴切换。

### 4.2 输入

| 输入 | 类型 | 坐标系 | 来源 | 说明 |
|------|------|--------|------|------|
| `rate` | `Vector3f` [roll, pitch, yaw] | **机体系** | `vehicle_angular_velocity` (陀螺仪) | 当前实际角速度 [rad/s] |
| `rate_sp` | `Vector3f` [roll, pitch, yaw] | **机体系** | 姿态控制器输出 / ACRO 模式 | 期望角速度 [rad/s] |
| `angular_accel` | `Vector3f` [roll, pitch, yaw] | **机体系** | `vehicle_angular_velocity.xyz_derivative` | 角加速度估计 [rad/s²]，PID 的 D 项使用 |
| `dt` | `float` | — | 陀螺仪时间戳计算 | 控制周期时间步长 [s] |
| `landed` | `bool` | — | `vehicle_land_detected` | 着陆标志，着陆时重置 ESO/TD 并简化控制律 |
| 控制分配饱和状态 | `Vector3<bool>` × 2 | — | `control_allocator_status` | 正/负饱和标志，用于积分器抗饱和 |

### 4.3 内部计算步骤

#### PID 路径（所有模式下始终运行）

```
① 计算角速度误差
    → rate_error = rate_sp - rate                    // 机体系 [rad/s]

② PID 计算
    → torque_pid = Kp ⊙ rate_error                  // 比例项
                 + rate_int                           // 积分项
                 - Kd ⊙ angular_accel                // 微分项（用角加速度代替误差微分，减少噪声）
                 + Kff ⊙ rate_sp                     // 前馈项

③ 更新积分器（仅飞行中）
    → 饱和保护：输出饱和时停止对应方向积分
    → 非线性衰减：i_factor = max(0, 1 - (error/400°)²)
    → 硬限幅：constrain(-lim, +lim)
```

#### ESO 路径（ESO 模式下运行）

```
① ESO 闭环校正（每轴独立）
    → 用陀螺仪测量值校正上一周期的开环预测
    → 输出：z1（估计角速度）、z2（估计扰动加速度）、z_inertia（执行器惯性响应）

② TD 跟踪微分器 — 平滑期望信号
    → 输入：rate_sp（可能含阶跃变化）
    → 输出：x2（平滑期望角速度）、x3（期望角加速度）

③ 单参数控制律（三层递进）

    速度层：
    → Tv1 = P1 × (x2 - z1) + x3
      含义：角速度误差 × 反馈增益 + 角加速度前馈

    加速度层：
    → Tv2 = P2 × (x3 - α_ESO)
      含义：角加速度误差 × 反馈增益

    合成：
    → Ta1 = P2 × (Tv1 - α_ESO) + Tv2
      含义：叠加速度层和加速度层控制量

④ 执行器模型反解（角加速度→归一化力矩）
    → 飞行中：torque = (z_inertia + T × Ta1) / b
      · z_inertia：ESO 估计的执行器当前惯性响应（扰动补偿）
      · T：执行器一阶惯性时间常数 [s]
      · b：控制输入到角加速度的增益
    → 着陆时：torque = (T × Ta1) / b （去掉扰动补偿防振荡）

⑤ ESO 开环预测 — 为下一周期做准备
    → 将本周期力矩输出告知 ESO
    → ESO 据此预测下一周期角速度和扰动状态
```

#### 模式选择（逐轴）

```
⑥ 根据 RATE_CTRL_MODE 参数选择每轴输出

    模式 0：全 PID（安全回退）
    模式 1：Roll=ESO, Pitch=PID, Yaw=PID
    模式 2：Roll=PID, Pitch=ESO, Yaw=PID
    模式 3：Roll=PID, Pitch=PID, Yaw=ESO
    模式 4：Roll=ESO, Pitch=ESO, Yaw=PID
    模式 5：Roll=ESO, Pitch=ESO, Yaw=ESO
```

### 4.4 输出

| 输出 | 类型 | 坐标系 | 单位 | 说明 |
|------|------|--------|------|------|
| `torque_setpoint` | `Vector3f` [roll, pitch, yaw] | **机体系** | 归一化 [-1, 1] | 三轴扭矩指令，发布到 `vehicle_torque_setpoint` |
| `thrust_setpoint` | `Vector3f` [x, y, z] | **机体系** | 归一化 [-1, 1] | 推力指令（直接透传姿态控制器的推力设定值），发布到 `vehicle_thrust_setpoint` |

> 输出经过偏航低通滤波和电池电压补偿后发布。

---

## 5. 合并后完整运行流程

每当陀螺仪数据（`vehicle_angular_velocity`）更新时，`Run()` 被触发执行以下步骤：

```
                       陀螺仪数据更新
                            │
                            ▼
┌─────────────── Run() 开始 ──────────────────────────────────────┐
│                                                                  │
│  ① 读取 angular_velocity                                        │
│     · rates [roll, pitch, yaw]          机体系, rad/s            │
│     · angular_accel [roll, pitch, yaw]  机体系, rad/s²           │
│     · 计算 dt (基于陀螺仪时间戳)                                  │
│                                                                  │
│  ② 更新通用状态                                                   │
│     · vehicle_control_mode（控制模式标志）                         │
│     · vehicle_status（解锁状态、机型）                             │
│     · vehicle_land_detected（着陆状态）                            │
│     · hover_thrust_estimate（悬停推力估计）                        │
│     · vehicle_local_position（航向有效性）                         │
│                                                                  │
│  ③ 姿态控制（有新 attitude 数据时执行）                            │
│     ┌────────────────────────────────────────────────┐            │
│     │ 输入：                                          │            │
│     │   q (四元数)           World→Body, 来自 EKF     │            │
│     │   q_d (期望四元数)     World→Body, 来自位控/手动 │            │
│     │   yawspeed_setpoint   世界系 Z 轴, rad/s        │            │
│     │   thrust_body         机体系, 归一化             │            │
│     │                                                │            │
│     │ 计算：                                          │            │
│     │   四元数误差 → 比例控制 → 偏航前馈 → 限幅        │            │
│     │                                                │            │
│     │ 输出：                                          │            │
│     │   _rates_setpoint     机体系, rad/s             │            │
│     │   _thrust_setpoint_body  机体系, 归一化          │            │
│     └────────────────────────────────────────────────┘            │
│            │ 内部变量直接传递（零延迟）                              │
│            ▼                                                      │
│  ④ ACRO 模式（手动 + 无姿态控制时替代③）                           │
│     ┌────────────────────────────────────────────────┐            │
│     │ 输入：                                          │            │
│     │   manual_control_setpoint.roll/pitch/yaw [-1,1] │            │
│     │   manual_control_setpoint.throttle [-1,1]       │            │
│     │                                                │            │
│     │ 计算：                                          │            │
│     │   superexpo 映射 → × acro_rate_max             │            │
│     │                                                │            │
│     │ 输出：                                          │            │
│     │   _rates_setpoint     机体系, rad/s             │            │
│     │   _thrust_setpoint_body  机体系, 归一化          │            │
│     └────────────────────────────────────────────────┘            │
│            │                                                      │
│            ▼                                                      │
│  ⑤ 角速度控制（每次触发都执行）                                     │
│     ┌────────────────────────────────────────────────┐            │
│     │ 输入：                                          │            │
│     │   rates              机体系, rad/s (陀螺仪)     │            │
│     │   _rates_setpoint    机体系, rad/s (③或④的输出) │            │
│     │   angular_accel      机体系, rad/s² (陀螺仪导数)│            │
│     │   dt                 秒                         │            │
│     │   landed             着陆标志                    │            │
│     │                                                │            │
│     │ 计算：                                          │            │
│     │   PID: Kp·e + ∫Ki·e·dt - Kd·α̇ + Kff·rate_sp  │            │
│     │   ESO: 观测器校正 → TD平滑 → 单参数控制律       │            │
│     │        → 执行器反解                             │            │
│     │   模式选择: 按 RATE_CTRL_MODE 逐轴选择          │            │
│     │                                                │            │
│     │ 输出：                                          │            │
│     │   torque_setpoint    机体系, 归一化 [-1,1]      │            │
│     └────────────────────────────────────────────────┘            │
│            │                                                      │
│            ▼                                                      │
│  ⑥ 后处理                                                         │
│     · 偏航扭矩低通滤波（截止频率 MC_YAW_TQ_CUTOFF）               │
│     · 电池电压补偿（MC_BAT_SCALE_EN 启用时）                       │
│       thrust *= battery_scale, torque *= battery_scale            │
│                                                                  │
│  ⑦ 发布输出                                                       │
│     · vehicle_torque_setpoint  [roll, pitch, yaw]  机体系, [-1,1] │
│     · vehicle_thrust_setpoint  [x, y, z]           机体系, [-1,1] │
│     · vehicle_rates_setpoint   (供日志和其他模块)                   │
│     · rate_ctrl_status         (ESO/PID 内部状态日志)              │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
                            │
                            ▼
                     control_allocator
                     (扭矩/推力 → 各电机 PWM)
```

---

## 6. 数据流向汇总表

### 6.1 输入数据

| uORB 话题 | 关键字段 | 坐标系 | 单位 | 用途 |
|-----------|---------|--------|------|------|
| `vehicle_angular_velocity` | `xyz` | 机体系 | rad/s | 当前角速度（**触发源**） |
| `vehicle_angular_velocity` | `xyz_derivative` | 机体系 | rad/s² | 角加速度估计（PID D 项） |
| `vehicle_attitude` | `q[4]` | World→Body | 四元数 | 当前姿态估计 |
| `vehicle_attitude_setpoint` | `q_d[4]` | World→Body | 四元数 | 期望姿态 |
| `vehicle_attitude_setpoint` | `thrust_body[3]` | 机体系 | 归一化 | 推力设定值 |
| `vehicle_attitude_setpoint` | `yaw_sp_move_rate` | 世界系 Z 轴 | rad/s | 偏航角速度前馈 |
| `manual_control_setpoint` | `roll/pitch/yaw/throttle` | — | [-1, 1] | 遥控器杆量（手动模式） |
| `control_allocator_status` | `unallocated_torque[3]` | 机体系 | 归一化 | 饱和状态（积分器抗饱和） |
| `battery_status` | `scale` | — | 无量纲 | 电压补偿系数 |

### 6.2 输出数据

| uORB 话题 | 关键字段 | 坐标系 | 单位 | 接收者 |
|-----------|---------|--------|------|--------|
| `vehicle_torque_setpoint` | `xyz[3]` | **机体系** | 归一化 [-1, 1] | `control_allocator` |
| `vehicle_thrust_setpoint` | `xyz[3]` | **机体系** | 归一化 [-1, 1] | `control_allocator` |
| `vehicle_rates_setpoint` | `roll/pitch/yaw` | **机体系** | rad/s | 日志记录、其他模块参考 |
| `vehicle_attitude_setpoint` | `q_d[4]` | World→Body | 四元数 | 日志记录（手动模式下生成） |
| `rate_ctrl_status` | ESO/TD/PID 内部状态 | **机体系** | 各异 | 日志记录、调试 |
| `actuator_controls_status` | `control_power[3]` | — | W (功率) | 振动监测 |

### 6.3 坐标系总结

| 物理量 | 坐标系 | 说明 |
|--------|--------|------|
| 姿态 `q`, `q_d` | World→Body | 四元数描述旋转关系 |
| 偏航角速度前馈 `yawspeed_setpoint` | **世界系** Z 轴 | 在控制器内部转换到机体系 |
| 角速度 `rates`, `rate_sp` | **机体系** | [roll, pitch, yaw] 分别对应 Body [X, Y, Z] 轴 |
| 角加速度 `angular_accel` | **机体系** | 陀螺仪导数 |
| 扭矩输出 `torque_setpoint` | **机体系** | [roll, pitch, yaw] 绕 Body [X, Y, Z] 轴 |
| 推力输出 `thrust_setpoint` | **机体系** | 通常仅 Z 分量有值 `[0, 0, -T]`，向上为负 |
| 遥控器杆量 | 无坐标系 | 归一化 [-1, 1] 标量 |

---

## 7. 控制频率一致性

合并后的关键特性：**姿态控制和角速度控制在同一次 `Run()` 调用中顺序执行。**

```
时间线 (250 Hz, 每 4ms 一次):

  t=0ms     t=4ms     t=8ms     t=12ms
    │         │         │         │
    ▼         ▼         ▼         ▼
  ┌─Run()─┐ ┌─Run()─┐ ┌─Run()─┐ ┌─Run()─┐
  │att→rate│ │att→rate│ │att→rate│ │att→rate│
  └───────┘ └───────┘ └───────┘ └───────┘

  同一个 dt，同一次函数调用，零传递延迟
```

- **触发源**：`vehicle_angular_velocity`（陀螺仪输出，250Hz@仿真，硬件上可达 1kHz+）
- **工作队列**：`wq:rate_ctrl`（优先级 0，最高）
- **姿态数据**：通过普通 `Subscription` 轮询，有新数据时更新 `_rates_setpoint`，无新数据时沿用上一周期值
- **结果**：两个控制环共享同一个 `dt`，`_rates_setpoint` 通过成员变量直接传递，无 uORB 延迟
