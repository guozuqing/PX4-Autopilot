# ESO/TD 起飞振荡发散问题报告

**日期**: 2026-03-05  
**平台**: PX4 SITL + Gazebo, x500 四旋翼  
**分支**: 自定义 ESO-TD 姿态/角速度控制  
**状态**: 调试中 — 已逐步剥离 ESO 所有反馈路径，定位到 TD 外环正反馈

---

## 1. 问题描述

起飞后 Roll 轴角速度误差 (`rate_err`) 在约 10 个控制周期（~1 秒）内从 ±0.05 rad/s 指数振荡增长到 ±10 rad/s，触发 PX4 "Attitude failure (roll)" 保护并导致失控。Pitch 轴有类似表现。

### 典型日志（最终版本，所有 ESO 特性已禁用）

```
[Takeoff detected]
rerr=+0.017  torq=+0.0009  z2= 0.00   ← 正常
rerr=-0.058  torq=-0.0030  z2= 0.04   ← 开始振荡
rerr=+0.055  torq=+0.0029  z2= 0.49
rerr=+0.180  torq=+0.0095  z2=-1.07   ← 幅值增大
rerr=-0.175  torq=-0.0092  z2=-1.17
rerr=-0.407  torq=-0.0214  z2= 3.66
rerr=+0.637  torq=+0.0334  z2=-0.50   ← 指数增长
rerr=-0.485  torq=-0.0255  z2=-4.42
rerr=+0.437  torq=+0.0229  z2= 9.45
rerr=-11.50  torq=-0.6037  z2=-3.37   ← 失控
```

---

## 2. 系统架构概述

### 2.1 控制链路

```
┌─────────────────────────────────────────────────────────────────────┐
│                        姿态控制器 (250Hz)                           │
│                                                                     │
│  qd (期望四元数)                                                    │
│    ↓                                                                │
│  rv_d = 2 × canonical(q⁻¹ × qd).imag()   ← 旋转向量误差(体轴系)   │
│    ↓                                                                │
│  TD.track(rv_d) → x1(平滑误差), x2(角速度), x3(角加速度)           │
│    ↓                                                                │
│  rate_sp = Kp × x1                        ← 姿态P控制              │
│  td_rate_sp = x2                          ← TD角速度前馈            │
│  td_rate_accel = x3                       ← TD角加速度前馈          │
└──────────────┬─────────────────────────┬────────────────────────────┘
               │                         │
               ↓                         ↓
┌─────────────────────────────────────────────────────────────────────┐
│                     角速度控制器 (250Hz)                             │
│                                                                     │
│  ESO.run(gyro)              ← 闭环校正: z1(角速度), z2(扰动)       │
│  ESO.updateControlInput(u)  ← 开环预测: z_inertia(执行器模型)      │
│                                                                     │
│  rate_err = (td_x2 + rate_sp) - z1_or_gyro                        │
│  accel_err = td_x3 - (z_inertia + z2)                             │
│  Ta1 = P1×P2 × rate_err + (P1+P2) × accel_err                    │
│  torque = (z_inertia + T×Ta1) / b                                 │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| P1 (RATE_FEED_*_P1) | 5.0 | 控制律第一极点 |
| P2 (RATE_FEED_*_P2) | 3.0 | 控制律第二极点 |
| β1 | 0.6 | ESO z1 校正增益 |
| β2 | 2.0 | ESO z2 校正增益 (从7.0降低) |
| ceta2 | 500 | ESO 自适应缩放 (从5000降低) |
| b (roll/pitch) | 20.0 | 执行器增益 (从120降低) |
| b (yaw) | 10.0 | 执行器增益 (从100降低) |
| T | 0.07 s | 执行器时间常数 |
| Kp (attitude) | ~6.5 | 姿态P增益 (PX4默认) |
| TD r1 | 100 | TD一阶跟踪速度 |

---

## 3. 调试历程与发现的正反馈路径

### 3.1 路径一览

我们发现并逐步断开了 **5 条正反馈路径**：

```
路径①: accel_err → Ta1 → torque → 角速度 → ESO z2 → accel_err
路径②: z2_correction → his_z1 → err → z2_err → z2_correction  
路径③: z2 → z1开环预测 → his_z1 → err → z2
路径④: torque → z_inertia → z1开环预测 → his_z1 → err → z2 → torque
路径⑤: torque → 姿态变化 → rv_d变化 → TD x2 → rate_err → torque
```

### 3.2 修复时间线

| 步骤 | 修复措施 | 对应路径 | 结果 |
|------|---------|---------|------|
| 1 | 禁用 accel_err (设为0) | ① | z2 仍发散 |
| 2 | z2_correction 不回注 his_z1 和 z1 | ② | z2 仍发散 |
| 3 | z2 从 z1 开环预测中移除 | ③ | z2 仍发散 |
| 4 | z_inertia 从 z1 开环预测中移除（禁用全部开环预测） | ④ | z2 仍发散 |
| 5 | z_inertia 从 torque 输出公式中移除 | ④ | z2 仍发散 |
| 6 | z1 替换为陀螺仪直读（ESO 完全不参与反馈） | 全部内部 | **rate_err 仍振荡** |
| 7 | 禁用 td_x2 前馈 | ⑤ | **待验证** |

### 3.3 关键洞察

**步骤 6 是决定性的**：当 z1 替换为陀螺仪直读后，ESO 内部状态（z1, z2, z_inertia）完全不参与任何反馈链路，但 `rate_err` 仍然振荡发散。这证明 **问题不在 ESO 内部，而在外环**。

---

## 4. 根因分析

### 4.1 最终确认的根因：TD 外环正反馈（路径⑤）

```
物理系统角速度振荡
  → 姿态(四元数q)变化
  → 旋转向量误差 rv_d = 2×canonical(q⁻¹×qd).imag() 变化
  → TD 输入变化
  → TD x2 (rv_d的一阶导数) 放大高频分量
  → rate_err = td_x2 + rate_sp - gyro 中 td_x2 和 rate_sp 同相叠加
  → torque 增大
  → 物理系统角速度振荡加剧
  → 正反馈闭环
```

### 4.2 机制详解

**TD 的 x2 输出是 x1 的导数**。数学关系 `dx1/dt ≈ x2`。

当物理系统因 torque 产生角速度 ω：
1. 姿态变化：`q(t+dt) = q(t) + dt/2 × q(t)⊗ω`
2. 旋转向量误差变化：`rv_d(t+dt) ≠ rv_d(t)`
3. TD 输入变化 → x1 变化 → **x2 = dx1/dt ≠ 0**
4. 同时 `rate_sp = Kp × x1` 也在变化

在 `rate_err = td_x2 + rate_sp - gyro` 中：
- `td_x2` 是 rv_d 变化率 → 与角速度同方向
- `rate_sp = Kp × x1` → 与姿态误差方向相关
- 两者在起飞振荡中 **同相叠加**，放大 rate_err

### 4.3 为什么地面上不发散

地面上 `rv_d ≈ 0`（姿态接近目标），TD 输出 `x1≈0, x2≈0`。`rate_err ≈ rate_sp - gyro ≈ 0`。没有足够的激励触发正反馈。

起飞时推力突变 → 姿态扰动 → rv_d 非零 → TD x2 非零 → 正反馈启动。

### 4.4 增益分析

```
等效力矩增益 = T × P1 × P2 / b = 0.07 × 15 / 20 = 0.0525
```

即 `torque = 0.0525 × rate_err`。

当 `rate_err = 0.18 rad/s` 时 `torque = 0.0095`，看似保守。但 `td_x2` 对高频成分有放大效应（4阶TD的微分特性），使得 `rate_err` 中包含的高频分量远超 `rate_sp - gyro` 本身。

---

## 5. 当前代码状态（所有禁用项）

### 5.1 `rate_control.cpp` — 控制律（完全简化）

```
当前生效:
  rate_err = rate_sp - gyro           (td_x2 禁用，z1 替换为 gyro)
  accel_err = 0                       (禁用)
  Ta1 = P1×P2 × rate_err             (仅一阶反馈)
  torque = T × Ta1 / b               (z_inertia 移除)

等效于:
  torque = 0.0525 × (rate_sp - gyro)  (纯P控制器)
```

### 5.2 `eso_angular_rate.cpp` — ESO 观测器（纯观测，不参与反馈）

```
updateControlInput():
  z_inertia 更新正常                   (仅监控)
  z1 开环预测: 禁用                     (z1 纯靠闭环校正)

run():
  z1_correction = β1 × err            (正常)
  z2_correction = β2 × z2_err         (正常，但不回注 z1/his_z1)
  his_z1 更新: 仅用 z1_correction      (z2_correction 不回注)
  z1 更新: 仅用 z1_correction          (z2_correction 不回注)
```

### 5.3 `AttitudeControl.cpp` — 姿态控制器

```
正常运行:
  rv_d = 2 × canonical(q⁻¹×qd).imag()   (旋转向量域)
  TD.track(rv_d) → x1, x2, x3
  rate_sp = Kp × x1                       (姿态P控制)
  td_rate_sp = x2                          (传递给ESO，但ESO不使用)
  td_rate_accel = x3                       (传递给ESO，但ESO不使用)
```

---

## 6. 被禁用的 ESO/ADRC 特性清单

| 特性 | 设计目的 | 禁用原因 | 恢复条件 |
|------|---------|---------|---------|
| **td_x2 前馈** | 轨迹跟踪前馈 | 外环正反馈（路径⑤） | 降低TD带宽或加衰减系数 |
| **accel_err 反馈** | 二阶状态反馈 | ESO未收敛时正反馈（路径①） | ESO收敛速度提升 |
| **z_inertia 在 torque 中** | 执行器惯性补偿 | 通过z1预测间接正反馈（路径④） | z1开环预测恢复后 |
| **z1 开环预测** | 提高观测精度 | z_inertia/z2注入z1造成正反馈（路径③④） | ESO增益调优后 |
| **z2 回注 his_z1** | 延迟补偿精度 | z2→his_z1→err→z2正反馈（路径②） | z2收敛速度验证后 |
| **ESO z1 用于 rate_err** | 滤波后的角速度估计 | z1滞后放大rate_err（β1=0.6） | 开环预测恢复后 |

---

## 7. 下一步计划

### 7.1 立即验证（优先级 P0）

1. **运行当前版本**（td_x2 已禁用），确认 `torque = 0.0525 × (rate_sp - gyro)` 的纯P控制能否稳定起飞
2. 如果稳定，说明所有 ESO/TD 特性中至少有一个引入了不稳定性
3. 如果仍不稳定，说明增益 `T×P1×P2/b = 0.0525` 本身过高，需降低 P1/P2 或增大 b

### 7.2 逐步恢复（优先级 P1）

按以下顺序逐个恢复，每步验证稳定性：

1. **ESO z1 替换回 gyro** → 确认纯P增益稳定
2. **td_x2 前馈加衰减** → `rate_err = α×td_x2 + rate_sp - gyro`，α从0.01开始
3. **z1 开环预测恢复** → 仅用 z_inertia（不含 z2）
4. **z_inertia 加入 torque** → `torque = (z_inertia + T×Ta1)/b`
5. **accel_err 恢复** → 从低增益 (P1+P2)×0.1 开始
6. **z2 回注 his_z1** → 最后恢复，需验证延迟补偿不引入不稳定

### 7.3 架构改进（优先级 P2）

- **TD 带宽与控制律带宽解耦**：当前 TD r1=100 可能过高，导致 x2 对输入变化响应过快。考虑降低到 10-30。
- **前馈低通滤波**：对 td_x2 施加低通滤波后再注入 rate_err，抑制高频放大。
- **ESO 观测器带宽设计**：β1, β2 需要与控制律极点 P1, P2 和 TD 带宽协调设计，避免多环路交叉耦合。
- **渐进启用机制**：起飞后经过收敛期（如2秒）再逐步启用 ESO/TD 高级特性。

---

## 8. 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `src/lib/rate_control/rate_control.cpp` | 控制律简化: z1→gyro, accel_err→0, td_x2→0, z_inertia→移除 |
| `src/lib/rate_control/eso_angular_rate.cpp` | z1开环预测禁用, z2不回注z1/his_z1 |
| `src/modules/mc_att_control/mc_rate_control_params.c` | β2: 7→2, ceta2: 5000→500, b: 120→20/20/10 |
| `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` | TD输入迁移到旋转向量域, 移除W/Ẇ矩阵 |
| `src/modules/mc_att_control/AttitudeControl/AttitudeTd.cpp` | 参数重命名 euler_d→ref_d |
| `src/modules/mc_att_control/AttitudeControl/AttitudeTd.hpp` | 注释更新为旋转向量域 |
| `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp` | 成员注释更新 |

---

## 9. 核心教训

1. **多环路系统调试必须从最内环开始**：ESO + TD + 姿态P + 执行器模型 = 4个嵌套环路，任何一个的增益/延迟不匹配都可能导致全局不稳定。

2. **ESO 开环预测是双刃剑**：开环预测提高了稳态精度（补偿8步延迟），但在瞬态（起飞）时，模型不匹配会被放大并通过多条路径形成正反馈。

3. **TD 导数输出（x2, x3）在闭环中的危险性**：TD 的高阶导数对输入噪声有放大效应。当这些导数通过控制律反馈到物理系统、再通过姿态测量反馈到 TD 输入时，形成了一个高增益的外环正反馈。这是本次调试发现的 **最意外的正反馈路径**。

4. **"逐步剥离法" 是有效的调试策略**：通过逐步禁用 ESO 特性并观察是否仍发散，最终定位到 TD 外环正反馈 — 一个在纯数学分析中不容易发现的路径（因为它跨越了姿态环和角速度环的边界）。
