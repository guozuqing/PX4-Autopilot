# Yaw 轴 ESO 振荡发散问题：根因分析与修复方案

> 本文档记录 Yaw 轴在单参数 ESO 控制模式下出现高频振荡发散的问题，
> 包含完整的根因分析、数学推导、修复方案和调参建议。

---

## 一、故障现象

在 `RATE_CTRL_MODE = 3`（Yaw 轴使用 ESO）或 `RATE_CTRL_MODE = 5`（全 ESO）模式下：

- **Yaw Rate Setpoint** 振荡到 ±200 deg/s
- **Yaw Rate Estimated**（ESO z1）振荡到 ±300 deg/s
- 频率很高，振幅越来越大，最终**发散**
- 参数改小后仍然不能恢复
- Roll/Pitch 轴正常

---

## 二、系统信号流回顾

先回顾正常的 ESO 单参数控制闭环信号流：

```
                    ┌─────────────┐
  rate_sp ──→ TD ──→│  控制律      │──→ torque ──→ 电机 ──→ 角速度(实际)
                    │ Tv1,Tv2,Ta1 │                          │
                    └─────┬───────┘                          │
                          │                                   │
                    ┌─────┴───────┐                          │
                    │    ESO      │←── 陀螺仪测量 ←──────────┘
                    │ z1,z2,      │
                    │ z_inertia   │
                    └─────────────┘
```

ESO 有两个阶段：
1. **闭环校正**（`run()`）：用陀螺仪测量值修正上次的开环预测
2. **开环预测**（`updateControlInput()`）：用本周期力矩输出预测下一周期状态

---

## 三、根因分析：模型不匹配

### 3.1 ESO 内部模型（开环预测）

`updateControlInput(u)` 中的数学模型：

```
ż_inertia = (b × u − z_inertia) / T      ... (1) 一阶惯性模型
ż₁ = z_inertia + z₂                       ... (2) 角速度预测
```

离散化后：
```cpp
_z_inertia += _dt * _inv_T * (_b * _u - _z_inertia);   // 式(1)
_z1 += _dt * (_z_inertia + _z2);                        // 式(2)
```

ESO 认为：「力矩指令 u 通过一阶惯性（时间常数 T，增益 b）产生角加速度 z_inertia，
再加上扰动 z₂，共同驱动角速度 z₁ 的变化。」

### 3.2 修复前的 Yaw 控制律输出

```cpp
// 修复前（错误）:
torque_yaw = Ta1_yaw / b_yaw;
```

### 3.3 Roll/Pitch 控制律输出（正确的）

```cpp
// Roll/Pitch（正确）:
torque_roll = (z_inertia_roll + T_roll × Ta1_roll) / b_roll;
```

### 3.4 不匹配在哪里？

写出 Roll 轴 ESO 看到的「等效角加速度」：

```
ESO 预测的角加速度 = z_inertia + z₂

ESO 模型中 z_inertia 的稳态值:
  当 ż_inertia = 0 时: z_inertia_ss = b × u

代入 Roll 的 torque 公式:
  u = (z_inertia + T × Ta1) / b

  b × u = z_inertia + T × Ta1

  所以 ESO 看到的惯性输入 = z_inertia + T × Ta1 ✓
  这和 ESO 模型期望的一致！
```

但 Yaw 轴：
```
  u = Ta1 / b

  b × u = Ta1

  ESO 看到的惯性输入 = Ta1
  ESO 模型期望的 = z_inertia + T × Ta1（含惯性项）

  差异 = z_inertia + T × Ta1 − Ta1 = z_inertia + (T−1) × Ta1 ✗
```

**ESO 模型以为力矩会产生 z_inertia 的动态响应，但实际的 torque 公式没有补偿惯性。**

### 3.5 发散机制推导

让我们追踪一个扰动如何被放大。假设某一刻 z₂ 有一个小的正偏差 δ：

**步骤①**：z₂ 偏大 → ESO 估计的角加速度 (z_inertia + z₂) 偏大

**步骤②**：控制律中 Tv2 = P2 × (td_x3 − (z_inertia + z₂))
→ Tv2 偏负（因为减去了偏大的 z₂）
→ Ta1 偏负 → torque 偏负

**步骤③**：偏负的 torque 送入 ESO 的 `updateControlInput()`
→ z_inertia 减小 → z₁ 的开环预测偏小

**步骤④**：下一个 `run()` 中：
```
err = v(测量) − his_z1[0](预测偏小) → err 偏大(正)
z2_err = err − last_err → 正
```
→ z₂ 被进一步增大！

**步骤⑤**：回到步骤①，形成正反馈环路。

**关键**：在 Roll/Pitch 中，`z_inertia / b` 项直接出现在 torque 中，
它本质上是一个**扰动前馈补偿**，可以部分抵消 z₂ 的误差传递。
Yaw 缺少这个补偿，z₂ 的任何偏差都 100% 传递到 torque，形成正反馈。

### 3.6 为什么参数改小也没用？

一旦发散开始：
1. z₂ 偏差 → 观测误差持续同方向
2. `_err_continues_time` 持续增大
3. `beta2_scale = 1 + ζ₂ × t³` 指数级增大（t³ 增长极快）
4. β₂ 有效值可达基准的 5 倍
5. z₂ 校正量 = 5 × β₂ × z2_err → 更大的 z₂ 偏差

**即使此时改小参数，z₂ 已经积累了很大的值**，并且历史队列 his_z1[0..7]
全部被错误值污染，ESO 需要很多个周期才能恢复——但在这期间正反馈继续，
导致「改小参数也没用」的现象。

### 3.7 为什么 Roll/Pitch 没有这个问题？

因为 Roll/Pitch 的 torque 公式是：

```
torque = (z_inertia + T × Ta1) / b
```

- `z_inertia / b` 项是扰动前馈，抵消了 z₂ 误差向 torque 的传递
- ESO 的内部模型与控制律输出**完全匹配**
- z₂ 偏差 → torque 变化 → ESO 预测准确 → 没有正反馈

---

## 四、修复方案（方案 C：三层防护）

### 4.1 C-1：统一模型（根治）

**原理**：让 Yaw 轴也使用与 Roll/Pitch 完全一致的控制律输出公式。

修改文件：`src/lib/rate_control/rate_control.cpp`

```cpp
// 修复前（Yaw 特殊处理，模型不匹配）:
torque_yaw = Ta1_yaw / b_yaw;

// 修复后（三轴统一，模型匹配）:
torque_yaw = (z_inertia_yaw + T_yaw × Ta1_yaw) / b_yaw;
```

**为什么这样能解决问题？**

代入 ESO 模型验证：
```
u = (z_inertia + T × Ta1) / b

ESO 看到的惯性输入 = b × u = z_inertia + T × Ta1

ESO 预测 ż_inertia = (b×u − z_inertia) / T = Ta1

这正是 ESO 期望看到的！模型匹配 → 无系统偏差 → z₂ 不会被错误驱动。
```

### 4.2 C-2：限幅保护（兜底安全）

即使模型匹配，极端情况（传感器故障、参数错误）下仍可能出问题。
添加两道安全网：

**① z₂ 扰动估计限幅**

修改文件：`src/lib/rate_control/eso_angular_rate.cpp` + `.hpp`

```cpp
// eso_angular_rate.hpp 中定义常量:
static constexpr float Z2_LIMIT = 50.0f;  // ±50 rad/s² ≈ ±2865 deg/s²

// eso_angular_rate.cpp run() 中，z2 更新后立即限幅:
_z2 += z2_correction;
_z2 = math::constrain(_z2, -Z2_LIMIT, Z2_LIMIT);
```

**为什么选 50 rad/s²？**
- 正常飞行中，外部扰动（风、振动）通常 < 10 rad/s²
- 极端扰动（碰撞、突风）可能到 20~30 rad/s²
- 50 rad/s² 留了充足余量，但能有效防止发散

**② torque 输出限幅**

修改文件：`src/lib/rate_control/rate_control.cpp`

```cpp
// ESO 力矩输出限制在 [-1, 1] 范围内
for (int i = 0; i < 3; i++) {
    torque_acfly_setpoint(i) = math::constrain(torque_acfly_setpoint(i), -1.0f, 1.0f);
}
```

**为什么选 ±1.0？**
- 控制分配器的输入范围就是 [-1, 1]
- 超出此范围的值没有物理意义
- 这是最后一道防线

### 4.3 C-3：自适应增益时间上限（防加速发散）

修改文件：`src/lib/rate_control/eso_angular_rate.cpp`

```cpp
// 修复前: _err_continues_time 无上限
_err_continues_time += dt;

// 修复后: 加 2 秒上限
_err_continues_time += dt;
_err_continues_time = math::min(_err_continues_time, 2.0f);
```

**为什么选 2 秒？**

`beta2_scale = 1 + ζ₂ × t³`

| 持续时间 t | t³    | ζ₂=1 时 scale |
|-----------|-------|---------------|
| 0.5s      | 0.125 | 1.125         |
| 1.0s      | 1.0   | 2.0           |
| 1.5s      | 3.375 | 4.375         |
| 2.0s      | 8.0   | 被 5.0 限幅    |
| 3.0s      | 27.0  | 被 5.0 限幅    |

- 2 秒时 t³ = 8，对于 ζ₂ = 0.5 的典型值：scale = 1 + 0.5 × 8 = 5.0，恰好触及上限
- 无需等到更长时间，2 秒已能充分激活自适应增益
- 超过 2 秒持续同向误差多半是模型问题，不应该继续放大增益

---

## 五、修改的文件汇总

| 文件 | 修改内容 |
|------|---------|
| `rate_control.cpp` L136-137 | Yaw 启用 z_inertia |
| `rate_control.cpp` L187 | Yaw 启用 T 参数 |
| `rate_control.cpp` L209-216 | Yaw 使用完整公式 `(z_inertia + T×Ta1)/b` |
| `rate_control.cpp` L219-223 | torque 输出 ±1.0 限幅 |
| `eso_angular_rate.cpp` L134-135 | 自适应时间上限 2s |
| `eso_angular_rate.cpp` L206-207 | z₂ 限幅 ±50 |
| `eso_angular_rate.hpp` L230-233 | Z2_LIMIT 常量定义 |

---

## 六、为什么当初 Yaw 轴不用惯性补偿？

原设计可能出于以下考虑：

1. **Yaw 轴的物理特性不同**：Yaw 力矩主要来自螺旋桨反扭矩的差分，
   而 Roll/Pitch 力矩来自推力差分。两者的执行器动态确实有差异。

2. **简化设计**：Yaw 轴的惯性参数（T_yaw, b_yaw）可能不容易辨识，
   去掉惯性项可以减少需要调的参数。

3. **忽略了模型一致性**：ESO 内部的 `updateControlInput()` 对三轴使用
   完全相同的一阶惯性模型，如果控制律输出端不匹配，就会产生系统偏差。

**结论**：即使 Yaw 轴的惯性特性不同，也应该保持控制律输出和 ESO 模型的一致性。
如果认为 Yaw 不需要惯性补偿，正确的做法是**同时修改 ESO 模型**
（例如设 T_yaw → 极小值让惯性环节退化），而不是只修改控制律输出端。

---

## 七、调参建议

修复后 Yaw 轴的 ESO 参数需要重新调整：

### 7.1 执行器参数

| 参数 | 建议初始值 | 说明 |
|------|-----------|------|
| `RATE_ACT_T_Y` | 0.02~0.05 | Yaw 执行器时间常数，从小值开始 |
| `RATE_ACT_B_Y` | 与 Roll/Pitch 同量级 | Yaw 力矩效率，需要实测标定 |

- **T_yaw 偏小**：ESO 认为执行器响应快，力矩更直接 → 接近原来的 `Ta1/b` 行为
- **T_yaw 偏大**：ESO 认为执行器响应慢，会加更多前馈补偿 → 更激进

### 7.2 观测器参数

| 参数 | 建议范围 | 说明 |
|------|---------|------|
| `RATE_ESOY_BETA1` | 与 Roll 轴一致或偏小 | 角速度校正增益 |
| `RATE_ESOY_BETA2` | 与 Roll 轴一致或偏小 | 扰动校正增益 |
| `RATE_ESOY_CETA2` | 0.2~1.0 | 自适应系数，从保守值开始 |

### 7.3 调参步骤

1. 先用 `RATE_CTRL_MODE = 0`（全 PID）确认飞机基本稳定
2. 切到 `RATE_CTRL_MODE = 3`（仅 Yaw 用 ESO），用保守参数飞行
3. 观察日志中 `eso_rate_yaw`、`eso_dis_yaw`、`eso_inertia_yaw` 的波形
4. 逐步增大 `RATE_ESOY_BETA2` 观察扰动跟踪速度
5. 确认 Yaw 稳定后，切到 `RATE_CTRL_MODE = 5`（全 ESO）

### 7.4 验证方法

检查以下指标确认修复有效：
- `eso_dis_yaw`（z₂）应该在合理范围内波动（< 10 rad/s²），不应该单调增长
- `eso_rate_yaw`（z₁）应该跟踪陀螺仪测量值，无发散趋势
- `rate_cmd_yaw`（torque）应该在 ±0.5 以内，不应该饱和到 ±1.0
- Yaw Rate Setpoint 和 Actual 之间的跟踪误差应该收敛

---

## 八、数学附录：闭环稳定性分析

### 8.1 简化线性模型

为了分析稳定性，将系统在工作点附近线性化。假设 TD 跟踪完美（x2 ≈ rate_sp，x3 ≈ 0），
扰动为零（z₂_true = 0），只分析 z₂ 估计偏差 δz₂ 的传递特性。

**控制律**（飞行中）：
```
torque = (z_inertia + T × Ta1) / b

其中:
  Tv1 = P1 × (x2 − z1) + x3
  Tv2 = P2 × (x3 − (z_inertia + z₂))
  Ta1 = P2 × (Tv1 − (z_inertia + z₂)) + Tv2
```

当系统稳态（rate = rate_sp，z₁ = rate，z_inertia 稳态）时，
引入一个 z₂ 偏差 δz₂，分析其对 torque 的影响：

```
δTv2 = P2 × (0 − δz₂) = −P2 × δz₂
δTa1 = P2 × (0 − δz₂) + δTv2 = −P2 × δz₂ − P2 × δz₂ = −2P2 × δz₂
δtorque = T × δTa1 / b = −2P2 × T × δz₂ / b
```

### 8.2 修复前 Yaw（无惯性补偿）

```
δtorque = δTa1 / b = −2P2 × δz₂ / b       ... (A)
```

ESO 的开环预测中：
```
δ(ż₁) = δz_inertia + δz₂

δz_inertia 来自: ż_inertia = (b×u − z_inertia)/T

δ(ż_inertia) = b × δu / T − δz_inertia / T
             = b × (−2P2 × δz₂ / b) / T − δz_inertia / T
             = −2P2 × δz₂ / T − δz_inertia / T
```

稳态时 `δz_inertia_ss = −2P2 × δz₂`（T 倍衰减后收敛）

代入 ż₁：
```
δ(ż₁) = δz_inertia_ss + δz₂ = −2P2 × δz₂ + δz₂ = (1 − 2P2) × δz₂
```

**当 P2 > 0.5 时**，`1 − 2P2 < 0`，z₁ 的偏差与 z₂ 偏差反号。
这个 z₁ 偏差进入 8 步历史队列后，在下次 `run()` 中产生误差：

```
δerr = −δ(his_z1[0]) → 正号（因为 z₁ 偏负）
δz2_err = δerr − last_δerr
```

如果这个 z2_err 持续同号，z₂ 会被进一步增大 → **正反馈**。

### 8.3 修复后 Yaw（有惯性补偿）

```
δtorque = (δz_inertia + T × δTa1) / b
        = (δz_inertia − 2P2 × T × δz₂) / b      ... (B)
```

ESO 开环预测：
```
δ(ż_inertia) = (b × δtorque − δz_inertia) / T
             = (δz_inertia − 2P2×T×δz₂ − δz_inertia) / T
             = −2P2 × δz₂
```

稳态 δz_inertia_ss = −2P2 × T × δz₂

代入 ż₁：
```
δ(ż₁) = δz_inertia_ss + δz₂ = −2P2×T×δz₂ + δz₂ = (1 − 2P2×T) × δz₂
```

**由于 T 通常很小（0.02~0.05）**，`2P2 × T` 远小于 1，
所以 `1 − 2P2×T` 接近 1，δz₁ 与 δz₂ 同号且幅度接近。

这意味着 ESO 的 z₁ 预测偏差方向**与 z₂ 偏差一致**，
闭环校正时 `err = v − his_z1[0]` 会产生**负反馈**，
将 z₂ 拉回正确值，**系统稳定**。

### 8.4 结论对比

| 配置 | δ(ż₁) / δz₂ | 闭环特性 |
|------|-------------|---------|
| 修复前 Yaw（无惯性补偿）| 1 − 2P2（P2>0.5 时为负）| **正反馈 → 发散** |
| 修复后 Yaw（有惯性补偿）| 1 − 2P2T（接近 +1）| **负反馈 → 稳定** |
| Roll/Pitch（一直有惯性补偿）| 1 − 2P2T（接近 +1）| **负反馈 → 稳定** |
