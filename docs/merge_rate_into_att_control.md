# 将角速度控制环（mc_rate_control）融合到姿态控制环（mc_att_control）

## 1. 背景与动机

### 1.1 原始架构

PX4 多旋翼控制系统采用**级联控制架构**，由外环到内环依次为：

```
位置控制 → 姿态控制 → 角速度控制 → 控制分配 → 电机输出
(mc_pos_control)  (mc_att_control)  (mc_rate_control)  (control_allocator)
```

其中姿态控制和角速度控制是两个**独立模块**，运行在不同的工作队列中：

| 模块 | 工作队列 | 触发源 | 频率 | 优先级 |
|------|---------|--------|------|--------|
| `mc_att_control` | `wq:nav_and_controllers` | `vehicle_attitude`（EKF输出） | ~250Hz | -13（低） |
| `mc_rate_control` | `wq:rate_ctrl` | `vehicle_angular_velocity`（陀螺仪） | ~1000Hz（硬件）/ ~250Hz（仿真） | 0（高） |

两个模块之间通过 uORB 话题 `vehicle_rates_setpoint` 进行通信：
- `mc_att_control` 发布 `vehicle_rates_setpoint`（角速度设定值）
- `mc_rate_control` 订阅 `vehicle_rates_setpoint` 并输出 `vehicle_torque_setpoint` + `vehicle_thrust_setpoint`

### 1.2 存在的问题

1. **控制频率不一致**：姿态环和角速度环运行在不同工作队列，由不同传感器数据触发，频率可能不同步
2. **uORB 通信延迟**：两个环之间通过 `vehicle_rates_setpoint` 话题通信，引入至少一个控制周期的延迟
3. **时序不确定性**：两个工作队列独立调度，无法保证姿态环输出的 rate setpoint 在下一个 rate control 周期被立即使用

### 1.3 合并目标

- **统一控制频率**：姿态控制和角速度控制在同一个 `Run()` 循环内执行，保证完全同步
- **消除通信延迟**：rate setpoint 不再经过 uORB，直接通过内部变量 `_rates_setpoint` 传递
- **保持功能不变**：所有控制算法（PID、ESO、TD）、参数、ACRO 模式等功能完全保留

---

## 2. 核心设计：如何实现控制频率一致

### 2.1 关键决策：选择触发源

合并后的模块需要选择一个传感器数据作为触发源。有两个选择：

| 方案 | 触发源 | 优点 | 缺点 |
|------|--------|------|------|
| A | `vehicle_attitude`（EKF） | 姿态数据直接可用 | 频率受限于 EKF 输出频率，角速度控制无法更快 |
| **B（采用）** | **`vehicle_angular_velocity`（陀螺仪）** | **频率最高，角速度控制以最快速率运行** | **姿态数据需要额外订阅，可能不是每次都有更新** |

**选择方案 B** 的原因：
- 角速度控制是最内环，对延迟最敏感，应以最高频率运行
- 姿态数据（EKF输出）更新频率 ≤ 陀螺仪频率，在 angular_velocity 触发时通过普通 `Subscription` 轮询即可获取最新值
- 这样 attitude P 控制器和 rate PID/ESO 控制器在**同一次 `Run()` 调用**中顺序执行

### 2.2 控制流程（合并后）

```
vehicle_angular_velocity 更新（陀螺仪数据）
         │
         ▼
    ┌─ Run() 被触发 ─────────────────────────────┐
    │                                              │
    │  1. 读取 angular_velocity（rates, angular_accel）│
    │  2. 计算 dt                                   │
    │  3. 更新通用订阅（control_mode, status 等）    │
    │                                              │
    │  ┌─── 姿态控制部分 ──────────────────────┐    │
    │  │ 4. 轮询 vehicle_attitude（有更新才执行）│    │
    │  │ 5. 姿态 P 控制 → rates_sp             │    │
    │  │ 6. _rates_setpoint = rates_sp  ←内部传递│    │
    │  │ 7. 发布 vehicle_rates_setpoint（日志用）│    │
    │  └───────────────────────────────────────┘    │
    │                                              │
    │  ┌─── ACRO 模式（无姿态控制时）──────────┐    │
    │  │ 8. 摇杆输入 → superexpo → rates_sp    │    │
    │  │ 9. _rates_setpoint = rates_sp          │    │
    │  └───────────────────────────────────────┘    │
    │                                              │
    │  ┌─── 角速度控制部分 ────────────────────┐    │
    │  │ 10. 读取 control_allocator 饱和状态    │    │
    │  │ 11. _rate_control.update(              │    │
    │  │       rates, _rates_setpoint,          │    │
    │  │       angular_accel, dt, landed)       │    │
    │  │ 12. 偏航扭矩低通滤波                   │    │
    │  │ 13. 电池电压补偿                       │    │
    │  │ 14. 发布 vehicle_torque_setpoint       │    │
    │  │ 15. 发布 vehicle_thrust_setpoint       │    │
    │  └───────────────────────────────────────┘    │
    │                                              │
    └──────────────────────────────────────────────┘
```

### 2.3 频率一致性保证

合并后，步骤 5（姿态控制）和步骤 11（角速度控制）在**同一次函数调用**中顺序执行：

```cpp
// 同一个 Run() 中：
// 1) 姿态控制（如果有新的 attitude 数据）
Vector3f rates_sp = _attitude_control.update(q);
_rates_setpoint = rates_sp;  // 内部变量直接传递，零延迟

// 2) 角速度控制（每次都执行）
Vector3f torque_setpoint = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, landed);
```

- 两个控制器共用同一个 `dt`
- rate setpoint 通过成员变量 `_rates_setpoint` 传递，**没有 uORB 延迟**
- 当 attitude 没有新数据时，角速度控制器使用上一次的 `_rates_setpoint` 继续运行

### 2.4 工作队列选择

合并后使用 `wq:rate_ctrl` 工作队列：

```cpp
WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
```

原因：
- `rate_ctrl` 优先级为 0（最高），保证控制环不被其他任务抢占
- `rate_ctrl` 栈空间更大（3150 bytes vs 2240 bytes），足以容纳合并后的代码
- 角速度控制对实时性要求最高，必须在高优先级队列中运行

---

## 3. 详细实现步骤

### 3.1 修改头文件 `mc_att_control.hpp`

#### 3.1.1 添加 include

新增角速度控制所需的头文件：

```cpp
#include <uORB/PublicationMulti.hpp>                  // 多实例发布
#include <uORB/topics/actuator_controls_status.h>     // 执行器控制状态
#include <uORB/topics/battery_status.h>               // 电池状态（电压补偿）
#include <uORB/topics/control_allocator_status.h>     // 控制分配器饱和状态
#include <uORB/topics/rate_ctrl_status.h>             // 角速度控制器状态
#include <uORB/topics/vehicle_angular_velocity.h>     // 角速度数据（新触发源）
#include <uORB/topics/vehicle_thrust_setpoint.h>      // 推力输出
#include <uORB/topics/vehicle_torque_setpoint.h>      // 扭矩输出
#include <lib/rate_control/rate_control.hpp>           // RateControl 库
#include <lib/systemlib/mavlink_log.h>                // MAVLink 日志
```

#### 3.1.2 更改触发源

```cpp
// 原来：由 vehicle_attitude 触发
uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

// 修改为：由 vehicle_angular_velocity 触发
uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

// vehicle_attitude 降级为普通订阅（轮询获取最新值）
uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
```

#### 3.1.3 添加发布者

```cpp
uORB::Publication<actuator_controls_status_s>   _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
uORB::PublicationMulti<rate_ctrl_status_s>       _controller_status_pub{ORB_ID(rate_ctrl_status)};
uORB::Publication<vehicle_torque_setpoint_s>     _vehicle_torque_setpoint_pub;
uORB::Publication<vehicle_thrust_setpoint_s>     _vehicle_thrust_setpoint_pub;
```

#### 3.1.4 添加 RateControl 对象

```cpp
RateControl _rate_control;  // 角速度 PID/ESO 控制器
```

#### 3.1.5 添加角速度控制相关成员变量

```cpp
matrix::Vector3f _acro_rate_max;         // ACRO 模式最大角速度
matrix::Vector3f _rates_setpoint{};      // 角速度设定值（内部传递，无需 uORB）
float _battery_status_scale{0.0f};       // 电池电压缩放因子
float _energy_integration_time{0.0f};    // 执行器能量积分时间
float _control_energy[4] {};             // 执行器控制能量
AlphaFilter<float> _output_lpf_yaw;      // 偏航扭矩低通滤波
int _last_rate_ctrl_mode{-1};            // 上次控制模式（用于检测模式切换）
bool _maybe_landed{true};               // 可能着陆标志
bool _rates_armed{false};               // 解锁状态
```

#### 3.1.6 添加所有角速度控制参数

DEFINE_PARAMETERS 宏中新增：

- **PID 增益**：`MC_ROLLRATE_P/I/D/FF/K`、`MC_PITCHRATE_P/I/D/FF/K`、`MC_YAWRATE_P/I/D/FF/K`
- **积分限幅**：`MC_RR_INT_LIM`、`MC_PR_INT_LIM`、`MC_YR_INT_LIM`
- **偏航滤波**：`MC_YAW_TQ_CUTOFF`
- **ACRO 参数**：`MC_ACRO_R_MAX`、`MC_ACRO_P_MAX`、`MC_ACRO_Y_MAX`、`MC_ACRO_EXPO`、`MC_ACRO_SUPEXPO` 等
- **电池补偿**：`MC_BAT_SCALE_EN`
- **ESO 观测器参数**：`RATE_ESOR_*`、`RATE_ESOP_*`、`RATE_ESOY_*`
- **执行器模型参数**：`RATE_ACT_T_*`、`RATE_ACT_B_*`
- **TD 跟踪微分器参数**：`RATE_TD_P1/P2/P3`
- **反馈增益**：`RATE_FEED_*`
- **控制模式**：`RATE_CTRL_MODE`

### 3.2 修改实现文件 `mc_att_control_main.cpp`

#### 3.2.1 构造函数

```cpp
MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),  // ← 改为 rate_ctrl 工作队列
    _vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
    _vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),  // ← 新增
    _vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),  // ← 新增
    ...
```

关键改动：
- 工作队列从 `nav_and_controllers` 改为 `rate_ctrl`
- 初始化 torque/thrust 发布者（VTOL 使用虚拟话题）

#### 3.2.2 init() — 注册回调

```cpp
bool MulticopterAttitudeControl::init()
{
    // 原来注册 vehicle_attitude 回调
    // 改为注册 vehicle_angular_velocity 回调
    if (!_vehicle_angular_velocity_sub.registerCallback()) {
        PX4_ERR("callback registration failed");
        return false;
    }
    return true;
}
```

#### 3.2.3 parameters_updated() — 扩展参数更新

在原有姿态控制参数设置之后，添加角速度控制参数设置：

```cpp
void MulticopterAttitudeControl::parameters_updated()
{
    // ---- 姿态控制参数（原有）----
    _attitude_control.setProportionalGain(...);
    _attitude_control.setRateLimit(...);

    // ---- 角速度控制参数（新增）----
    _rate_control.setPidGains(P, I, D);
    _rate_control.setEsoGains(beta1, beta2, ceta1, ceta2);
    _rate_control.setActGains(T, b);
    _rate_control.setTdGains(p1, p2, p3);
    _rate_control.setRateCtrlMode(mode);
    _rate_control.setIntegratorLimit(lim);
    _rate_control.setFeedForwardGain(ff);
    _rate_control.setFeedbackGains(fb1, fb2);
    _acro_rate_max = Vector3f(r, p, y);
    _output_lpf_yaw.setCutoffFreq(cutoff);
}
```

#### 3.2.4 Run() — 合并控制逻辑

Run() 函数的核心改动——三段式结构：

**第一段：姿态控制**（当有新的 attitude 数据时执行）
```cpp
vehicle_attitude_s v_att;
if (_vehicle_attitude_sub.update(&v_att)) {
    // 执行姿态 P 控制
    Vector3f rates_sp = _attitude_control.update(q);
    _rates_setpoint = rates_sp;  // 直接内部传递
}
```

**第二段：ACRO 模式**（当手动控制且无姿态控制时）
```cpp
if (flag_control_manual_enabled && !flag_control_attitude_enabled) {
    // 摇杆 → superexpo 映射 → 角速度设定值
    _rates_setpoint = man_rate_sp.emult(_acro_rate_max);
}
```

**第三段：角速度控制**（每次都执行）
```cpp
if (flag_control_rates_enabled) {
    Vector3f torque = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, landed);
    // 低通滤波、电池补偿、发布 torque/thrust
}
```

#### 3.2.5 新增 updateActuatorControlsStatus()

从 `mc_rate_control` 迁移过来的执行器控制能量统计函数，每 500ms 发布一次。

### 3.3 修改 CMakeLists.txt

```cmake
DEPENDS
    AttitudeControl
    circuit_breaker    # ← 新增
    mathlib
    RateControl        # ← 新增
    px4_work_queue
    StickYaw
```

### 3.4 迁移参数文件

| 操作 | 文件 | 说明 |
|------|------|------|
| **复制** | `mc_rate_control_params.c` → `mc_att_control/` | PID/ESO/TD/电池补偿参数定义 |
| **复制** | `mc_acro_params.c` → `mc_att_control/` | ACRO 模式参数定义 |
| **删除** | `mc_rate_control/mc_rate_control_params.c` | 避免全局参数重复定义 |
| **删除** | `mc_rate_control/mc_acro_params.c` | 避免全局参数重复定义 |

> **注意**：参数 `.c` 文件不需要加入 CMakeLists 的 SRCS 中。PX4 构建系统会自动扫描模块目录下的 `*_params.c` 文件来提取参数元数据。将其加入 SRCS 会导致编译错误（缺少 `PARAM_DEFINE_*` 宏定义）。

### 3.5 修改启动脚本

#### `rc.mc_apps`（多旋翼启动脚本）

```diff
-mc_rate_control start
-mc_att_control start
+# mc_rate_control is no longer needed — rate control is now inside mc_att_control.
+mc_att_control start
```

#### `rc.vtol_apps`（VTOL 启动脚本）

```diff
-mc_rate_control start vtol
 mc_att_control start vtol
+# mc_rate_control is no longer needed — rate control is now inside mc_att_control.
```

---

## 4. 验证结果

### 4.1 编译验证

```
make px4_sitl_default  →  成功，无错误
```

### 4.2 NSH 运行时验证

```
pxh> mc_rate_control status
INFO  [mc_rate_control] not running          ← mc_rate_control 未启动 ✅

pxh> mc_att_control status
INFO  [mc_att_control] running               ← mc_att_control 正常运行 ✅
```

### 4.3 工作队列验证

```
pxh> work_queue status
|__ 1) wq:rate_ctrl
|   |__ 7) mc_att_control   250.0 Hz   4000 us   ← 运行在 rate_ctrl 队列，250Hz ✅
```

`mc_att_control` 现在运行在 `wq:rate_ctrl`（优先级最高的工作队列），频率 250Hz，与 `vehicle_angular_velocity`（250Hz）完全同步。

### 4.4 话题输出验证

```
pxh> listener vehicle_torque_setpoint
    xyz: [-0.00071, 0.00045, -0.00038]      ← 扭矩输出正常 ✅

pxh> listener vehicle_thrust_setpoint
    xyz: [0.00000, 0.00000, -0.72814]       ← 推力输出正常 ✅

pxh> listener vehicle_rates_setpoint
    roll: 0.00477  pitch: -0.00056  yaw: 0.00124   ← 角速度设定值正常 ✅
```

### 4.5 飞行验证

仿真中成功解锁、起飞、悬停，控制行为正常。

---

## 5. 受影响的文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/modules/mc_att_control/mc_att_control.hpp` | **修改** | 添加 RateControl、订阅/发布、参数 |
| `src/modules/mc_att_control/mc_att_control_main.cpp` | **修改** | 合并 Run() 逻辑、改触发源和工作队列 |
| `src/modules/mc_att_control/CMakeLists.txt` | **修改** | 添加 RateControl、circuit_breaker 依赖 |
| `src/modules/mc_att_control/mc_rate_control_params.c` | **新增** | 从 mc_rate_control 迁移的参数定义 |
| `src/modules/mc_att_control/mc_acro_params.c` | **新增** | 从 mc_rate_control 迁移的 ACRO 参数定义 |
| `src/modules/mc_rate_control/mc_rate_control_params.c` | **删除** | 参数已迁移，避免重复 |
| `src/modules/mc_rate_control/mc_acro_params.c` | **删除** | 参数已迁移，避免重复 |
| `ROMFS/px4fmu_common/init.d/rc.mc_apps` | **修改** | 移除 `mc_rate_control start` |
| `ROMFS/px4fmu_common/init.d/rc.vtol_apps` | **修改** | 移除 `mc_rate_control start vtol` |

---

## 6. 注意事项

1. **`mc_rate_control` 模块未删除**：代码仍保留在仓库中，但不再被启动脚本调用。如需彻底清理，可后续删除该模块目录并从所有 `.px4board` 配置中移除。

2. **VTOL 支持**：合并后的 `mc_att_control` 在 VTOL 模式下使用虚拟话题（`vehicle_torque_setpoint_virtual_mc` / `vehicle_thrust_setpoint_virtual_mc`），与 `vtol_att_control` 模块协调工作。

3. **参数兼容性**：所有参数名称保持不变，已有的参数配置文件无需修改，用户无感知。

4. **日志兼容性**：`vehicle_rates_setpoint` 话题仍然被发布（用于日志记录和其他模块引用），`rate_ctrl_status` 也继续发布。
