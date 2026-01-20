# PX4 sky/v6x-rt 板卡 16通道PWM扩展修改文档

## 概述

本文档记录了将 sky/v6x-rt 板卡的 PWM 输出从 **12通道** 扩展到 **16通道** 的所有修改内容。

---

## 修改背景

PX4 默认配置支持最多 12 个电机输出。sky/v6x-rt 板卡硬件支持 16 路 PWM 输出，需要修改软件配置以启用全部 16 个通道。

---

## 修改文件清单

| 序号 | 文件路径 | 修改类型 |
|-----|---------|---------|
| 1 | `src/lib/mixer_module/output_functions.yaml` | 电机函数数量 |
| 2 | `msg/versioned/ActuatorMotors.msg` | 电机控制消息 |
| 3 | `msg/ActuatorTest.msg` | 执行器测试消息 |
| 4 | `src/modules/control_allocator/module.yaml` | 控制分配参数 |
| 5 | `boards/sky/v6x-rt/src/timer_config.cpp` | 头文件包含顺序 |

---

## 详细修改内容

### 1. output_functions.yaml

**文件路径:** `src/lib/mixer_module/output_functions.yaml`

**修改说明:** 定义输出功能函数的电机数量

**修改前:**
```yaml
Motor:
  start: 101
  count: 12
```

**修改后:**
```yaml
Motor:
  start: 101
  count: 16
```

**影响:**
- 生成 Motor1 ~ Motor16 的输出功能定义
- 参数 `PWM_AUX_FUNC1` ~ `PWM_AUX_FUNC16` 支持选择 Motor1 ~ Motor16

---

### 2. ActuatorMotors.msg (版本化消息)

**文件路径:** `msg/versioned/ActuatorMotors.msg`

**修改说明:** 定义电机控制消息的数组大小和常量

**修改前:**
```
# Normalised thrust setpoint for up to 12 motors.
...
uint8 NUM_CONTROLS = 12  #
float32[12] control
```

**修改后:**
```
# Normalised thrust setpoint for up to 16 motors.
...
uint8 NUM_CONTROLS = 16  #
float32[16] control
```

**影响:**
- `actuator_motors_s::NUM_CONTROLS` 值变为 16
- 控制分配器 `MAX_NUM_MOTORS` 自动更新为 16
- 控制数组可容纳 16 个电机的推力值

---

### 3. ActuatorTest.msg

**文件路径:** `msg/ActuatorTest.msg`

**修改说明:** 定义执行器测试的电机数量上限

**修改前:**
```
uint8 MAX_NUM_MOTORS  = 12
```

**修改后:**
```
uint8 MAX_NUM_MOTORS  = 16
```

**影响:**
- `actuator_test` 命令支持测试 Motor1 ~ Motor16
- 修复编译时的 `static_assert` 校验错误

---

### 4. control_allocator/module.yaml

**文件路径:** `src/modules/control_allocator/module.yaml`

**修改说明:** 控制分配器的电机参数配置

**修改前:**
```yaml
__max_num_mc_motors: &max_num_mc_motors 12
...
CA_R_REV:
    ...
    bit:
        0: Motor 1
        ...
        11: Motor 12
```

**修改后:**
```yaml
__max_num_mc_motors: &max_num_mc_motors 16
...
CA_R_REV:
    ...
    bit:
        0: Motor 1
        ...
        11: Motor 12
        12: Motor 13
        13: Motor 14
        14: Motor 15
        15: Motor 16
```

**影响:**
- `CA_R0_SLEW` ~ `CA_R15_SLEW` 参数自动生成（16个电机转速限制参数）
- `CA_R_REV` 参数支持配置 16 个电机的可逆转方向

---

### 5. timer_config.cpp (头文件顺序修复)

**文件路径:** `boards/sky/v6x-rt/src/timer_config.cpp`

**修改说明:** 修复头文件包含顺序，确保 `MAX_IO_TIMERS` 正确定义

**修改前:**
```cpp
#include <px4_arch/io_timer_hw_description.h>
#include <board_config.h>
```

**修改后:**
```cpp
#include <drivers/drv_pwm_output.h>
#include "board_config.h"

#include <px4_arch/io_timer_hw_description.h>
```

**影响:**
- `board_config.h` 中定义的 `BOARD_NUM_IO_TIMERS=14` 在 `io_timer_hw_description.h` 之前生效
- `MAX_IO_TIMERS` 正确设置为 14，支持全部 14 个定时器
- 所有 16 个 PWM 通道正确初始化

---

## 相关常量定义（无需修改，仅供参考）

以下常量已经正确定义，支持 16 通道配置：

| 常量 | 值 | 文件 |
|-----|---|-----|
| `DIRECT_PWM_OUTPUT_CHANNELS` | 16 | `boards/sky/v6x-rt/src/board_config.h` |
| `BOARD_NUM_IO_TIMERS` | 14 | `boards/sky/v6x-rt/src/board_config.h` |
| `PWM_OUTPUT_MAX_CHANNELS` | 16 | `src/drivers/drv_pwm_output.h` |
| `MAX_TIMER_IO_CHANNELS` | 16 | `platforms/nuttx/src/px4/nxp/imxrt/include/px4_arch/io_timer.h` |
| `NUM_ACTUATORS` | 16 | `src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp` |

---

## 编译与验证

### 编译命令
```bash
cd /home/gbb/PX4-Autopilot
make clean                      # 清理（修改消息定义后必须）
make sky_v6x-rt_default upload  # 编译并上传
```

### 验证命令 (NSH)

#### 1. 检查 PWM 驱动状态
```bash
pwm_out status
```
预期输出应显示 Channel 0-15 全部配置，min/max 值非零。

#### 2. 检查参数生成
```bash
param show PWM_AUX_FUNC*   # 应显示 FUNC1 ~ FUNC16
param show PWM_AUX_TIM*    # 应显示 TIM0 ~ TIM13
param show CA_R*_SLEW      # 应显示 CA_R0_SLEW ~ CA_R15_SLEW
```

#### 3. 测试单个电机输出
```bash
# 测试电机 1（需解锁或设置 CBRK_IO_SAFETY=22027）
actuator_test set -m 1 -v 0.1

# 测试电机 13-16
actuator_test set -m 13 -v 0.1
actuator_test set -m 14 -v 0.1
actuator_test set -m 15 -v 0.1
actuator_test set -m 16 -v 0.1

# 停止测试
actuator_test set -m 1 -v -1
```

#### 4. 配置通道功能
```bash
# 将通道 13 配置为 Motor13
param set PWM_AUX_FUNC13 113

# 将通道 16 配置为 Motor16
param set PWM_AUX_FUNC16 116

# 保存参数
param save
```

---

## 通道与电机映射表

sky/v6x-rt 板卡的 PWM 通道与 FlexPWM 模块映射关系：

| 通道 | FlexPWM | GPIO引脚 | 默认功能 |
|-----|---------|---------|---------|
| CH1 | PWM1_SM0_A | GPIO_EMC_B1_23 | Motor1 |
| CH2 | PWM1_SM1_A | GPIO_EMC_B1_25 | Motor2 |
| CH3 | PWM1_SM2_A | GPIO_EMC_B1_27 | Motor3 |
| CH4 | PWM2_SM0_A | GPIO_EMC_B1_06 | Motor4 |
| CH5 | PWM2_SM1_A | GPIO_EMC_B1_08 | Motor5 |
| CH6 | PWM2_SM2_A | GPIO_EMC_B1_10 | Motor6 |
| CH7 | PWM2_SM3_A | GPIO_EMC_B1_19 | Motor7 |
| CH8 | PWM3_SM0_A | GPIO_EMC_B1_29 | Motor8 |
| CH9 | PWM3_SM1_A | GPIO_EMC_B1_31 | Motor9 |
| CH10 | PWM3_SM3_A | GPIO_EMC_B1_21 | Motor10 |
| CH11 | PWM4_SM0_A | GPIO_EMC_B1_00 | Motor11 |
| CH12 | PWM4_SM1_A | GPIO_EMC_B1_02 | Motor12 |
| CH13 | PWM4_SM2_A | GPIO_EMC_B1_04 | Motor13 |
| CH14 | PWM4_SM3_A | GPIO_EMC_B1_17 | Motor14 |
| CH15 | PWM1_SM1_B | GPIO_EMC_B1_26 | Motor15 |
| CH16 | PWM2_SM3_B | GPIO_EMC_B1_20 | Motor16 |

---

## 注意事项

1. **消息定义修改后必须 `make clean`**
   - 修改 `.msg` 文件后，uORB 消息结构会改变
   - 必须清理并重新编译，否则运行时会出现内存错误

2. **参数兼容性**
   - 新增的 Motor13-16 功能值为 113-116
   - 旧固件保存的参数在新固件上可能需要重新配置

3. **硬件连接**
   - CH15 和 CH16 分别使用 PWM1_SM1_B 和 PWM2_SM3_B
   - 这两个通道与 CH2、CH7 共享定时器子模块，PWM频率相同

---

## 修改日期

**日期:** 2026年1月20日

**修改人:** Cascade AI Assistant

**固件版本:** sky_v6x-rt_default
