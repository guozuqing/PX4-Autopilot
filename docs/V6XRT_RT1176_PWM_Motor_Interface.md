# V6XRT (i.MX RT1176) PWM电机控制接口文档

## 概述

PX4 FMU-V6XRT 基于 NXP i.MX RT1176 MCU，本文档详细分析了该芯片支持PWM和DShot的所有可用接口。

**关键信息：**
- **FlexPWM模块**: 4个 (FlexPWM1-4)，每个模块4个子模块 (SM0-SM3)
- **FlexIO1范围**: GPIO_EMC_B1_00 到 GPIO_EMC_B1_31 (共32个引脚)
- **DShot要求**: 引脚必须同时支持 FlexPWM 和 FlexIO1

---

## DShot硬件支持分析

### FlexIO1 范围限制

DShot协议需要通过 **FlexIO** 外设生成精确的数字时序信号。

| FlexIO模块 | 对应GPIO范围 | 引脚数量 |
|------------|--------------|----------|
| **FlexIO1** | GPIO_EMC_B1_00 ~ GPIO_EMC_B1_31 | 32个 |
| FlexIO2 | GPIO_AD_00 ~ GPIO_AD_31 | 32个 |

⚠️ **重要**: GPIO_EMC_B1_32 及以上的引脚 **不支持FlexIO1**，因此 **不能使用DShot**。

---

## 完整引脚映射表

### 同时支持 FlexPWM + FlexIO1 的引脚（可用于DShot）

| FlexPWM | 子模块 | 通道 | GPIO引脚 | FlexIO1 | DShot支持 |
|---------|--------|------|----------|---------|-----------|
| **FlexPWM1** | SM0 | A | GPIO_EMC_B1_23 | IO23 | ✅ |
| FlexPWM1 | SM0 | B | GPIO_EMC_B1_24 | IO24 | ✅ |
| FlexPWM1 | SM1 | A | GPIO_EMC_B1_25 | IO25 | ✅ |
| FlexPWM1 | SM1 | B | GPIO_EMC_B1_26 | IO26 | ✅ |
| FlexPWM1 | SM2 | A | GPIO_EMC_B1_27 | IO27 | ✅ |
| FlexPWM1 | SM2 | B | GPIO_EMC_B1_28 | IO28 | ✅ |
| FlexPWM1 | SM3 | A | GPIO_EMC_B1_38 | - | ❌ 超出范围 |
| FlexPWM1 | SM3 | B | GPIO_EMC_B1_39 | - | ❌ 超出范围 |
| **FlexPWM2** | SM0 | A | GPIO_EMC_B1_06 | IO06 | ✅ |
| FlexPWM2 | SM0 | B | GPIO_EMC_B1_07 | IO07 | ✅ |
| FlexPWM2 | SM1 | A | GPIO_EMC_B1_08 | IO08 | ✅ |
| FlexPWM2 | SM1 | B | GPIO_EMC_B1_09 | IO09 | ✅ |
| FlexPWM2 | SM2 | A | GPIO_EMC_B1_10 | IO10 | ✅ |
| FlexPWM2 | SM2 | B | GPIO_EMC_B1_11 | IO11 | ✅ |
| FlexPWM2 | SM3 | A | GPIO_EMC_B1_19 | IO19 | ✅ |
| FlexPWM2 | SM3 | B | GPIO_EMC_B1_20 | IO20 | ✅ |
| **FlexPWM3** | SM0 | A | GPIO_EMC_B1_29 | IO29 | ✅ |
| FlexPWM3 | SM0 | B | GPIO_EMC_B1_30 | IO30 | ✅ |
| FlexPWM3 | SM1 | A | GPIO_EMC_B1_31 | IO31 | ✅ |
| FlexPWM3 | SM1 | B | GPIO_EMC_B1_32 | - | ❌ 超出范围 |
| FlexPWM3 | SM2 | A | GPIO_EMC_B1_33 | - | ❌ 超出范围 |
| FlexPWM3 | SM2 | B | GPIO_EMC_B1_34 | - | ❌ 超出范围 |
| FlexPWM3 | SM3 | A | GPIO_EMC_B1_21 | IO21 | ✅ |
| FlexPWM3 | SM3 | B | GPIO_EMC_B1_22 | IO22 | ✅ |
| **FlexPWM4** | SM0 | A | GPIO_EMC_B1_00 | IO00 | ✅ |
| FlexPWM4 | SM0 | B | GPIO_EMC_B1_01 | IO01 | ✅ |
| FlexPWM4 | SM1 | A | GPIO_EMC_B1_02 | IO02 | ✅ |
| FlexPWM4 | SM1 | B | GPIO_EMC_B1_03 | IO03 | ✅ |
| FlexPWM4 | SM2 | A | GPIO_EMC_B1_04 | IO04 | ✅ |
| FlexPWM4 | SM2 | B | GPIO_EMC_B1_05 | IO05 | ✅ |
| FlexPWM4 | SM3 | A | GPIO_EMC_B1_17 | IO17 | ✅ |
| FlexPWM4 | SM3 | B | GPIO_EMC_B1_18 | IO18 | ✅ |

---

## DShot支持统计

| 类型 | 数量 | 说明 |
|------|------|------|
| **支持DShot的PWM_A通道** | 14个 | FlexPWM1(3) + FlexPWM2(4) + FlexPWM3(3) + FlexPWM4(4) |
| **支持DShot的PWM_B通道** | 12个 | 可作为备选 |
| **总计支持DShot** | 26个 | 同时具备FlexPWM和FlexIO1 |
| **不支持DShot** | 6个 | GPIO_EMC_B1_32及以上 |

---

## 16路全DShot配置方案

### 推荐配置 (14路PWM_A + 2路PWM_B)

| 通道 | FlexPWM | 子模块 | 类型 | GPIO | FlexIO | 状态 |
|------|---------|--------|------|------|--------|------|
| CH1 | FlexPWM1 | SM0 | A | GPIO_EMC_B1_23 | IO23 | ✅ DShot |
| CH2 | FlexPWM1 | SM1 | A | GPIO_EMC_B1_25 | IO25 | ✅ DShot |
| CH3 | FlexPWM1 | SM2 | A | GPIO_EMC_B1_27 | IO27 | ✅ DShot |
| CH4 | FlexPWM2 | SM0 | A | GPIO_EMC_B1_06 | IO06 | ✅ DShot |
| CH5 | FlexPWM2 | SM1 | A | GPIO_EMC_B1_08 | IO08 | ✅ DShot |
| CH6 | FlexPWM2 | SM2 | A | GPIO_EMC_B1_10 | IO10 | ✅ DShot |
| CH7 | FlexPWM2 | SM3 | A | GPIO_EMC_B1_19 | IO19 | ✅ DShot |
| CH8 | FlexPWM3 | SM0 | A | GPIO_EMC_B1_29 | IO29 | ✅ DShot |
| CH9 | FlexPWM3 | SM1 | A | GPIO_EMC_B1_31 | IO31 | ✅ DShot |
| CH10 | FlexPWM3 | SM3 | A | GPIO_EMC_B1_21 | IO21 | ✅ DShot |
| CH11 | FlexPWM4 | SM0 | A | GPIO_EMC_B1_00 | IO00 | ✅ DShot |
| CH12 | FlexPWM4 | SM1 | A | GPIO_EMC_B1_02 | IO02 | ✅ DShot |
| CH13 | FlexPWM4 | SM2 | A | GPIO_EMC_B1_04 | IO04 | ✅ DShot |
| CH14 | FlexPWM4 | SM3 | A | GPIO_EMC_B1_17 | IO17 | ✅ DShot |
| CH15 | FlexPWM1 | SM1 | B | GPIO_EMC_B1_26 | IO26 | ✅ DShot |
| CH16 | FlexPWM2 | SM3 | B | GPIO_EMC_B1_20 | IO20 | ✅ DShot |

### FlexPWM模块分配图

```
FlexPWM1 (4通道):
├── SM0_A → CH1  (GPIO_EMC_B1_23, FLEXIO23)
├── SM1_A → CH2  (GPIO_EMC_B1_25, FLEXIO25)
├── SM2_A → CH3  (GPIO_EMC_B1_27, FLEXIO27)
└── SM1_B → CH15 (GPIO_EMC_B1_26, FLEXIO26) [新增]

FlexPWM2 (5通道):
├── SM0_A → CH4  (GPIO_EMC_B1_06, FLEXIO06)
├── SM1_A → CH5  (GPIO_EMC_B1_08, FLEXIO08)
├── SM2_A → CH6  (GPIO_EMC_B1_10, FLEXIO10)
├── SM3_A → CH7  (GPIO_EMC_B1_19, FLEXIO19)
└── SM3_B → CH16 (GPIO_EMC_B1_20, FLEXIO20) [新增]

FlexPWM3 (3通道):
├── SM0_A → CH8  (GPIO_EMC_B1_29, FLEXIO29)
├── SM1_A → CH9  (GPIO_EMC_B1_31, FLEXIO31)
└── SM3_A → CH10 (GPIO_EMC_B1_21, FLEXIO21)

FlexPWM4 (4通道):
├── SM0_A → CH11 (GPIO_EMC_B1_00, FLEXIO00)
├── SM1_A → CH12 (GPIO_EMC_B1_02, FLEXIO02)
├── SM2_A → CH13 (GPIO_EMC_B1_04, FLEXIO04) [新增]
└── SM3_A → CH14 (GPIO_EMC_B1_17, FLEXIO17) [新增]
```

---

## 硬件设计注意事项

### 新增通道引脚冲突检查

| GPIO引脚 | 推荐用途 | 当前可能占用 | 需要处理 |
|----------|----------|--------------|----------|
| GPIO_EMC_B1_04 | CH13 PWM | NFC_GPIO | 需释放 |
| GPIO_EMC_B1_17 | CH14 PWM | nARMED | 需释放 |
| GPIO_EMC_B1_26 | CH15 PWM | 未占用 | ✅ 可用 |
| GPIO_EMC_B1_20 | CH16 PWM | 可能空闲 | 需确认 |

### 时钟配置

```c
// PWM时钟源: QTimer3 通过 XBAR 提供
// 总线时钟: BUS_CLK_ROOT_SYS_PLL3_CLK / 2 = 240 MHz
// PWM时钟: 240 MHz / 15 = 16 MHz
// 分频配置: COMP1 = 8, COMP2 = 7

imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM1_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);
imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM2_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);
imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM34_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);
```

---

## 支持的协议

### 1. 标准PWM
- **所有16个通道**均支持标准PWM输出
- 典型频率: 50Hz - 400Hz
- 脉宽范围: 1000μs - 2000μs

### 2. OneShot125
- **所有16个通道**支持OneShot125协议
- 更低延迟的电机控制

### 3. DShot (数字协议)
- **全部16个通道**支持DShot协议
- 通过 FlexIO1 实现
- 支持的DShot速率:

| 协议 | 比特率 | 帧时间 | 推荐用途 |
|------|--------|--------|----------|
| DShot150 | 150 kbit/s | 106.7μs | 长线缆 |
| DShot300 | 300 kbit/s | 53.3μs | 一般用途 |
| DShot600 | 600 kbit/s | 26.7μs | **推荐** |
| DShot1200 | 1200 kbit/s | 13.3μs | 短线缆 |

---

## 软件配置示例

### timer_config.cpp 配置结构

```cpp
// io_timers 数组 - 16个定时器配置
constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    initIOPWMDshot(PWM::FlexPWM1, PWM::Submodule0),  // CH1
    initIOPWMDshot(PWM::FlexPWM1, PWM::Submodule1),  // CH2
    initIOPWMDshot(PWM::FlexPWM1, PWM::Submodule2),  // CH3, CH15
    initIOPWMDshot(PWM::FlexPWM2, PWM::Submodule0),  // CH4
    initIOPWMDshot(PWM::FlexPWM2, PWM::Submodule1),  // CH5
    initIOPWMDshot(PWM::FlexPWM2, PWM::Submodule2),  // CH6
    initIOPWMDshot(PWM::FlexPWM2, PWM::Submodule3),  // CH7, CH16
    initIOPWMDshot(PWM::FlexPWM3, PWM::Submodule0),  // CH8
    initIOPWMDshot(PWM::FlexPWM3, PWM::Submodule1),  // CH9
    initIOPWMDshot(PWM::FlexPWM3, PWM::Submodule3),  // CH10
    initIOPWMDshot(PWM::FlexPWM4, PWM::Submodule0),  // CH11
    initIOPWMDshot(PWM::FlexPWM4, PWM::Submodule1),  // CH12
    initIOPWMDshot(PWM::FlexPWM4, PWM::Submodule2),  // CH13
    initIOPWMDshot(PWM::FlexPWM4, PWM::Submodule3),  // CH14
};

// timer_io_channels 数组 - 16个通道配置
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    /* CH1  */ initIOTimerChannelDshot(io_timers, {PWM::PWM1_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_B1_23, GPIO_FLEXIO1_FLEXIO23_1 | IOMUX_DSHOT_DEFAULT, 23),
    /* CH2  */ initIOTimerChannelDshot(io_timers, {PWM::PWM1_PWM_A, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_B1_25, GPIO_FLEXIO1_FLEXIO25_1 | IOMUX_DSHOT_DEFAULT, 25),
    /* CH3  */ initIOTimerChannelDshot(io_timers, {PWM::PWM1_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_B1_27, GPIO_FLEXIO1_FLEXIO27_1 | IOMUX_DSHOT_DEFAULT, 27),
    /* CH4  */ initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_B1_06, GPIO_FLEXIO1_FLEXIO06_1 | IOMUX_DSHOT_DEFAULT, 6),
    /* CH5  */ initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_B1_08, GPIO_FLEXIO1_FLEXIO08_1 | IOMUX_DSHOT_DEFAULT, 8),
    /* CH6  */ initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_B1_10, GPIO_FLEXIO1_FLEXIO10_1 | IOMUX_DSHOT_DEFAULT, 10),
    /* CH7  */ initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule3}, IOMUX::Pad::GPIO_EMC_B1_19, GPIO_FLEXIO1_FLEXIO19_1 | IOMUX_DSHOT_DEFAULT, 19),
    /* CH8  */ initIOTimerChannelDshot(io_timers, {PWM::PWM3_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_B1_29, GPIO_FLEXIO1_FLEXIO29_1 | IOMUX_DSHOT_DEFAULT, 29),
    /* CH9  */ initIOTimerChannelDshot(io_timers, {PWM::PWM3_PWM_A, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_B1_31, GPIO_FLEXIO1_FLEXIO31_1 | IOMUX_DSHOT_DEFAULT, 31),
    /* CH10 */ initIOTimerChannelDshot(io_timers, {PWM::PWM3_PWM_A, PWM::Submodule3}, IOMUX::Pad::GPIO_EMC_B1_21, GPIO_FLEXIO1_FLEXIO21_1 | IOMUX_DSHOT_DEFAULT, 21),
    /* CH11 */ initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_B1_00, GPIO_FLEXIO1_FLEXIO00_1 | IOMUX_DSHOT_DEFAULT, 0),
    /* CH12 */ initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_B1_02, GPIO_FLEXIO1_FLEXIO02_1 | IOMUX_DSHOT_DEFAULT, 2),
    /* CH13 */ initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_B1_04, GPIO_FLEXIO1_FLEXIO04_1 | IOMUX_DSHOT_DEFAULT, 4),
    /* CH14 */ initIOTimerChannelDshot(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule3}, IOMUX::Pad::GPIO_EMC_B1_17, GPIO_FLEXIO1_FLEXIO17_1 | IOMUX_DSHOT_DEFAULT, 17),
    /* CH15 */ initIOTimerChannelDshot(io_timers, {PWM::PWM1_PWM_B, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_B1_26, GPIO_FLEXIO1_FLEXIO26_1 | IOMUX_DSHOT_DEFAULT, 26),
    /* CH16 */ initIOTimerChannelDshot(io_timers, {PWM::PWM2_PWM_B, PWM::Submodule3}, IOMUX::Pad::GPIO_EMC_B1_20, GPIO_FLEXIO1_FLEXIO20_1 | IOMUX_DSHOT_DEFAULT, 20),
};
```

### board_config.h 配置

```c
#define DIRECT_PWM_OUTPUT_CHANNELS  16
#define BOARD_NUM_IO_TIMERS         16
```

---

## 使用建议

### 电机分配建议

| 机型 | 推荐通道 | 协议 |
|------|----------|------|
| 四旋翼 | CH1-CH4 | DShot600 |
| 六旋翼 | CH1-CH6 | DShot600 |
| 八旋翼 | CH1-CH8 | DShot600 |
| X8同轴 | CH1-CH8 | DShot600 |
| 十六旋翼 | CH1-CH16 | DShot600 |
| 固定翼+VTOL | CH1-CH8电机, CH9-CH16舵机 | DShot600 + PWM |

### 参数配置

```
# DShot配置
DSHOT_CONFIG = 600       # 推荐DShot600

# PWM频率配置
PWM_MAIN_RATE = 400      # 电机 400Hz
PWM_AUX_RATE = 50        # 舵机 50Hz
```

---

## 相关源文件

| 文件 | 描述 |
|------|------|
| `boards/sky/v6x-rt/src/timer_config.cpp` | PWM定时器和通道配置 |
| `boards/sky/v6x-rt/src/board_config.h` | 板级配置定义 |
| `platforms/nuttx/src/px4/nxp/rt117x/include/px4_arch/io_timer_hw_description.h` | FlexPWM硬件描述 |
| `platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/hardware/rt117x/imxrt117x_pinmux.h` | GPIO引脚复用定义 |

---

## 参考资料

- [PX4 PWM输出驱动文档](https://docs.px4.io/main/en/peripherals/pwm_escs_and_servo.html)
- [NXP i.MX RT1170参考手册](https://www.nxp.com/docs/en/reference-manual/IMXRT1170RM.pdf)
- [DShot协议说明](https://docs.px4.io/main/en/peripherals/dshot.html)
- [FlexIO用户指南](https://www.nxp.com/docs/en/application-note/AN12174.pdf)
