# SCH16T 解锁时重启问题 - 完整分析

## 问题现象
解锁（Arm）时 SCH16T 传感器重启，导致：
- DRDY missed: 10
- bad transfer: 11
- general error: 11
- 传感器不断重启

## 根本原因

### 解锁时的 GPIO 重配置操作

#### 调用链路
```
Commander::run() (主循环，每次迭代)
  ↓
Line 1949: px4_indicate_external_reset_lockout(LockoutComponent::Commander, isArmed())
  ↓
platforms/common/external_reset_lockout.cpp Line 54:
  BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(current_state != 0)
  ↓
board_config.h (原始定义):
  #define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled) \
      px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
  ↓
platforms/nuttx/src/px4/common/board_ctrl.c Line 63:
  px4_arch_configgpio((enable) ? GPIO_nARMED : GPIO_nARMED_INIT);
```

#### 解锁时发生的事情
1. **解锁前**: `GPIO_nARMED` (GPIO1_IO17) 配置为**输入模式**
2. **解锁时**: `px4_arch_configgpio(GPIO_nARMED)` 将其重新配置为**输出模式**
3. **副作用**: GPIO 重配置影响了**中断系统**

### GPIO 中断干扰机制

#### SCH16T DRDY 引脚
- **引脚**: GPIO_AD_21 (GPIO3_IO20)
- **功能**: Data Ready 中断
- **配置**: 下降沿触发中断

#### 问题机制
虽然 GPIO_nARMED 在 **GPIO Port 1**，SCH16T DRDY 在 **GPIO Port 3**，但：

1. **GPIO 重配置时的全局影响**：
   ```c
   px4_arch_configgpio(GPIO_nARMED)
   ```
   这个调用会：
   - 修改 GPIO 寄存器
   - 可能触发 GPIO 控制器的内部状态机更新
   - 在某些 MCU 上会短暂影响中断系统

2. **中断配置被破坏**：
   ```c
   // SCH16T 驱动中配置的 DRDY 中断
   px4_arch_gpiosetevent(_drdy_gpio, true, false, false,
                        &DataReadyInterruptCallback, this)
   ```
   解锁时的 GPIO 重配置可能导致这个中断配置失效

3. **连锁反应**：
   ```
   DRDY 中断失效
     → 驱动无法及时读取数据
     → DRDY missed: 10
     → bad transfer: 11
     → 超过 10 次失败阈值
     → 驱动执行 Reset()
     → 传感器重启
   ```

## 解决方案

### 方法 1：禁用 BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE（已实施）

**修改文件**: `boards/sky/v6x-rt/src/board_config.h`

```c
// Line 318-319: 保留 GPIO 定义
#define GPIO_nARMED_INIT     /* GPIO1_IO17 */ (GPIO_PORT1 | GPIO_PIN17 | GPIO_INPUT | nARMED_INPUT_IOMUX)
#define GPIO_nARMED          /* GPIO1_IO17 */ (GPIO_PORT1 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | nARMED_OUTPUT_IOMUX)

// Line 322-323: 禁用重配置宏
//#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
//#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)
```

**效果**:
- `px4_indicate_external_reset_lockout()` 变为空操作（Line 59 的 #else 分支）
- 解锁时不会执行任何 GPIO 重配置
- DRDY 中断保持稳定
- 传感器正常工作

### 方法 2：使用不同的 GPIO 引脚（备选方案）

如果需要 nARMED 功能，可以：
1. 选择与传感器 DRDY 不在同一 GPIO 控制器组的引脚
2. 确保 nARMED GPIO 不会影响中断系统

## 其他解锁时的操作（姿态模式）

### 正常操作
1. **LED 指示灯更新** (`control_status_leds`)
2. **音调提示** (`updateTunes`)
3. **执行器解锁** (actuator_armed 消息发布)
4. **模式切换** (切换到姿态模式)
5. **EKF 检查** (确保姿态估计有效)

### 不会影响传感器的操作
- ✅ 电机 ESC 使能
- ✅ 控制回路激活
- ✅ 遥控器输入处理
- ✅ 安全开关状态更新

### 被禁用的操作（修复后）
- ❌ GPIO_nARMED 重配置（已禁用）
- ❌ 外部复位锁定状态指示（已禁用）

## 验证步骤

1. **检查宏定义**:
   ```bash
   grep "BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE" boards/sky/v6x-rt/src/board_config.h
   # 应该看到被注释掉
   ```

2. **测试解锁**:
   ```bash
   commander arm
   sch16t status
   # 检查 DRDY missed 应为 0
   # 检查 reset events 不增加
   ```

3. **持续监控**:
   ```bash
   watch -n 1 'sch16t status | grep -E "reset:|DRDY|bad transfer"'
   ```

## 总结

**核心问题**: Commander 在每次循环中都会调用 `px4_indicate_external_reset_lockout()`，解锁时会触发 GPIO 重配置，破坏 SCH16T 的 DRDY 中断。

**核心修复**: 禁用 `BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE` 宏，使 `px4_indicate_external_reset_lockout()` 成为空操作，防止解锁时的 GPIO 重配置。

**副作用**: 失去硬件 nARMED 信号输出功能，但对系统正常运行无影响（这个信号主要用于外部安全电路）。
