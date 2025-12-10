# SPI3 DMA 硬件故障修复说明文档

## 问题现象

启用 `CONFIG_IMXRT_LPSPI3_DMA=y` 后，系统出现以下问题：

### 阶段 1：系统不断重启崩溃
```
故障类型：Hard Fault（硬件故障）
故障位置：armv7-m/arm_memfault.c 第 101 行
运行任务：wq:SPI3
程序计数器：PC: 0xfffffffe（无效地址，返回地址已损坏）
链接寄存器：LR: 0x301ba925
```

### 阶段 2：系统不崩溃，但读不到数据
修复第一个 bug 后：
```
INFO  [sch16t] Probing SCH16T sensor...
ERROR [sch16t] Sensor not ready - all registers read as 0xFFFF
ERROR [sch16t] Power stabilization may be insufficient
```

## 问题根源分析

NuttX 的 iMXRT LPSPI DMA 驱动存在**两个连续的严重 bug**，导致 DMA 传输了错误的数据量。

### Bug 1：字节数计算错误

**问题代码位置**：`/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c` 第 1578 行

```c
// 错误代码：
nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;
```

**问题分析**：
- 对于 16 位 SPI 传输，应该是 `字数 × 2` 得到字节数
- 但代码写成了 `nwords << 2`，左移 2 位相当于**乘以 4**
- 这导致计算出的字节数是实际需要的 **2 倍**

**造成的后果**：
1. 驱动为 N 个 16 位字分配缓冲区（N × 2 字节）
2. DMA 被配置为传输 N × 4 字节
3. TX DMA 从缓冲区末尾继续读取，读到栈上的数据
4. RX DMA 往缓冲区末尾继续写入，覆盖栈上的数据
5. **函数返回地址被破坏** → PC 变成 0xfffffffe
6. 触发硬件故障，系统崩溃重启

### Bug 2：DMA 迭代次数配置错误

**问题代码位置**：`/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c` 第 1629、1643 行

```c
// 错误代码：
config.iter   = nbytes;   // nbytes 已经是 nwords × 2
config.nbytes = adjust;   // adjust = 2（对于 16 位传输）
// 实际传输量 = iter × nbytes = (nwords × 2) × 2 = nwords × 4 字节 ❌
```

**问题分析**：
iMXRT 的 eDMA 控制器工作原理：
- `config.iter`：主循环迭代次数（传输多少次）
- `config.nbytes`：每次迭代传输的字节数
- **实际传输量** = `iter × nbytes`

对于 16 位 SPI，正确配置应该是：
- `iter` = 字数（要传输多少个 16 位字）
- `nbytes` = 2（每个 16 位字 = 2 字节）
- 总传输量 = 字数 × 2 字节 ✓

但错误代码把 `iter` 设置为了 `nbytes`（已经是字数×2），导致：
- `iter` = nwords × 2
- `nbytes` = 2
- 总传输量 = (nwords × 2) × 2 = nwords × 4 字节 ❌

**造成的后果**：
虽然修复了 Bug 1 后不再崩溃，但 DMA 仍然传输双倍数据：
1. DMA 读写超出正确的内存范围
2. 读取的数据全是无效数据（0xFFFF）
3. SCH16T 传感器无法正常工作

## 修复方案

### 修复 1：纠正字节数计算

**文件**：`/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c`
**行号**：1578

```c
// 修复前（错误）：
nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;

// 修复后（正确）：
nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;
```

**说明**：
- 将左移 2 位（`<< 2`，乘以 4）改为左移 1 位（`<< 1`，乘以 2）
- 16 位传输：字节数 = 字数 × 2 ✓
- 这个修复解决了系统崩溃问题

### 修复 2：纠正 DMA 迭代次数

**文件**：`/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c`
**行号**：1629、1643

```c
// 修复前（错误）：
config.iter   = nbytes;   // RX DMA 配置
// ... 以及 ...
config.iter   = nbytes;   // TX DMA 配置

// 修复后（正确）：
config.iter   = nwords;   // RX DMA 配置
// ... 以及 ...
config.iter   = nwords;   // TX DMA 配置
```

**说明**：
- 将迭代次数从 `nbytes`（字节数）改为 `nwords`（字数）
- 配合 `config.nbytes = adjust`（adjust = 2 对于 16 位）
- 实际传输量 = nwords × 2 字节 ✓
- 这个修复解决了数据传输错误问题

### 修复 3：增加工作队列栈大小（安全措施）

**文件**：`/boards/sky/v6x-rt/nuttx-config/nsh/defconfig`
**行号**：261

```
修复前：
CONFIG_SCHED_HPWORKSTACKSIZE=1800

修复后：
CONFIG_SCHED_HPWORKSTACKSIZE=2400
```

**说明**：
- 将高优先级工作队列栈大小从 1800 字节增加到 2400 字节
- 为 SPI DMA 操作提供额外的安全边际
- 防止其他潜在的栈溢出问题

## 技术细节

### 16 位 SPI 传输的正确配置

SCH16T 传感器使用 16 位 SPI 通信，每次传输 1 个 16 位字（2 字节）。

**正确的 DMA 配置**：
```c
// 假设要传输 100 个 16 位字
nwords = 100;
adjust = 2;  // 16 位 = 2 字节

// DMA 配置
config.iter   = 100;   // 迭代 100 次
config.nbytes = 2;     // 每次传输 2 字节
config.ssize  = EDMA_16BIT;  // 源数据宽度 16 位
config.dsize  = EDMA_16BIT;  // 目标数据宽度 16 位

// 总传输量 = 100 × 2 = 200 字节 ✓
```

**错误的配置（修复前）**：
```c
// 第一步计算错误
nbytes = nwords << 2;  // 100 << 2 = 400 字节（错误！应该是 200）

// 第二步配置错误
config.iter   = 400;   // 使用了字节数作为迭代次数
config.nbytes = 2;     // 每次传输 2 字节

// 总传输量 = 400 × 2 = 800 字节 ❌（应该是 200 字节）
```

### DMA 缓冲区溢出原理

```
正常情况：
┌─────────────────────┐
│   TX Buffer (200B)  │ ← DMA 读取 200 字节
├─────────────────────┤
│   Stack Data        │ ← 安全
└─────────────────────┘

Bug 情况：
┌─────────────────────┐
│   TX Buffer (200B)  │ ← DMA 读取 200 字节
├─────────────────────┤
│   Stack Data        │ ← DMA 继续读取 600 字节！
│   (Return Address)  │ ← 返回地址被读取并作为数据发送
│   (Local Variables) │
└─────────────────────┘
                        ↓
                  导致栈数据泄露和接收端数据错误

对于 RX：
┌─────────────────────┐
│   RX Buffer (200B)  │ ← DMA 写入 200 字节
├─────────────────────┤
│   Stack Data        │ ← DMA 继续写入 600 字节！
│   (Return Address)  │ ← 返回地址被覆盖 → 0xfffffffe
│   (Local Variables) │
└─────────────────────┘
                        ↓
                  函数返回时跳转到无效地址 → 硬件故障
```

## 影响范围

这个 bug 影响所有使用以下配置的系统：
- **芯片**：iMXRT117x 系列（如 iMXRT1176）
- **功能**：启用了 LPSPI DMA（`CONFIG_IMXRT_LPSPI*_DMA=y`）
- **传输模式**：16 位 SPI 传输
- **受影响的板子**：所有基于 iMXRT 的飞控，如 FMU-v6XRT、Sky v6x-rt 等

## 验证测试

修复后的测试步骤：

1. **重新编译固件**：
   ```bash
   make sky_v6x-rt_default
   ```

2. **烧录固件**：
   ```bash
   make sky_v6x-rt_default upload
   ```

3. **验证结果**：
   - ✓ 系统正常启动，不再重启
   - ✓ SCH16T 传感器成功初始化
   - ✓ 能正确读取传感器数据
   - ✓ `hardfault_log` 中没有新的故障记录

4. **预期日志**：
   ```
   INFO  [sch16t] Probing SCH16T sensor...
   INFO  [sch16t] SCH16T sensor found
   INFO  [sch16t] Device ID: 0x16
   ```

## 其他受影响的 SPI 总线

虽然问题在 SPI3 上发现，但修复适用于所有 LPSPI 总线：
- LPSPI1（SPI1）- ICM45686 传感器
- LPSPI2（SPI2）- IIM42653 传感器
- LPSPI3（SPI3）- SCH16T 传感器
- LPSPI4（SPI4）
- LPSPI6（SPI6）- 外部 SPI

如果这些总线启用了 DMA 且使用 16 位传输，都会受到影响。

## 相关文件

- **驱动修复**：`/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c`
- **配置修改**：`/boards/sky/v6x-rt/nuttx-config/nsh/defconfig`
- **SPI 配置**：`/boards/sky/v6x-rt/src/spi.cpp`
- **传感器驱动**：`/src/drivers/imu/murata/sch16t/SCH16T.cpp`
- **板级初始化**：`/boards/sky/v6x-rt/src/init.c`
- **启动脚本**：`/boards/sky/v6x-rt/init/rc.board_sensors`

## 修复日期

- **问题发现**：2024-12-02
- **第一次修复**（Bug 1 - 字节计算）：2024-12-02
- **第二次修复**（Bug 2 - 迭代次数）：2024-12-03
- **验证通过**：2024-12-03

## 总结

这是一个典型的 DMA 配置错误导致的缓冲区溢出问题。由于 DMA 硬件会直接访问内存而不经过 CPU 检查，配置错误会导致：
1. 内存越界访问
2. 栈数据损坏
3. 系统崩溃或数据错误

修复的关键是理解 eDMA 控制器的工作原理：
- **迭代次数（iter）** × **每次字节数（nbytes）** = **总传输量**

对于 16 位传输，正确配置是：
- `iter` = 字数
- `nbytes` = 2
- 总传输量 = 字数 × 2 字节

经过两次修复，SPI3 DMA 现在可以正常工作，SCH16T 传感器能够稳定运行。
