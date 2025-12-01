# SCH16T 传感器电源循环修复方案

## 问题描述
传感器需要热插拔才能读取数据，说明传感器的初始化需要一次完整的电源循环。

## 根本原因
1. **初始化时序不足**: 原始的 `board_spi_reset(10, 0xffff)` 只断电 10ms
2. **SafeSPI 协议要求**: SCH16T 使用 SafeSPI 协议，可能需要更长的电源稳定时间
3. **多次初始化干扰**: 传感器经历了两次快速的初始化，导致内部状态机混乱

## 修复方案

### 当前电源时序

#### 系统启动时（init.c）
```c
// Line 329: 第一次上电
imxrt_spiinitialize();
// 执行：board_control_spi_sensors_power(true, 0xffff)

// Line 386: 第二次上电（复位）
board_spi_reset(10, 0xffff);
// 执行：
// - 断电所有 SPI 总线
// - 等待 10ms
// - 重新上电
```

#### 传感器启动时（rc.board_sensors）
```bash
# Line 32: SCH16T 专属电源循环
board_spi_reset 100 4
# 执行：
# - 断电 SPI3 总线 (VDD_3V3_SENSORS3_EN)
# - 等待 100ms
# - 重新上电

# Line 33: 等待传感器稳定
sleep 1

# Line 34: 启动 SCH16T 驱动
sch16t -R 0 -s -b 3 start
```

### 时序图

```
系统启动                    传感器启动
   |                           |
   v                           v
imxrt_spiinitialize()    board_spi_reset(100, 4)
   |                           |
   +---> 全部上电                +---> SPI3 断电 100ms
   |                           |
   v                           v
board_spi_reset(10, 0xffff)  sleep 1
   |                           |
   +---> 全部断电 10ms            +---> 等待稳定
   |                           |
   +---> 重新上电                 v
   |                      sch16t start
   |                           |
   v                           v
rc.board_sensors         传感器工作正常
```

### 关键参数

| 参数 | 值 | 说明 |
|------|---|------|
| 断电时间 | 100ms | 足够长的时间让传感器内部电容放电，完全复位 |
| 上电延迟 | 1s | 等待电源稳定和传感器内部初始化完成 |
| 总线掩码 | 4 (0x04) | 1 << (3-1)，只复位 SPI3 总线 |

## board_spi_reset 函数说明

### 函数原型
```c
void board_spi_reset(int ms, int bus_mask)
```

### 参数说明
- `ms`: 断电持续时间（毫秒）
- `bus_mask`: 总线选择掩码
  - Bit 0 (0x01): SPI1
  - Bit 1 (0x02): SPI2
  - Bit 2 (0x04): SPI3
  - Bit 3 (0x08): SPI4
  - ...
  - 0xFFFF: 所有总线

### 计算公式
```
bus_mask = 1 << (bus_number - 1)

示例：
SPI1: 1 << (1-1) = 1 << 0 = 0x01
SPI2: 1 << (2-1) = 1 << 1 = 0x02
SPI3: 1 << (3-1) = 1 << 2 = 0x04
SPI6: 1 << (6-1) = 1 << 5 = 0x20
```

### 函数执行流程
1. 禁用 SPI 总线上的所有设备（CS、DRDY 引脚设为 OFF）
2. 关闭电源（VDD_3V3_SENSORSx_EN = 0）
3. 等待指定时间（ms）
4. 重新配置引脚（CS、DRDY、SCK、MOSI、MISO）
5. 打开电源（VDD_3V3_SENSORSx_EN = 1）
6. 等待 100μs 让电源稳定

## 为什么热插拔能工作？

热插拔过程：
1. **拔出**: 传感器断电，内部电容完全放电（~100ms）
2. **插入**: 传感器重新上电，执行完整的上电初始化序列
3. **稳定**: 电源稳定后，传感器内部状态机正确初始化

我们的修复方案模拟了这个过程：
- 100ms 断电 → 等同于"拔出"
- 1s 上电延迟 → 等同于"插入后等待稳定"

## 验证方法

### 1. 编译和上传固件
```bash
make sky_v6x-rt upload
```

### 2. 监控启动日志
```bash
# 连接串口
screen /dev/ttyACM0 57600

# 或使用 minicom
minicom -D /dev/ttyACM0 -b 57600
```

### 3. 查看传感器日志
```bash
# 在 NuttX shell 中
dmesg | grep sch16t

# 应该看到:
# [rc.board_sensors] board_spi_reset 100 4
# [rc.board_sensors] sleep 1
# [sch16t] Resetting (soft)
# [sch16t] ASIC_ID: 0x21, COMP_ID: 0x23
# [sch16t] Sensor configuration validated
```

### 4. 检查传感器状态
```bash
sch16t status

# 应该显示:
# Running on SPI3
# State: READ
# Temperature: XX.X °C
# Sensor data OK
```

### 5. 监听数据流
```bash
listener sensor_accel -n 10
listener sensor_gyro -n 10

# 应该看到持续的数据更新
```

## 备选方案

如果 100ms 断电仍然不够，可以尝试：

### 方案 A: 增加断电时间
```bash
board_spi_reset 200 4    # 断电 200ms
sleep 2                  # 等待 2 秒
```

### 方案 B: 多次电源循环
```bash
board_spi_reset 100 4
sleep 1
board_spi_reset 100 4    # 第二次复位
sleep 1
sch16t -R 0 -s -b 3 start
```

### 方案 C: 移除 init.c 中的早期复位
如果传感器对多次复位敏感，可以移除 Line 329 的 `imxrt_spiinitialize()`，
只保留 Line 386 的 `board_spi_reset(10, 0xffff)`。

## 技术原理

### SCH16T 上电复位要求
根据 Murata SCH16T 数据手册：
- **上电时间**: VDD 从 0V 到 3.3V 需要 < 100ms
- **电源稳定时间**: 上电后需要等待 250ms 才能进行 SPI 通信
- **内部初始化**: 传感器内部需要初始化 MEMS、ADC、温度传感器等模块
- **状态机复位**: 不完整的电源循环可能导致状态机停留在中间状态

### SafeSPI 协议要求
- **CRC 初始化**: CRC8 计算器需要正确初始化
- **FIFO 清空**: 内部 FIFO 需要完全清空
- **寄存器复位**: 所有控制寄存器需要复位到默认值
- **时钟同步**: SPI 时钟域需要与内部时钟同步

### RT1176 电源管理
- **GPIO 切换时间**: < 10ns @ 1GHz
- **LDO 建立时间**: ~10ms（如果使用板载 LDO）
- **去耦电容放电**: 100ms 足以让 10μF 电容从 3.3V 放电到 < 0.5V

## 总结

✅ **已实施修复**:
- 在 SCH16T 启动前添加 100ms 电源循环
- 增加 1s 稳定时间
- 使用精确的总线掩码 (0x04)

⏳ **待验证**:
- 上传固件并测试
- 确认传感器无需热插拔即可工作
- 验证数据质量和稳定性

📝 **注意事项**:
- 如果问题仍然存在，考虑增加断电时间或上电延迟
- 监控系统日志，确保没有其他初始化错误
- 使用示波器验证电源波形（如有条件）
