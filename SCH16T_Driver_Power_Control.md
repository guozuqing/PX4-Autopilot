# SCH16T 驱动内部电源控制方案

## 修改说明

将传感器电源循环控制从启动脚本移至驱动内部，让驱动自己管理传感器的初始化流程。

## 修改文件

### 1. src/drivers/imu/murata/sch16t/SCH16T.cpp

在 `SCH16T::init()` 函数中添加电源循环逻辑：

```cpp
int SCH16T::init()
{
    // Power cycle the sensor to ensure clean initialization
    // This simulates hot-plug behavior which is known to work
    extern void board_control_spi_sensors_power(bool enable_power, int bus_mask);

    PX4_INFO("Power cycling SPI3 sensor (100ms)");

    // Power off SPI3 (bus mask: 1 << (3-1) = 0x04)
    board_control_spi_sensors_power(false, 0x04);
    px4_usleep(100000);  // 100ms power-off

    // Power on SPI3
    board_control_spi_sensors_power(true, 0x04);
    px4_usleep(1000000); // 1s wait for sensor stabilization

    PX4_INFO("Power cycle complete, initializing sensor");

    int ret = SPI::init();
    // ... 后续初始化
}
```

### 2. boards/sky/v6x-rt/init/rc.board_sensors

简化传感器启动命令：

```bash
# SCH16T on SPI3 with DRDY (GPIO_AD_21 GPIO3_IO20)
# Note: Driver handles power cycling internally for proper initialization
sch16t -R 0 -s -b 3 start
```

## 技术细节

### 电源控制函数

```c
void board_control_spi_sensors_power(bool enable_power, int bus_mask);
```

- **函数位置**: `platforms/nuttx/src/px4/nxp/imxrt/spi/spi.cpp`
- **参数说明**:
  - `enable_power`: true=上电, false=断电
  - `bus_mask`: 总线掩码，对于 SPI3 使用 `1 << (3-1) = 0x04`

### 电源循环时序

```
驱动启动
    |
    v
断电 SPI3 (VDD_3V3_SENSORS3_EN = 0)
    |
    v
等待 100ms (电容放电)
    |
    v
上电 SPI3 (VDD_3V3_SENSORS3_EN = 1)
    |
    v
等待 1s (电源稳定 + 传感器初始化)
    |
    v
SPI::init() (配置 SPI 接口)
    |
    v
Reset() (软件复位传感器)
    |
    v
传感器正常工作
```

### GPIO 控制

- **电源引脚**: GPIO_VDD_3V3_SENSORS3_EN (GPIO_EMC_B1_14, GPIO1_IO14)
- **总线掩码**: 0x04 (二进制 0000 0100，第2位)
- **计算方法**: `1 << (bus_number - 1)` = `1 << (3 - 1)` = `1 << 2` = 4

## 优势

### 1. 代码更清晰
- 电源管理逻辑集中在驱动内部
- 启动脚本更简洁
- 符合面向对象的封装原则

### 2. 更可靠
- 每次驱动启动都会执行电源循环
- 无需手动维护启动脚本
- 避免忘记添加电源控制命令

### 3. 更灵活
- 驱动可以根据需要调整电源时序
- 可以添加重试机制
- 方便添加更多的初始化逻辑

### 4. 日志更详细
- 驱动打印电源循环状态
- 便于调试和故障排查
- 可以精确追踪初始化过程

## 初始化流程对比

### 修改前（启动脚本控制）

```bash
rc.board_sensors:
  board_spi_reset 100 4    # 启动脚本控制电源
  sleep 1
  sch16t start             # 启动驱动

驱动内部:
  SCH16T::init()
    -> SPI::init()
    -> Reset()
```

### 修改后（驱动内部控制）

```bash
rc.board_sensors:
  sch16t start             # 仅启动驱动

驱动内部:
  SCH16T::init()
    -> board_control_spi_sensors_power(false, 0x04)  # 断电
    -> px4_usleep(100ms)
    -> board_control_spi_sensors_power(true, 0x04)   # 上电
    -> px4_usleep(1s)
    -> SPI::init()
    -> Reset()
```

## 验证方法

### 1. 编译固件
```bash
make sky_v6x-rt upload
```

### 2. 查看启动日志
连接串口后，观察启动日志：

```
nsh> sch16t start
[sch16t] Power cycling SPI3 sensor (100ms)
[sch16t] Power cycle complete, initializing sensor
[sch16t] Resetting (soft)
[sch16t] ASIC_ID: 0x21, COMP_ID: 0x23
[sch16t] Sensor configuration validated
[sch16t] Running on SPI3
```

### 3. 检查传感器状态
```bash
sch16t status
```

预期输出：
```
Running on SPI3
State: READ
Temperature: XX.X °C
Gyro range: ±300 °/s
Accel range: ±80 g
Data updates: XXXX
```

### 4. 监听数据
```bash
listener sensor_accel -n 10
listener sensor_gyro -n 10
```

应该看到稳定的数据流。

## 调试技巧

### 增加详细日志

如果需要更详细的调试信息，可以在驱动中添加：

```cpp
PX4_INFO("Power off SPI3");
board_control_spi_sensors_power(false, 0x04);
PX4_INFO("Waiting 100ms...");
px4_usleep(100000);

PX4_INFO("Power on SPI3");
board_control_spi_sensors_power(true, 0x04);
PX4_INFO("Waiting 1s for stabilization...");
px4_usleep(1000000);
```

### 调整时序参数

如果仍有问题，可以尝试调整：

```cpp
// 增加断电时间到 200ms
px4_usleep(200000);

// 增加稳定时间到 2s
px4_usleep(2000000);
```

### 添加错误检查

可以验证电源控制是否成功：

```cpp
// 读取 ADC 确认电压
// 或检查 GPIO 状态
```

## 相关函数

### board_control_spi_sensors_power()
- **功能**: 控制 SPI 总线的电源
- **实现**: `platforms/nuttx/src/px4/nxp/imxrt/spi/spi.cpp`
- **调用**: 遍历所有 SPI 总线，根据掩码控制对应的电源 GPIO

### board_spi_reset()
- **功能**: 完整的 SPI 总线复位（断电+重新初始化）
- **实现**: 调用 `board_control_spi_sensors_power()` + 引脚重新配置
- **区别**: 比单纯的电源控制更彻底，但在驱动内部不需要

## 注意事项

1. **总线掩码计算**: 确保使用正确的掩码值 `1 << (bus_number - 1)`
2. **延迟时间**: 100ms 断电 + 1s 稳定时间是经验值，可能需要根据实际情况调整
3. **其他传感器**: 如果 SPI3 上有多个传感器，电源循环会影响所有设备
4. **线程安全**: 确保没有其他线程同时访问该传感器
5. **错误处理**: 如果初始化失败，电源状态会保持在上电

## 未来优化

### 1. 添加重试机制
```cpp
int retry_count = 3;
while (retry_count-- > 0) {
    if (init_sensor() == PX4_OK) {
        break;
    }
    // 再次电源循环
    power_cycle();
}
```

### 2. 检测热插拔
```cpp
// 定期检查传感器是否在线
// 如果掉线，自动重新初始化
```

### 3. 电源状态监控
```cpp
// 读取 ADC 监控电压
// 确保电源稳定后再初始化
```

## 总结

✅ **优点**:
- 代码更清晰，逻辑集中
- 自动化的电源管理
- 更好的封装性
- 详细的日志输出

⚠️ **注意**:
- 确保总线掩码正确
- 注意延迟时间设置
- 考虑总线上其他设备的影响

📝 **测试**:
- 编译并上传固件
- 检查启动日志
- 验证传感器数据
