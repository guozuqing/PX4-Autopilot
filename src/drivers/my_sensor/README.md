# MySensor 驱动使用说明

## 概述

MySensor是一个简单的UART传感器驱动，用于读取通过串口发送的基本数字数据。它可以解析简单的文本格式数据（如"123", "45.67"等）并通过uORB消息发布。

## 功能特性

- 支持任意UART端口配置
- 自动解析整数和浮点数
- 实时数据速率统计
- 错误计数和恢复
- 可配置的采样率和缓冲区大小
- 完整的性能监控

## 配置选项

### Kconfig配置

在 `menuconfig` 中的配置路径：
```
Device Drivers -> my_sensor
```

主要配置选项：
- `DRIVERS_MY_SENSOR`: 启用MySensor驱动
- `MY_SENSOR_DEFAULT_PORT`: 默认UART端口 (默认: /dev/ttyS2)
- `MY_SENSOR_DEFAULT_BAUDRATE`: 默认波特率 (默认: 9600)
- `MY_SENSOR_BUFFER_SIZE`: 接收缓冲区大小 (默认: 64字节)
- `MY_SENSOR_SAMPLE_RATE_HZ`: 采样率 (默认: 10Hz)

### 运行时参数

- `MY_SENSOR_EN`: 启用/禁用驱动 (0/1)
- `MY_SENSOR_PORT`: UART端口选择 (1-7)
- `MY_SENSOR_BAUD`: 波特率设置
- `MY_SENSOR_RATE`: 采样率设置

## 使用方法

### 1. 编译配置

在板子配置文件 `boards/px4/fmu-v6x/default.px4board` 中添加：
```
CONFIG_DRIVERS_MY_SENSOR=y
```

### 2. 命令行启动

```bash
# 使用默认配置启动
my_sensor start

# 指定串口和波特率
my_sensor start -d /dev/ttyS2 -b 9600

# 在GPS1端口启动
my_sensor start -d /dev/ttyS1 -b 38400

# 在TELEM1端口启动
my_sensor start -d /dev/ttyS6 -b 57600
```

### 3. 查看状态

```bash
# 查看驱动状态
my_sensor status

# 停止驱动
my_sensor stop
```

### 4. 监听数据

```bash
# 监听传感器消息
listener sensor_simple

# 使用MAVLink监听
mavlink stream -d /dev/ttyS6 -s SENSOR_SIMPLE -r 10
```

## 数据格式

### 输入格式

驱动期望接收以换行符结尾的文本数据：
```
123\n
45.67\n
-789\n
3.14159\n
```

### 输出消息 (sensor_simple)

```cpp
struct sensor_simple_s {
    uint64 timestamp;        // 时间戳 (微秒)
    uint32 device_id;        // 设备ID
    string data_string;      // 原始字符串数据
    int32 numeric_value;     // 解析的数值
    bool parse_success;      // 解析是否成功
    uint32 message_count;    // 消息计数
    uint32 parse_errors;     // 解析错误计数
    uint32 uart_errors;      // UART错误计数
    float32 data_rate_hz;    // 数据速率 (Hz)
    uint64 last_valid_time;  // 最后有效数据时间
};
```

## 硬件连接

### V6X H7板子串口映射

| 设备文件 | UART控制器 | TX引脚 | RX引脚 | 用途 |
|---------|-----------|--------|--------|------|
| /dev/ttyS0 | USART1 | PB6 | PB7 | 调试 |
| /dev/ttyS1 | USART2 | PD5 | PA3 | GPS1 |
| /dev/ttyS2 | USART3 | PD8 | PD9 | TELEM3 |
| /dev/ttyS3 | UART4 | PH13 | PH14 | GPS2 |
| /dev/ttyS4 | UART5 | PC12 | PD2 | TELEM2 |
| /dev/ttyS5 | USART6 | PC6 | PC7 | RC/PX4IO |
| /dev/ttyS6 | UART7 | PE8 | PF6 | TELEM1 |
| /dev/ttyS7 | UART8 | PE1 | PE0 | 控制台 |

### 连接示例

连接一个简单的传感器到TELEM3端口：
- 传感器TX -> PX4 RX (PD9)
- 传感器RX -> PX4 TX (PD8) [如果需要]
- 传感器GND -> PX4 GND
- 传感器VCC -> PX4 5V/3.3V

## 测试方法

### 1. 使用串口工具测试

在Linux/Mac上使用screen或minicom：
```bash
# 连接到串口
screen /dev/ttyUSB0 9600

# 发送测试数据
echo "123" > /dev/ttyUSB0
echo "45.67" > /dev/ttyUSB0
echo "-789" > /dev/ttyUSB0
```

### 2. 使用Arduino测试

```cpp
// Arduino测试代码
void setup() {
  Serial.begin(9600);
}

void loop() {
  static int counter = 0;

  // 发送递增的数字
  Serial.println(counter++);

  delay(1000);  // 每秒发送一次
}
```

### 3. Python测试脚本

```python
#!/usr/bin/env python3
import serial
import time
import random

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

try:
    while True:
        # 发送随机数据
        value = random.randint(-1000, 1000)
        ser.write(f"{value}\n".encode())
        print(f"Sent: {value}")
        time.sleep(1)

except KeyboardInterrupt:
    ser.close()
    print("Test stopped")
```

## 故障排除

### 常见问题

1. **驱动无法启动**
   - 检查串口设备文件是否存在：`ls -l /dev/ttyS*`
   - 检查Kconfig配置是否正确启用
   - 检查编译是否包含该模块

2. **无数据接收**
   - 检查硬件连接
   - 检查波特率设置是否匹配
   - 使用示波器或逻辑分析仪检查信号

3. **解析错误**
   - 检查数据格式是否正确（需要换行符结尾）
   - 检查是否有特殊字符干扰
   - 查看错误统计：`my_sensor status`

### 调试命令

```bash
# 查看系统日志
dmesg | grep -i uart

# 查看中断统计
cat /proc/interrupts | grep uart

# 监听原始uORB消息
listener sensor_simple

# 查看驱动性能
my_sensor status
```

## 扩展开发

### 添加新的数据格式支持

在 `processCompleteLine()` 函数中添加自定义解析逻辑：

```cpp
void MySensor::processCompleteLine()
{
    // 现有的数值解析代码...

    // 添加自定义格式解析
    if (strncmp(_rx_buffer, "TEMP:", 5) == 0) {
        // 解析温度数据格式: "TEMP:25.6"
        float temp = strtof(_rx_buffer + 5, nullptr);
        sensor_msg.numeric_value = (int32_t)(temp * 100); // 保存为厘度
        sensor_msg.parse_success = true;
    }
}
```

### 添加多数据字段支持

修改 `SensorSimple.msg` 文件添加更多字段：

```
# 扩展的传感器消息
uint64 timestamp
uint32 device_id

string data_string
int32 value1
int32 value2
float32 temperature
float32 humidity
bool parse_success

# 统计信息...
```

这个完整的MySensor驱动提供了一个简单但功能完整的UART数据读取和发布框架，可以作为更复杂传感器驱动的基础。
