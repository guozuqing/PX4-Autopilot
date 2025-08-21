# 简化版 MySensor 驱动

## 功能说明
这是一个最简单的UART传感器驱动，专门为初学者设计：
- **只使用TELEM3端口** (`/dev/ttyS2`)
- **固定波特率9600**
- **只发布接收到的数据**
- **没有复杂的配置和参数**

## 使用方法

### 1. 编译
确保 `boards/px4/fmu-v6x/default.px4board` 文件中有：
```
CONFIG_DRIVERS_MY_SENSOR=y
```

然后编译：
```bash
make px4_fmu-v6x_default
```

### 2. 使用命令

```bash
# 启动驱动
my_sensor start

# 查看状态
my_sensor status

# 停止驱动
my_sensor stop

# 监听数据
listener sensor_simple
```

## 硬件连接

连接你的传感器到TELEM3端口：
- 传感器TX -> PX4 RX (PD9引脚)
- 传感器GND -> PX4 GND
- 传感器VCC -> PX4 5V或3.3V

## 数据格式

### 输入格式
传感器应该发送以换行符结尾的文本数据：
```
Hello World
123
Temperature: 25.6
```

### 输出消息
驱动会发布 `sensor_simple` 消息，包含：
- `timestamp`: 时间戳
- `data`: 接收到的原始字符串数据

## 测试方法

### 使用Arduino测试
```cpp
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello PX4!");
  delay(1000);
}
```

### 使用串口工具测试
```bash
# 在Linux上
echo "Test Data" > /dev/ttyUSB0
```

## 查看结果

启动驱动后，使用以下命令查看接收到的数据：
```bash
listener sensor_simple
```

你应该能看到类似的输出：
```
TOPIC: sensor_simple
timestamp: 1234567890
data: Hello PX4!
```

## 代码说明

这个驱动非常简单：
1. **初始化**: 打开TELEM3串口，设置9600波特率
2. **读取**: 每100ms读取一次串口数据
3. **处理**: 当遇到换行符时，把这一行数据发布出去
4. **发布**: 通过uORB发布sensor_simple消息

## 常见问题

**Q: 为什么选择TELEM3？**
A: TELEM3通常没有被其他重要功能占用，比较安全用于测试。

**Q: 可以改成其他串口吗？**
A: 可以，修改代码中的`"/dev/ttyS2"`为其他串口设备文件。

**Q: 可以改波特率吗？**
A: 可以，修改代码中的`setBaudrate(9600)`为其他波特率。

**Q: 驱动启动失败怎么办？**
A: 检查串口是否被其他程序占用，或者重新编译固件。
