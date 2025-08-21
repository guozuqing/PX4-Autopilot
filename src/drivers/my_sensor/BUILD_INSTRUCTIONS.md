# MySensor驱动编译和安装指导

## 问题诊断

如果出现 `nsh: my_sensor: command not found` 错误，说明驱动没有被正确编译到固件中。

## 解决步骤

### 1. 检查配置文件

确保 `boards/px4/fmu-v6x/default.px4board` 文件中包含：
```
CONFIG_DRIVERS_MY_SENSOR=y
```

### 2. 清理并重新编译

```bash
# 清理构建
make distclean

# 重新配置和编译V6X固件
make px4_fmu-v6x_default

# 或者如果你使用的是其他板子
make px4_fmu-v6x_default upload
```

### 3. 检查编译日志

在编译过程中查找MySensor相关的编译信息：
```bash
make px4_fmu-v6x_default 2>&1 | grep -i my_sensor
```

应该看到类似的输出：
```
[xxx/xxx] Building CXX object src/drivers/my_sensor/CMakeFiles/drivers__my_sensor.dir/my_sensor.cpp.o
[xxx/xxx] Linking CXX static library src/drivers/my_sensor/libdrivers__my_sensor.a
```

### 4. 验证固件中包含驱动

编译完成后，可以检查固件是否包含MySensor：
```bash
# 检查固件符号表
nm build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf | grep my_sensor
```

### 5. 手动测试编译

如果仍有问题，可以单独编译MySensor模块：
```bash
# 进入构建目录
cd build/px4_fmu-v6x_default

# 单独编译MySensor
make drivers__my_sensor
```

## 替代方案

### 方案1: 使用现有驱动测试

如果MySensor编译有问题，可以先使用现有的GPS驱动测试UART功能：
```bash
# 在TELEM3端口启动GPS驱动
gps start -d /dev/ttyS2 -b 9600

# 查看状态
gps status

# 停止
gps stop
```

### 方案2: 简化版驱动

创建一个最简化的测试版本，去掉复杂的功能：

```cpp
// 简化版my_sensor.cpp
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_log.h>

extern "C" __EXPORT int my_sensor_main(int argc, char *argv[]);

int my_sensor_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_INFO("Usage: my_sensor {start|stop|status}");
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        PX4_INFO("MySensor: Hello World!");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        PX4_INFO("MySensor: Status OK");
        return 0;
    }

    if (strcmp(argv[1], "stop") == 0) {
        PX4_INFO("MySensor: Stopped");
        return 0;
    }

    PX4_ERR("Unknown command: %s", argv[1]);
    return 1;
}
```

## 调试命令

### 检查可用命令
```bash
help | grep sensor
```

### 检查模块加载
```bash
# 查看所有可用的模块
help

# 查看系统信息
ver all

# 查看工作队列
work_queue status
```

### 检查串口设备
```bash
# 列出所有串口设备
ls /dev/tty*

# 检查串口状态
cat /proc/devices | grep tty
```

## 常见错误和解决方案

### 错误1: "command not found"
- 原因: 驱动没有编译到固件中
- 解决: 检查配置文件，重新编译固件

### 错误2: 编译错误
- 原因: 代码语法错误或依赖问题
- 解决: 检查编译日志，修复代码错误

### 错误3: 链接错误
- 原因: 缺少依赖库或符号
- 解决: 检查CMakeLists.txt中的依赖配置

### 错误4: 运行时错误
- 原因: 运行时逻辑错误
- 解决: 添加调试日志，使用gdb调试

## 成功验证

当驱动正确编译和安装后，应该能看到：

```bash
nsh> help | grep my_sensor
my_sensor

nsh> my_sensor start
MySensor started on /dev/ttyS2 at 9600 baud

nsh> my_sensor status
UART: /dev/ttyS2 at 9600 baud
Messages: 0, Parse errors: 0, UART errors: 0
Data rate: 0.0 Hz

nsh> listener sensor_simple
# 应该能看到传感器消息（如果有数据输入）
```
