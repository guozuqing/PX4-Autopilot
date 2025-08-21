# MySensor驱动集成说明

## 需要添加到现有文件的内容

### 1. 在 `src/drivers/CMakeLists.txt` 中添加：

```cmake
# 在适当位置添加以下行
add_subdirectory(my_sensor)
```

### 2. 在板子配置文件中启用驱动

在 `boards/px4/fmu-v6x/default.px4board` 中添加：

```
CONFIG_DRIVERS_MY_SENSOR=y
CONFIG_MY_SENSOR_DEFAULT_PORT="/dev/ttyS2"
CONFIG_MY_SENSOR_DEFAULT_BAUDRATE=9600
CONFIG_MY_SENSOR_BUFFER_SIZE=64
CONFIG_MY_SENSOR_SAMPLE_RATE_HZ=10
```

### 3. 在启动脚本中添加驱动启动

在 `boards/px4/fmu-v6x/init/rc.board_sensors` 中添加：

```bash
# 启动MySensor驱动
my_sensor start -d /dev/ttyS2 -b 9600
```

或者创建单独的启动脚本 `boards/px4/fmu-v6x/init/rc.my_sensor`:

```bash
#!/bin/sh
#
# MySensor startup script
#

# 检查是否启用MySensor
if param compare MY_SENSOR_EN 1
then
    # 获取配置参数
    set MY_SENSOR_PORT /dev/ttyS2
    set MY_SENSOR_BAUD 9600

    if param compare MY_SENSOR_PORT 1
    then
        set MY_SENSOR_PORT /dev/ttyS1
    fi

    if param compare MY_SENSOR_PORT 2
    then
        set MY_SENSOR_PORT /dev/ttyS2
    fi

    if param compare MY_SENSOR_PORT 6
    then
        set MY_SENSOR_PORT /dev/ttyS6
    fi

    # 启动驱动
    my_sensor start -d $MY_SENSOR_PORT -b $MY_SENSOR_BAUD

    unset MY_SENSOR_PORT
    unset MY_SENSOR_BAUD
fi
```
