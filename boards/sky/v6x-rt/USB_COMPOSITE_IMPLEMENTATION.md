# USB CDC-MSC 复合设备实现文档

## 概述

本文档描述了在 `sky/v6x-rt` 板子上实现 USB CDC-MSC 复合设备的完整过程。该实现允许飞控同时提供：

- **CDC ACM (USB 串口)**：用于 MAVLink 通信和 NSH 控制台
- **MSC (USB 大容量存储)**：将 SD 卡暴露为 USB 存储设备，方便日志下载

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                         PC (Host)                           │
│  ┌─────────────────┐         ┌─────────────────┐           │
│  │   /dev/ttyACM0  │         │   /dev/sdX      │           │
│  │   (串口通信)     │         │   (U盘/SD卡)    │           │
│  └────────┬────────┘         └────────┬────────┘           │
└───────────┼────────────────────────────┼───────────────────┘
            │                            │
            │         USB Cable          │
            │                            │
┌───────────┴────────────────────────────┴───────────────────┐
│                    USB Composite Device                     │
│  ┌─────────────────┐         ┌─────────────────┐           │
│  │    CDC ACM      │         │      MSC        │           │
│  │  Interface 0,1  │         │   Interface 2   │           │
│  │  EP1,EP2,EP3    │         │    EP4,EP5      │           │
│  └────────┬────────┘         └────────┬────────┘           │
│           │                           │                     │
│  ┌────────┴───────────────────────────┴────────┐           │
│  │           NuttX USB Device Driver            │           │
│  │              (i.MX RT1170 OTG)               │           │
│  └──────────────────────────────────────────────┘           │
│                                                             │
│  ┌──────────────────┐      ┌──────────────────┐            │
│  │   /dev/ttyACM0   │      │   /dev/mmcsd0    │            │
│  │   (USB串口设备)   │      │   (SD卡块设备)   │            │
│  └──────────────────┘      └──────────────────┘            │
│                                                             │
│                      sky/v6x-rt Board                       │
└─────────────────────────────────────────────────────────────┘
```

## 修改的文件列表

| 文件路径 | 修改类型 | 说明 |
|---------|---------|------|
| `boards/sky/v6x-rt/nuttx-config/nsh/defconfig` | 修改 | 添加 Composite 设备配置 |
| `boards/sky/v6x-rt/src/composite.c` | 新建/修改 | 实现 `board_composite_initialize` 和 `board_composite_connect` |
| `boards/sky/v6x-rt/src/CMakeLists.txt` | 修改 | 添加 `composite.c` 到编译列表 |
| `boards/sky/v6x-rt/src/usbmsc.c` | 已存在 | 实现 `board_usbmsc_initialize` |
| `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp` | 修改 | 支持 Composite 模式 |
| `platforms/nuttx/cmake/upload.cmake` | 修改 | 修复 Composite 模式下的编译问题 |

---

## 详细配置说明

### 1. NuttX defconfig 配置

文件: `boards/sky/v6x-rt/nuttx-config/nsh/defconfig`

#### CDC ACM 配置 (Composite 模式)

```kconfig
CONFIG_CDCACM=y
CONFIG_CDCACM_COMPOSITE=y
CONFIG_CDCACM_BULKIN_REQLEN=96
CONFIG_CDCACM_PRODUCTID=0x001d
CONFIG_CDCACM_PRODUCTSTR="PX4 FMU v6XRT.x"
CONFIG_CDCACM_RXBUFSIZE=600
CONFIG_CDCACM_TXBUFSIZE=12000
CONFIG_CDCACM_VENDORID=0x3643
CONFIG_CDCACM_VENDORSTR="Dronecode Project, Inc."
```

#### Composite 设备配置

```kconfig
CONFIG_COMPOSITE_IAD=y
CONFIG_COMPOSITE_PRODUCTID=0x001f
CONFIG_COMPOSITE_PRODUCTSTR="PX4 FMU v6XRT Composite"
CONFIG_COMPOSITE_VENDORID=0x3643
CONFIG_COMPOSITE_VENDORSTR="Dronecode Project, Inc."
```

#### USB 设备核心配置

```kconfig
CONFIG_USBDEV=y
CONFIG_USBDEV_BUSPOWERED=y
CONFIG_USBDEV_COMPOSITE=y
CONFIG_USBDEV_DMA=y
CONFIG_USBDEV_DUALSPEED=y
CONFIG_USBDEV_MAXPOWER=500
```

#### USB MSC 配置 (Composite 模式)

```kconfig
CONFIG_USBMSC=y
CONFIG_USBMSC_COMPOSITE=y
CONFIG_USBMSC_EPBULKIN=5
CONFIG_USBMSC_EPBULKOUT=4
CONFIG_USBMSC_NWRREQS=8
CONFIG_USBMSC_NRDREQS=8
CONFIG_USBMSC_BULKINREQLEN=2048
CONFIG_USBMSC_BULKOUTREQLEN=2048
CONFIG_USBMSC_VENDORID=0x3643
CONFIG_USBMSC_VENDORSTR="PX4"
CONFIG_USBMSC_PRODUCTID=0x001e
CONFIG_USBMSC_PRODUCTSTR="PX4 FMU v6XRT SD"
CONFIG_USBMSC_NLUNS=1
CONFIG_USBMSC_REMOVABLE=y
```

#### Windows 兼容性关键配置

| 配置项 | 推荐值 | 说明 |
|-------|-------|------|
| `CONFIG_USBMSC_REMOVABLE` | y | **必须**，否则 Windows 不显示盘符 |
| `CONFIG_USBMSC_NWRREQS` | 8 | 写请求队列深度，增大可提高稳定性 |
| `CONFIG_USBMSC_NRDREQS` | 8 | 读请求队列深度，增大可提高稳定性 |
| `CONFIG_USBMSC_BULKINREQLEN` | 2048 | 批量IN缓冲区，增大可避免扇区读取错误 |
| `CONFIG_USBMSC_BULKOUTREQLEN` | 2048 | 批量OUT缓冲区，增大可避免扇区写入错误 |

> **重要**: 默认的 512 字节缓冲区在 Windows 上可能出现扇区读取错误，建议使用 2048 或更大的值。

#### 系统命令配置

```kconfig
CONFIG_BOARDCTL_USBDEVCTRL=y
CONFIG_SYSTEM_COMPOSITE=y
CONFIG_SYSTEM_CDCACM=y
CONFIG_SYSTEM_USBMSC=y
CONFIG_SYSTEM_USBMSC_NLUNS=1
CONFIG_SYSTEM_USBMSC_DEVMINOR1=0
CONFIG_SYSTEM_USBMSC_DEVPATH1="/dev/mmcsd0"
```

#### 端点分配说明

| 端点 | 用途 | 方向 |
|-----|------|-----|
| EP1 | CDC ACM Interrupt IN | IN |
| EP2 | CDC ACM Bulk OUT | OUT |
| EP3 | CDC ACM Bulk IN | IN |
| EP4 | MSC Bulk OUT | OUT |
| EP5 | MSC Bulk IN | IN |

---

### 2. Board Composite 实现

文件: `boards/sky/v6x-rt/src/composite.c`

```c
/**
 * @file composite.c
 *
 * Board-specific USB Composite Device support (CDC/ACM + Mass Storage).
 */

#include <nuttx/config.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

#include <sys/types.h>
#include <stdint.h>
#include <syslog.h>
#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>
#include <sys/mount.h>

#ifdef CONFIG_USBMSC_COMPOSITE
static void *g_mschandle;
#endif

#ifdef CONFIG_USBMSC_COMPOSITE
/**
 * @brief 创建 MSC 类对象
 *
 * 该函数在复合设备初始化时被调用，用于配置 MSC 并绑定 SD 卡
 */
static int board_mscclassobject(int minor,
                                struct usbdev_devinfo_s *devinfo,
                                struct usbdevclass_driver_s **classdev)
{
    int ret;

    DEBUGASSERT(g_mschandle == NULL);

    /* 在绑定 MSC 之前先卸载 SD 卡，避免冲突 */
    syslog(LOG_INFO, "board_mscclassobject: Unmounting /fs/microsd\n");
    ret = umount("/fs/microsd");
    if (ret < 0) {
        syslog(LOG_WARNING, "umount /fs/microsd failed: %d (may not be mounted)\n", -ret);
        /* 继续执行 - 可能尚未挂载 */
    }

    /* 配置 MSC，设置 LUN 数量为 1 */
    syslog(LOG_INFO, "board_mscclassobject: Configuring NLUNS=1\n");
    ret = usbmsc_configure(1, &g_mschandle);

    if (ret < 0) {
        syslog(LOG_ERR, "usbmsc_configure failed: %d\n", -ret);
        return ret;
    }

    /* 将 /dev/mmcsd0 (SD卡) 绑定到 LUN 0 */
    syslog(LOG_INFO, "board_mscclassobject: Bind LUN=0 to /dev/mmcsd0\n");
    ret = usbmsc_bindlun(g_mschandle, "/dev/mmcsd0", 0, 0, 0, false);

    if (ret < 0) {
        syslog(LOG_ERR, "usbmsc_bindlun failed: %d\n", -ret);
        usbmsc_uninitialize(g_mschandle);
        g_mschandle = NULL;
        return ret;
    }

    /* 获取 MSC 类对象 */
    ret = usbmsc_classobject(g_mschandle, devinfo, classdev);

    if (ret < 0) {
        syslog(LOG_ERR, "usbmsc_classobject failed: %d\n", -ret);
        usbmsc_uninitialize(g_mschandle);
        g_mschandle = NULL;
    }

    return ret;
}

/**
 * @brief 释放 MSC 类对象
 */
static void board_mscuninitialize(struct usbdevclass_driver_s *classdev)
{
    DEBUGASSERT(g_mschandle != NULL);
    usbmsc_uninitialize(g_mschandle);
    g_mschandle = NULL;
}
#endif

/**
 * @brief 板级复合设备初始化
 *
 * 由 boardctl(BOARDIOC_USBDEV_CONTROL) 调用
 */
int board_composite_initialize(int port)
{
    return OK;
}

/**
 * @brief 连接复合设备
 *
 * 创建并返回复合设备句柄，包含 CDC ACM 和 MSC 两个功能
 *
 * @param port USB 端口号
 * @param configid 配置 ID (0 = CDC+MSC)
 * @return 复合设备句柄，失败返回 NULL
 */
void *board_composite_connect(int port, int configid)
{
    if (configid == 0) {
#ifdef CONFIG_USBMSC_COMPOSITE
        struct composite_devdesc_s dev[2];
        int ifnobase = 0;
        int strbase  = COMPOSITE_NSTRIDS;

        /* 配置 CDC ACM (设备 0) */
        cdcacm_get_composite_devdesc(&dev[0]);

        dev[0].classobject  = cdcacm_classobject;
        dev[0].uninitialize = cdcacm_uninitialize;
        dev[0].devinfo.ifnobase = ifnobase;
        dev[0].minor = 0;
        dev[0].devinfo.strbase = strbase;
        dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;  /* EP1: Interrupt IN */
        dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 2;  /* EP2: Bulk OUT */
        dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 3;  /* EP3: Bulk IN */

        ifnobase += dev[0].devinfo.ninterfaces;
        strbase  += dev[0].devinfo.nstrings;

        /* 配置 MSC (设备 1) */
        usbmsc_get_composite_devdesc(&dev[1]);

        dev[1].classobject  = board_mscclassobject;
        dev[1].uninitialize = board_mscuninitialize;
        dev[1].devinfo.ifnobase = ifnobase;
        dev[1].minor = 0;
        dev[1].devinfo.strbase = strbase;
        dev[1].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 4;  /* EP4: Bulk OUT */
        dev[1].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 5;  /* EP5: Bulk IN */

        /* 初始化复合设备，包含 2 个功能 */
        return composite_initialize(2, dev);
#endif
    }

    return NULL;
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
```

#### 关键函数说明

| 函数 | 说明 |
|------|------|
| `board_composite_initialize()` | 板级初始化，由 NuttX boardctl 调用 |
| `board_composite_connect()` | 创建复合设备，配置 CDC+MSC |
| `board_mscclassobject()` | 配置 MSC 类，绑定 SD 卡到 LUN |
| `board_mscuninitialize()` | 释放 MSC 资源 |

---

### 3. cdcacm_autostart 修改

文件: `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp`

#### 修改说明

在 `cdcacm_autostart` 模块中添加对 Composite 设备的支持。当检测到 USB VBUS 时，根据配置选择使用 `conn` (Composite) 或 `sercon` (standalone CDC ACM) 命令：

```cpp
__BEGIN_DECLS
#include <arch/board/board.h>
#include <builtin/builtin.h>

#if defined(CONFIG_USBDEV_COMPOSITE)
extern int conn_main(int c, char **argv);
extern int disconn_main(int c, char **argv);
#define usb_connect()    conn_main(0, nullptr)
#define usb_disconnect() disconn_main(0, nullptr)
#else
extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
#define usb_connect()    sercon_main(0, nullptr)
#define usb_disconnect() serdis_main(0, nullptr)
#endif
__END_DECLS
```

#### 状态机流程

```
                    ┌─────────────┐
                    │ disconnected│
                    └──────┬──────┘
                           │ VBUS detected
                           ▼
                    ┌─────────────┐
              ┌─────│ connecting  │
              │     └──────┬──────┘
              │            │ usb_connect() success
              │            ▼
              │     ┌─────────────┐
              │     │  connected  │
              │     └──────┬──────┘
              │            │ VBUS lost
              │            ▼
              │     ┌─────────────┐
              └────►│disconnecting│
                    └──────┬──────┘
                           │ usb_disconnect()
                           ▼
                    ┌─────────────┐
                    │ disconnected│
                    └─────────────┘
```

---

### 4. upload.cmake 修改

文件: `platforms/nuttx/cmake/upload.cmake`

#### 修改说明

修复 Composite 模式下 CMake 配置失败的问题。当启用 `CONFIG_USBDEV_COMPOSITE` 时，使用 Composite 的 vendor/product 字符串：

```cmake
# NuttX CDCACM vendor and product strings
# Use composite strings if in composite mode, otherwise use standalone CDCACM strings
set(vendorstr_underscore)
set(productstr_underscore)
if(CONFIG_USBDEV_COMPOSITE AND CONFIG_COMPOSITE_VENDORSTR)
    string(REPLACE " " "_" vendorstr_underscore ${CONFIG_COMPOSITE_VENDORSTR})
    string(REPLACE "," "_" vendorstr_underscore "${vendorstr_underscore}")
    string(REPLACE " " "_" productstr_underscore ${CONFIG_COMPOSITE_PRODUCTSTR})
elseif(CONFIG_CDCACM_VENDORSTR)
    string(REPLACE " " "_" vendorstr_underscore ${CONFIG_CDCACM_VENDORSTR})
    string(REPLACE "," "_" vendorstr_underscore "${vendorstr_underscore}")
    string(REPLACE " " "_" productstr_underscore ${CONFIG_CDCACM_PRODUCTSTR})
else()
    set(vendorstr_underscore "PX4")
    set(productstr_underscore "FMU")
endif()
```

---

### 5. CMakeLists.txt 修改

文件: `boards/sky/v6x-rt/src/CMakeLists.txt`

添加 `composite.c` 到编译列表：

```cmake
px4_add_library(drivers_board
    autoleds.c
    automount.c
    #can.c
    composite.c    # <-- 新增
    i2c.cpp
    init.c
    led.c
    mtd.cpp
    sdhc.c
    spi.cpp
    timer_config.cpp
    usb.c
    usbmsc.c
    # ... 其他文件
)
```

---

## 编译和烧录

### 编译固件

```bash
# 清理旧编译 (可选，修改 defconfig 后建议执行)
rm -rf build/sky_v6x-rt_default

# 编译
make sky_v6x-rt_default
```

### 烧录固件

```bash
# 通过 USB 烧录
make sky_v6x-rt_default upload

# 或手动使用 px_uploader.py
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/sky_v6x-rt_default/sky_v6x-rt_default.px4
```

---

## 使用说明

### PC 端 (Windows)

连接 USB 后，系统会自动识别两个设备：

1. **CDC ACM 串口**: `COMx` (如 COM3)
2. **MSC 存储设备**: 在"此电脑"中显示为可移动磁盘

#### 查看设备

1. **设备管理器** (`devmgmt.msc`)
   - 端口 (COM 和 LPT) → USB 串行设备 (COMx)
   - 磁盘驱动器 → PX4 FMU v6XRT SD

2. **磁盘管理** (`diskmgmt.msc`)
   - 可查看 SD 卡分区和盘符

#### 串口连接

使用 QGroundControl、PuTTY 或其他串口工具连接 COMx 端口。

#### 常见问题

- **只在设备管理器可见，"此电脑"无盘符**：检查 `CONFIG_USBMSC_REMOVABLE=y`
- **扇区读取错误**：增大缓冲区配置 (见 Windows 兼容性配置)
- **需要格式化提示**：SD 卡文件系统可能损坏，建议使用 FAT32 格式化

---

### PC 端 (Linux)

连接 USB 后，系统会自动识别两个设备：

1. **CDC ACM 串口**: `/dev/ttyACM0`
2. **MSC 存储设备**: `/dev/sdX` (如 `/dev/sdb`)

#### 查看设备

```bash
# 查看 USB 设备
lsusb

# 查看串口设备
ls -la /dev/ttyACM*

# 查看块设备
lsblk

# 查看内核日志
dmesg | tail -30
```

#### 挂载 SD 卡

```bash
# 自动挂载 (大多数桌面环境)
# 或手动挂载
sudo mount /dev/sdb1 /mnt/px4sd
```

### NSH 命令

```bash
# 查看 USB 设备状态
ls /dev

# 手动连接复合设备 (通常由 cdcacm_autostart 自动执行)
conn

# 手动断开复合设备
disconn

# 查看 SD 卡设备
ls /dev/mmcsd*
```

---

## 注意事项

### SD 卡访问冲突

当 MSC 功能启用时，SD 卡会被卸载 (`/fs/microsd`)，此时：

- ✅ PC 可以访问 SD 卡
- ❌ PX4 无法读写日志到 SD 卡
- ❌ 参数保存可能失败

**建议**：
- 在地面调试时使用 MSC 功能下载日志
- 飞行时断开 USB，SD 卡会自动重新挂载

### USB 重新枚举

断开 USB 后重新连接，`cdcacm_autostart` 会自动检测 VBUS 并重新初始化复合设备。

### 调试信息

可以通过查看 syslog 获取 USB 复合设备初始化日志：

```
board_mscclassobject: Unmounting /fs/microsd
board_mscclassobject: Configuring NLUNS=1
board_mscclassobject: Bind LUN=0 to /dev/mmcsd0
```

---

## 故障排除

### 问题 1: MSC 设备不显示

**可能原因**：
- SD 卡未插入或未被识别
- `/dev/mmcsd0` 不存在

**解决方法**：
```bash
# 在 NSH 中检查
ls /dev/mmcsd*
```

### 问题 2: 仅首次启动有效

**可能原因**：
- SD 卡在启动时被 PX4 挂载，导致 MSC 无法绑定

**解决方法**：
- 确保 `composite.c` 中包含 `umount("/fs/microsd")` 调用

### 问题 3: CDC 串口无响应

**可能原因**：
- `cdcacm_autostart` 模块未启动
- VBUS 检测失败

**解决方法**：
```bash
# 检查模块状态
cdcacm_autostart status

# 手动启动
cdcacm_autostart start
```

### 问题 4: 编译错误 "undefined reference to board_composite_*"

**解决方法**：
- 确保 `composite.c` 已添加到 `CMakeLists.txt`
- 确保 defconfig 中启用了 `CONFIG_USBDEV_COMPOSITE`

### 问题 5: Windows 不显示盘符 (设备管理器可见)

**可能原因**：
- 未启用 `CONFIG_USBMSC_REMOVABLE`

**解决方法**：
```kconfig
CONFIG_USBMSC_REMOVABLE=y
```

### 问题 6: Windows 扇区读取错误

**可能原因**：
- USB MSC 缓冲区太小，Windows 对时序要求严格

**解决方法**：
增大缓冲区和请求队列：
```kconfig
CONFIG_USBMSC_NWRREQS=8
CONFIG_USBMSC_NRDREQS=8
CONFIG_USBMSC_BULKINREQLEN=2048
CONFIG_USBMSC_BULKOUTREQLEN=2048
```

### 问题 7: Linux 正常但 Windows 无法读取

**可能原因**：
- Windows 和 Linux 对 USB MSC 协议实现差异
- 缓冲区对齐问题

**解决方法**：
1. 确保所有 Windows 兼容性配置已启用
2. 检查 SD 卡文件系统是否为 FAT32 (Windows 更好支持)
3. 尝试在 Windows 磁盘管理中格式化 SD 卡

---

## 配置选项汇总

### 核心配置

| 配置项 | 值 | 说明 |
|-------|-----|------|
| `CONFIG_USBDEV_COMPOSITE` | y | 启用 USB 复合设备 |
| `CONFIG_CDCACM_COMPOSITE` | y | CDC ACM 作为复合设备一部分 |
| `CONFIG_USBMSC_COMPOSITE` | y | MSC 作为复合设备一部分 |
| `CONFIG_COMPOSITE_IAD` | y | 使用接口关联描述符 |
| `CONFIG_SYSTEM_COMPOSITE` | y | 启用 conn/disconn 命令 |
| `CONFIG_BOARDCTL_USBDEVCTRL` | y | 启用 boardctl USB 控制 |

### Windows 兼容性配置 (重要)

| 配置项 | 值 | 说明 |
|-------|-----|------|
| `CONFIG_USBMSC_REMOVABLE` | y | **必须** - Windows 显示盘符 |
| `CONFIG_USBMSC_NWRREQS` | 8 | 写请求队列 (默认4，建议8) |
| `CONFIG_USBMSC_NRDREQS` | 8 | 读请求队列 (默认4，建议8) |
| `CONFIG_USBMSC_BULKINREQLEN` | 2048 | IN缓冲区 (默认512，建议2048) |
| `CONFIG_USBMSC_BULKOUTREQLEN` | 2048 | OUT缓冲区 (默认512，建议2048) |

---

## 参考资料

- [NuttX USB Device Documentation](https://nuttx.apache.org/docs/latest/components/drivers/character/usb.html)
- [PX4 Board Configuration](https://docs.px4.io/main/en/hardware/porting_guide.html)
- [USB Composite Device Class](https://www.usb.org/document-library/usb-interface-association-descriptor-device-class-code-and-use-model-10)

---

## 版本历史

| 日期 | 版本 | 修改说明 |
|------|------|---------|
| 2026-01-28 | 1.0 | 初始实现 CDC+MSC 复合设备 |
| 2026-01-28 | 1.1 | 添加 SD 卡自动卸载，修复重启后 MSC 不工作问题 |
| 2026-01-28 | 1.2 | 添加 `CONFIG_USBMSC_REMOVABLE` 修复 Windows 盘符显示 |
| 2026-01-28 | 1.3 | 增大缓冲区配置修复 Windows 扇区读取错误 |

