//引入自定义头文件：包含SkyCdev类的声明
#include "test_cdev_main.h"
#include <px4_log.h>  // 包含 PX4 日志头文件，定义了 PX4_INFO 等宏

//定义设备路径：驱动注册后，应用通过"/dev/testCDev"访问该设备
#define DEV_PATH "/dev/testCDev"

//导出主函数（C 风格符号），让PX4系统能识别并调用该入口
extern "C" __EXPORT int test_cdev_main(int argc, char *argv[]);

// ===================== SkyCdev 类成员实现 ===================== //

//构造函数：调用基类CDev的构造函数，传入设备路径完成初始化
//注册的驱动文件名为 /dev/testCDev
SkyApp::SkyApp(const char *path) : CDev(path) {}

//析构函数：驱动销毁时执行资源清理
SkyApp::~SkyApp() = default;

//重写open接口：处理“设备打开”请求
//仅打印调用日志（演示用，没有使用参数file_t的成员变量f_priv），实际功能依赖基类 CDev::open
int SkyApp::open(cdev::file_t *filp)
{
    // __func__ 是编译器内置宏，自动获取当前函数名（如 "open"），用于调试输出
    PX4_INFO("Function : SkyCdev::%s has been called!", __func__);
    return CDev::open(filp);  // 调用基类逻辑，完成默认“打开”操作
}

//重写 close 接口：处理“设备关闭”请求
int SkyCdev::close(cdev::file_t *filp)
{
    PX4_INFO("Function : SkyCdev::%s has been called!", __func__);
    return CDev::close(filp);  // 调用基类逻辑，完成默认“关闭”操作
}

//重写 read 接口：处理“数据读取”请求
//模拟返回固定测试字符串，演示驱动与应用的数据交互
ssize_t SkyCdev::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
    PX4_INFO("Function : SkyCdev::%s has been called!", __func__);

    char rbuf[] = "This is read test! \0";  // 定义固定测试数据（可读内容）
    memcpy(buffer, rbuf, strlen(rbuf));     // 将测试数据复制到应用提供的缓冲区
    return strlen(rbuf);                    // 返回实际读取的字节数
}

//重写 write 接口：处理“数据写入”请求
ssize_t SkyCdev::write(cdev::file_t *filp, const char *buffer, size_t buflen)
{
    char wbuf[50];  // 定义缓冲区，存储应用写入的数据
    memcpy(wbuf, buffer, buflen + 1);  // 复制数据（+1 确保包含字符串结束符，需注意缓冲区大小）
    PX4_INFO("Function : SkyCdev::%s has been called!", __func__);
    // 打印写入的长度和内容，方便调试
    PX4_INFO("Write buflen is: %lu, and content is: %s", buflen, wbuf);

    return buflen;  // 返回实际写入的字节数（直接返回输入长度，模拟“写入成功”）
}

// ===================== 驱动主入口函数 ===================== //

//驱动初始化入口：PX4 应用的启动逻辑
//完成驱动对象创建、初始化（注册到系统）
int test_cdev_main(int argc, char *argv[])
{
    PX4_INFO("This is a cdev test!");  // 打印日志，标识驱动启动

    SkyApp *cdev = new SkyApp(DEV_PATH);  // 创建驱动对象，传入设备路径
    cdev->init();  // 调用基类CDev的init方法，完成驱动注册

    return PX4_OK;  // 返回成功状态码，告知系统初始化完成
