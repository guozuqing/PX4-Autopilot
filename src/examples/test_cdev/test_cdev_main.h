#pragma once  // 编译指令：防止头文件被重复包含（替代#ifndef系列写法）
#include <lib/cdev/CDev.hpp>  // 引入CDev基类的头文件，用于继承

// 定义TestCdev类，公有继承自cdev命名空间下的 CDev 类
class TestCdev : public cdev::CDev  //TestCdev
{
public:
    TestCdev(const char *devname);  // 构造函数：初始化设备（参数为设备名称）
    virtual ~TestCdev();  // 虚析构函数：确保派生类资源正确释放

    // 以下函数为重写 CDev 类的虚接口（驱动标准访问接口）：
    int open(cdev::file_t *filp) override;  // 重写“打开设备”接口
    int close(cdev::file_t *filp) override;  // 重写“关闭设备”接口
    ssize_t read(cdev::file_t *filp, char *buffer, size_t buflen) override;  // 重写“读取数据”接口
    ssize_t write(cdev::file_t *filp, const char *buffer, size_t buflen) override;  // 重写“写入数据”接口
};
