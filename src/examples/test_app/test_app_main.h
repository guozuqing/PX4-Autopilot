#pragma once  // 防止头文件重复包含，确保同一头文件在编译单元中仅处理一次
#include <px4_platform_common/module.h>  // 引入 ModuleBase 模板类，同时隐含包含 tasks、log、time 等常用头文件（对应注释(1)）

// 定义 TestApp 类，继承自 ModuleBase<TestApp>
// ModuleBase 是模板类，需显式指定模板参数为 TestApp（对应注释(3)）
class TestApp : public ModuleBase<TestApp>
{
public:
    TestApp();  // 构造函数：负责初始化 TestApp 对象
    virtual ~TestApp() = default;  // 虚析构函数：编译器自动生成默认实现，保障多态析构安全
    // 静态成员函数：创建程序工作（如任务创建、队列注册等，对应注释）
    static int task_spawn(int argc, char *argv[]);
    // 静态成员函数：创建 TestApp 类实例，供 ModuleBase 父类逻辑调用（对应注释）
    static TestApp *instantiate(int argc, char *argv[]);
    // 静态成员函数：处理程序自定义命令（响应非标准 CLI 指令，对应注释）
    static int custom_command(int argc, char *argv[]);
    // 静态成员函数：打印程序用法帮助，reason 可选（用于解释错误场景，对应注释）
    static int print_usage(const char *reason = nullptr);
    // 成员函数：程序核心工作逻辑，重写 ModuleBase 规定的虚函数（对应注释）
    void run() override;
};
