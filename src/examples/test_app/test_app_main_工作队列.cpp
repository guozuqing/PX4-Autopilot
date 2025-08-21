#include "test_app_main.h"  // 包含自定义的头文件 test_app_main.h，使编译器能识别 TestApp 类的声明、函数原型等，为后续代码实现提供依据
extern "C" __EXPORT int test_app_main(int argc, char *argv[]);  // 声明主函数，并指定为 C 链接方式，使其可被其他模块调用

// 调用基类 px4::ScheduledWorkItem 的构造函数，传入模块名 MODULE_NAME 和工作队列配置项 px4::wq_configurations::test，将 TestApp 实例添加到工作队列中，完成初步配置。
// 语法：C++ 构造函数初始化列表。用于在构造函数体执行前，初始化基类或成员变量
TestApp::TestApp() : px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test) {}

// 定义 TestApp 类的 init 成员函数
// 用于初始化工作队列相关的设置，设置任务调度的时间间隔
void TestApp::init()
{
    ScheduleOnInterval(5000000, 5000000);  // 调用 ScheduleOnInterval 函数（PX4 工作队列相关），向系统注册任务执行函数（Run 函数），并设置调度间隔为 5000000 微秒（即 5 秒），使任务按此周期执行。第一个参数（5000000 纳秒）：表示任务第一次被调度执行的初始延迟时间；第二个参数（5000000 纳秒）：表示任务在工作队列中被调度执行的周期间隔时间。
}

// 定义 TestApp 类的 Run 成员函数
// 当工作队列调度到该任务时，会调用这个函数来执行相应的任务逻辑
void TestApp::Run()
{
    PX4_INFO("This is a PX4 work queue test app!");  // 打印信息，表明这是 PX4 工作队列测试应用程序
}

// 定义 TestApp 类的 task_spawn 静态成员函数
// 用于创建 TestApp 类的实例并启动工作队列任务
int TestApp::task_spawn(int argc, char *argv[])
{
    TestApp *instance = instantiate(argc - 1, argv + 1);  // 创建 TestApp 类的实例，调整参数（去掉第一个参数），instantiate这个函数在下面定义了。
    _object.store(instance);  // 将实例指针存储到 _object 中，以便后续使用，这个用法是PX4自带的，记住就行了。

    if (instance)  // 如果实例创建成功
    {
        _task_id = task_id_is_work_queue;  // 设置任务 ID 为工作队列任务 ID
        instance->init();  // 调用实例的 init 函数，进行工作队列相关的初始化
        return PX4_OK;  // 返回成功状态码
    }
    else  // 如果实例创建失败
    {
        PX4_ERR("failed to instantiate object");  // 打印错误信息，表明实例化对象失败
        _task_id = -1;  // 设置任务 ID 为 -1，表示任务未启动
        return PX4_ERROR;  // 返回错误状态码
    }
}

// 定义 TestApp 类的 instantiate 静态成员函数
// 用于创建 TestApp 类的实例
TestApp *TestApp::instantiate(int argc, char *argv[])
{
    TestApp *instance = new TestApp();  // 使用 new 操作符创建 TestApp 类的实例
    if (instance == nullptr)  // 如果创建实例失败
    {
        PX4_ERR("alloc failed");  // 打印错误信息，表明内存分配失败
        return nullptr;  // 返回空指针
    }
    return instance;  // 返回创建成功的实例指针
}

// 定义 TestApp 类的 custom_command 成员函数
// 用于处理自定义命令，默认情况下返回未知命令的使用信息
int TestApp::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command!");  // 调用 print_usage 函数，打印未知命令的提示信息
}

// 定义 TestApp 类的 print_usage 成员函数
// 用于打印模块的使用说明和相关信息
int TestApp::print_usage(const char *reason)
{
    if (reason)  // 如果有原因参数
    {
        PX4_WARN("%s\n", reason);  // 打印原因信息，作为警告信息输出
    }

    PRINT_MODULE_DESCRIPTION(  // 打印模块的描述信息
        R"DESCR_STR(
        ### Description
        test_app
        )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("test_app", "test");  // 打印模块的名称和测试类别
    PRINT_MODULE_USAGE_COMMAND("start");  // 打印模块的启动命令
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();  // 打印默认的模块命令选项

    return 0;  // 返回 0，表示函数执行成功
}

// 定义 test_app_main 函数，这是程序的入口函数
int test_app_main(int argc, char *argv[])
{
    return TestApp::main(argc, argv);  // 调用 TestApp 类的 main 函数，开始应用程序的执行
}
