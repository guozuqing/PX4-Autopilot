#include "sky_app_main.h"   // 引入 SkyApp 类的头文件，包含类声明和必要的 PX4 API 定义

// 声明 C 风格接口函数，作为 PX4 模块入口
// PX4 在启动模块时会调用这个函数，argv/argc 传递模块启动参数
extern "C" __EXPORT int sky_app_main(int argc, char *argv[]);

// 使用 PX4 定义的时间字面量命名空间（例如 1_s、500_ms 这样的单位写法）

// ====================== 构造与析构 ======================

// 构造函数
// SkyApp 类继承了 px4::ScheduledWorkItem（PX4 的工作队列任务类）
// 在初始化列表中调用父类构造函数，传入模块名和工作队列配置
SkyApp::SkyApp()
    : px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::sky_app)
{}



// ====================== 初始化函数 ======================

// 初始化函数
// 用于注册到工作队列，并设置任务调度周期
void SkyApp::init()
{
    // ScheduleOnInterval(首次延迟, 周期间隔)
    // 参数单位是微秒(us)
    // 这里：首次延迟 = 5000000us (5秒)，周期 = 5000000us (5秒)
    ScheduleOnInterval(500000, 500000);
    PX4_INFO("SkyApp init");
}

// ====================== 实例化函数 ======================

// 用于创建 SkyApp 类的实例
// 参数 argc/argv：模块启动参数
SkyApp *SkyApp::instantiate(int argc, char *argv[])
{
    // 创建对象实例
    SkyApp *instance = new SkyApp();

    // 判断是否创建成功
    if (instance == nullptr) {
        PX4_ERR("alloc failed");  // 内存分配失败
        return nullptr;           // 返回空指针
    }

    // 返回实例指针
    return instance;
}

// ====================== 任务启动函数 ======================

// 定义 SkyApp 类的 task_spawn 静态成员函数
// 用于创建 SkyApp 类的实例并启动工作队列任务
int SkyApp::task_spawn(int argc, char *argv[])
{
    SkyApp *instance = instantiate(argc - 1, argv + 1);  // 创建 SkyApp 类的实例，调整参数（去掉第一个参数），instantiate这个函数在下面定义了。
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

// ====================== 自定义命令处理 ======================

// 处理自定义命令（start/stop/status 之外的命令）
// 这里简单返回 usage 提示
int SkyApp::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

// ====================== 使用说明 ======================

// 打印模块使用说明
// 定义 SkyApp 类的 print_usage 成员函数
// 用于打印模块的使用说明和相关信息
int SkyApp::print_usage(const char *reason)
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

// ====================== 工作队列任务函数 ======================

// 定义 SkyApp 类的 Run 成员函数
// 当工作队列调度到该任务时，会调用这个函数来执行相应的任务逻辑
void SkyApp::Run()
{
    PX4_INFO("This is a PX4 work queue test app!");  // 打印信息，表明这是 PX4 工作队列测试应用程序
    px4_usleep(1000000);
    PX4_INFO("SkyApp Run 2");

}




// ====================== 模块主入口 ======================

// PX4 模块入口函数
// PX4 框架在命令行运行 `sky_app` 模块时，会调用这里
int sky_app_main(int argc, char *argv[])
{
    return SkyApp::main(argc, argv); // 调用 PX4ModuleBase 提供的 main() 入口
}
