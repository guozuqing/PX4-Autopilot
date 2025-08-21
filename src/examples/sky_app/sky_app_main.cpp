#include "sky_app_main.h"             // 模块头文件，声明 TestApp 类和接口
#include <drivers/drv_hrt.h>          // 高精度定时器，获取系统时间
#include <uORB/Publication.hpp>       // uORB 发布模板
#include <uORB/Subscription.hpp>      // uORB 订阅模板
#include <uORB/topics/parameter_update.h>  // 监听参数更新消息
#include <uORB/topics/debug_value.h>        // 调试数据发布消息

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);
// C 语言接口，模块入口函数，供 NuttX 调用

uORB::Subscription parameter_update_sub{ORB_ID(parameter_update)};
// 创建订阅对象，监听参数更新主题

// 构造函数，初始化 ModuleParams 基类（传nullptr表示默认参数存储）
TestApp::TestApp() : ModuleParams(nullptr) {}

// 参数更新检测和刷新
void TestApp::parameters_update(bool force) {
    // 检查是否有新的参数更新消息
    bool params_updated = parameter_update_sub.updated();

    // 如果检测到参数变化或者强制更新，则刷新参数缓存
    if (params_updated || force) {
        updateParams();  // 调用基类方法，更新所有绑定的参数
    }
}

// 主循环，定期发布调试参数
void TestApp::run() {
    // 创建调试值发布对象，绑定debug_value主题
    uORB::Publication<debug_value_s> dbg_pub(ORB_ID(debug_value));
    debug_value_s dbg = {};  // 初始化消息结构体

    // 循环直到收到退出信号
    while (!should_exit()) {
        parameters_update();                // 同步参数值缓存
        dbg.timestamp = hrt_absolute_time();  // 时间戳，微秒计数
        dbg.ind = 0;                       // 0表示单个调试值
        dbg.value = value_float.get();     // 读取缓存浮点参数值
        dbg_pub.publish(dbg);              // 发布调试消息
        px4_sleep(1);                     // 休眠1秒，控制发布频率
    }
}

// 任务创建函数，启动线程执行模块逻辑
int TestApp::task_spawn(int argc, char *argv[]) {
    _task_id = px4_task_spawn_cmd(
        "test_app",               // 线程名
        SCHED_DEFAULT,            // 调度策略（默认）
        SCHED_PRIORITY_MIN + 5,  // 优先级
        1024,                    // 栈大小（字节）
        (px4_main_t)&run_trampoline,  // 线程入口（静态中转函数）
        (char *const *)argv
    );

    if (_task_id < 0) {
        _task_id = -1;
        return PX4_ERROR;  // 创建失败返回错误
    }

    return 0;  // 成功
}

// 实例化模块对象
TestApp *TestApp::instantiate(int argc, char *argv[]) {
    TestApp *instance = new TestApp();  // new 动态分配内存
    if (!instance) {
        PX4_ERR("alloc failed");  // 内存分配失败日志
        return nullptr;
    }
    return instance;
}

// 自定义命令处理（这里未实现任何命令）
int TestApp::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command!");  // 打印帮助提示
}

// 帮助信息输出
int TestApp::print_usage(const char *reason) {
    if (reason) PX4_WARN("%s\n", reason);

    // 模块描述
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
test_app：演示设备文件读写的 PX4 模块
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("test_app", "test");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// 模块主入口
int test_app_main(int argc, char *argv[]) {
    return TestApp::main(argc, argv);  // 委托 ModuleBase 处理命令和任务管理
}
