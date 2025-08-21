#include "test_app_main.h"               // 引入头文件
#include <px4_platform_common/posix.h>   // 用于px4_open/read/write等文件操作
#define DEV_PATH "/dev/testCDev"         // 设备路径定义

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);  // 导出模块主入口
TestApp::TestApp() {}  // 空构造

//核心运行逻辑
void TestApp::run() {
    int fd = px4_open(DEV_PATH, 0);  // 打开设备（标志位0为示例简化，实际需指定读写模式）
    char rbuf[50];                   // 读缓冲区（50 字节）
    char wbuf[] = "This is write test! \0";  // 写数据（带结束符）
    while (!should_exit()) {  // 检查退出信号（ModuleBase 提供）
        unsigned int rlen = px4_read(fd, rbuf, 50);  // 读取数据，返回实际长度
        PX4_INFO("Read buflen is: %u, and content is: %s", rlen, rbuf);  // 打印读取信息

        px4_write(fd, wbuf, strlen(wbuf));  // 写入数据，长度为字符串有效长度
        px4_sleep(1);  // 休眠 1 秒，控制循环周期
    }
    px4_close(fd);  // 关闭设备
}

//任务创建
int TestApp::task_spawn(int argc, char *argv[]) {
    // 创建任务：名称 test_app，默认调度策略，优先级最低+5，栈 1024 字节，中转函数 run_trampoline
    _task_id = px4_task_spawn_cmd(
        "test_app",
        SCHED_DEFAULT,
        SCHED_PRIORITY_MIN + 5,
        1024,
        (px4_main_t)&run_trampoline,  // 静态中转函数，调用成员 run
        (char *const *)argv
    );
    if (_task_id < 0) {  // 任务创建失败
        _task_id = -1;
        return PX4_ERROR;
    }
    return 0;
}

//实例化
TestApp *TestApp::instantiate(int argc, char *argv[]) {
    TestApp *instance = new TestApp();  // 动态创建对象
    if (instance == nullptr) {
        PX4_ERR("alloc failed");  // 内存分配失败日志
        return nullptr;
    }
    return instance;
}

//自定义命令
int TestApp::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command!");  // 未知命令时打印帮助
}

//帮助信息
int TestApp::print_usage(const char *reason) {
    if (reason) {
	PX4_WARN("%s\n", reason);
}  // 打印错误原因

    // 模块描述（原始字符串字面量，避免转义）
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
test_app：演示设备文件读写的 PX4 模块
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("test_app", "test");  // 模块名与别名
    PRINT_MODULE_USAGE_COMMAND("start");          // 支持的命令（如 start）
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();        // 默认命令（如 help）
    return 0;
}

//模块主入口
int test_app_main(int argc, char *argv[]) {
    return TestApp::main(argc, argv);  // 委托 ModuleBase 处理命令与任务
}
