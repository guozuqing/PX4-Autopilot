#include "test_app_main.h"               // 引入自定义头文件，包含TestApp类等声明
#include <px4_platform_common/posix.h>   // PX4 POSIX兼容层，提供线程、睡眠等接口
#include <drivers/drv_hrt.h>             // 高分辨率定时器头文件，用于获取时间戳
#include <uORB/topics/test_topic.h>      // uORB自动生成的主题头文件（对应test_topic.msg）
#include <inttypes.h>

static bool thread_should_exit = false;  // 线程退出标志，控制发送/接收线程循环
extern "C" __EXPORT int test_app_main(int argc, char *argv[]); // 应用入口，extern "C"保证C++符号兼容，__EXPORT导出供系统调用

//线程pth_send：模拟每秒发布uORB主题，步骤：公告→填数据→发布（公告仅1次）
void *thread_loop_send(void *arg) {      // 发送线程函数，arg为线程创建时的入参（此处未用）
    test_topic_s test_data = {};         // 初始化test_topic_s结构体（由.msg生成，字段与msg定义对应）
    // 公告主题实例：ORB_ID(test_topic_x)指定主题，&test_data传初始数据，返回公告句柄
    orb_advert_t test_pub = orb_advertise(ORB_ID(test_topic_x), &test_data);

    uint64_t cnt = 0;                    // 计数器，用于填充data字段
    while (!thread_should_exit) {        // 未收到退出信号时持续发布
        test_data.timestamp = hrt_absolute_time(); // 填充时间戳：获取当前高分辨率时间
        test_data.data = cnt;            // 填充数据：将计数器值存入data字段
        // 发布数据：通过公告句柄test_pub，将test_data发布到test_topic_x主题
        orb_publish(ORB_ID(test_topic_x), test_pub, &test_data);
        ++cnt;                           // 计数器自增
        px4_sleep(0.1);                    // 睡眠1秒，模拟1Hz发布频率
    }
    return nullptr;                      // 线程退出，返回空指针
}

// 线程pth_receive：模拟每秒接收uORB主题，步骤：订阅→检查更新→拷贝数据（订阅仅1次）
void *thread_loop_receive(void *arg) {   // 接收线程函数，arg为线程创建时的入参（此处未用）
    // 订阅主题test_topic_x，返回订阅句柄test_sub
    int test_sub = orb_subscribe(ORB_ID(test_topic_x));
    bool updated;                        // 存储“数据是否更新”的标志
    test_topic_s test_data = {};         // 初始化接收缓冲区，用于存储拷贝的数据
    while (!thread_should_exit) {        // 未收到退出信号时持续接收
        orb_check(test_sub, &updated);   // 检查数据是否更新，结果存入updated
        if (updated) {                   // 若有新数据
            // 拷贝数据：从test_topic_x主题，通过订阅句柄test_sub，将数据拷贝到test_data
            orb_copy(ORB_ID(test_topic_x), test_sub, &test_data);
            // 打印时间戳（转换为秒，除以1e6）
            PX4_INFO("uorb topic send time is %" PRIu64 " s", test_data.timestamp / 1000000u);

            // 打印接收到的cnt值
            PX4_INFO("test data is cnt = %" PRIu64, test_data.data);
        }
        px4_sleep(1);                    // 睡眠1秒，模拟1Hz检查频率
    }
    return nullptr;                      // 线程退出，返回空指针
}

TestApp::TestApp() {}                    // TestApp类构造函数

void TestApp::run() {                    // TestApp类的运行方法，启动收发线程
    PX4_INFO("start!");                  // 打印启动信息

    pthread_attr_t pth_attr;             // 线程属性结构体，用于配置线程参数
    pthread_attr_init(&pth_attr);        // 初始化线程属性
    pthread_attr_setstacksize(&pth_attr, 64); // 设置线程栈大小（示例值，实际需合理配置）

    struct sched_param param;            // 调度参数结构体，用于设置线程优先级
    // 获取当前线程属性的调度参数（此处为pth_attr的默认参数）
    pthread_attr_getschedparam(&pth_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MIN + 5; // 设置优先级：最小优先级+5
    pthread_attr_setschedparam(&pth_attr, &param); // 应用调度参数到线程属性

    pthread_t pth_send;                  // 发送线程ID
    // 创建发送线程：使用pth_attr属性，执行thread_loop_send，入参nullptr
    pthread_create(&pth_send, &pth_attr, thread_loop_send, nullptr);

    pthread_t pth_receive;               // 接收线程ID
    // 创建接收线程：使用pth_attr属性，执行thread_loop_receive，入参nullptr
    pthread_create(&pth_receive, &pth_attr, thread_loop_receive, nullptr);

    pthread_attr_destroy(&pth_attr);     // 销毁线程属性结构体（不再使用）

    // 主线程循环：等待退出信号（should_exit()需实现，此处逻辑为持续睡眠）
    while (!should_exit()) {
        px4_sleep(1);                    // 每秒检查一次退出状态
    }

    thread_should_exit = true;           // 设置退出标志，通知收发线程退出
    pthread_join(pth_send, nullptr);     // 等待发送线程退出
    pthread_join(pth_receive, nullptr);  // 等待接收线程退出
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
    if (!instance) {
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
    if (reason) PX4_WARN("%s\n", reason);  // 打印错误原因

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
