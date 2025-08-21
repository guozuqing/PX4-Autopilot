#include <string.h>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>          // 引入线程操作相关的头文件，提供线程创建、属性管理等接口
#include <drivers/drv_hrt.h>
using namespace time_literals;

extern "C" __EXPORT int test_app_main(int argc, char *argv[]);

static px4_task_t task_id = -1;
static bool _task_should_exit = false;
// 静态初始化互斥锁和条件变量，互斥锁用于保护共享资源，条件变量用于线程同步
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

bool is_running()
{
    return task_id != -1;
}

//void*是通用指针类型，可以指向任意数据类型（如int*、struct*等）。
// 线程1的执行函数：每隔1秒打印信息并通知线程2
void *thread_loop1(void *arg)
{
    px4_prctl(PR_SET_NAME, "test_thread1", px4_getpid());  // 将当前线程的名称设置为 "test_thread1"，px4_getpid 用于获取当前进程ID
    while (!_task_should_exit)  // 当任务不应退出时，持续循环执行

    {
        PX4_INFO("thread1 is running!");
        pthread_cond_broadcast(&cond);   //唤醒所有等待该条件变量的线程（这里主要是唤醒线程2）
        px4_sleep(1);  // 休眠1秒（实现每隔1s执行）
    }
    return nullptr;    // 线程执行完毕返回
}

// 线程2的执行函数：阻塞等待线程1的通知
void *thread_loop2(void *arg)
{
    px4_prctl(PR_SET_NAME, "test_thread2", px4_getpid());  //// 设置线程名称为test_thread2
    while (!_task_should_exit)   // 当任务不应退出时循环
    {
        pthread_mutex_lock(&mutex);  // 对互斥锁加锁，保护共享资源（这里虽未操作共享资源，但遵循条件变量使用规范）
        pthread_cond_wait(&cond, &mutex);  // 线程2阻塞等待线程1的通知，等待条件变量，同时解锁mutex。当被唤醒时，会重新加锁mutex
        pthread_mutex_unlock(&mutex);  // 解锁互斥锁
        PX4_INFO("thread2 is running!");
    }
    return nullptr;  // 线程执行完毕返回
}

void run()
{
    PX4_INFO("Hello Sky!");
    pthread_t pth1, pth2;  // 定义两个线程标识符，用于存储线程ID
    pthread_attr_t pth_attr; // 定义线程属性对象，用于设置线程的堆栈大小、优先级等属性
    pthread_attr_init(&pth_attr);  // 初始化线程属性对象
    pthread_attr_setstacksize(&pth_attr, 64);  // 设置线程堆栈大小为64字节
    struct sched_param param;  // 定义调度参数结构，用于设置线程优先级
    pthread_attr_getschedparam(&pth_attr, &param);  // 获取线程当前的调度参数到param
    param.sched_priority = SCHED_PRIORITY_MIN + 5;  // 设置线程优先级为最低优先级加5
    pthread_attr_setschedparam(&pth_attr, &param);   // 将设置好的调度参数应用到线程属性中
    pthread_create(&pth1, &pth_attr, thread_loop1, nullptr);  // 创建线程1，指定线程属性、执行函数和参数
    pthread_create(&pth2, &pth_attr, thread_loop2, nullptr);  // 创建线程2
    pthread_attr_destroy(&pth_attr);  // 销毁线程属性对象，释放相关资源
    pthread_join(pth1, nullptr);  // 等待线程1执行完毕
    pthread_join(pth2, nullptr);  // 等待线程2执行完毕
}

int run_trampoline(int argc, char *argv[])
{
    _task_should_exit = false;  // 重置任务退出标志

    run(); // 执行主运行函数
    return PX4_OK;
}

int task_spawn(int argc, char *argv[])
{
    task_id = px4_task_spawn_cmd("test_app", SCHED_DEFAULT, SCHED_PRIORITY_MIN + 5, 1024, (px4_main_t)&run_trampoline, (char *const *)argv);  // 创建任务，指定任务名、调度策略、优先级、堆栈大小、执行函数和参数
    if (task_id < 0)  // 任务创建失败处理
    {
        task_id = -1;
        return -1;
    }
    return 0;
}

int start_command_base(int argc, char *argv[])
{
    int ret = 0;
    if (is_running())
    {
        ret = -1;
        PX4_INFO("Task already running!");
    }
    else
    {
        ret = task_spawn(argc, argv);
        if (ret < 0)
        {
            PX4_ERR("Task start failed (%i)", ret);
        }
    }
    return ret;
}

int status_command()
{
    if (is_running())
    {
        PX4_INFO("app is running!");
    }
    else
    {
        PX4_INFO("app is not running!");
    }
    return 0;
}

int stop_command()
{
    _task_should_exit = true;
    px4_sleep(2);
    task_id = -1;
    PX4_INFO("app stopped!");
    return 0;
}

int custom_command(int argc, char *argv[])
{
    PX4_ERR("unknown command!");
    return 0;
}

int print_usage()
{
    PX4_INFO("This is a test app!");
    return 0;
}

int test_app_main(int argc, char *argv[])
{
    if (argc <= 1 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "help") == 0 || strcmp(argv[1], "info") == 0 || strcmp(argv[1], "usage") == 0)
    {
        return print_usage();
    }
    if (strcmp(argv[1], "start") == 0)
    {
        return start_command_base(argc - 1, argv + 1);
    }
    if (strcmp(argv[1], "status") == 0)
    {
        return status_command();
    }
    if (strcmp(argv[1], "stop") == 0)
    {
        return stop_command();
    }
    int ret = custom_command(argc - 1, argv + 1);
    return ret;
