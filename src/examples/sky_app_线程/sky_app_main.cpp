#include "sky_app_main.h"

extern "C" __EXPORT int sky_app_main(int argc, char *argv[]);  // 导出模块主入口
using namespace time_literals;             // 使用时间字面量命名空间(如1_s, 1_ms等)

// 静态初始化互斥锁和条件变量，互斥锁用于保护共享资源，条件变量用于线程同步
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
/**
 * @brief 构造函数
 * 初始化SkyApp实例，设置初始状态
 */
SkyApp::SkyApp() {}

/**
 * @brief 析构函数
 * 清理资源，确保任务正确停止
 */
SkyApp::~SkyApp() {
    cleanup_resources();
}



/**
 * @brief 创建SkyApp实例
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 成功返回实例指针，失败返回nullptr
 */

 SkyApp *SkyApp::instantiate(int argc, char *argv[]) {

	SkyApp *instance = new SkyApp();
	if (instance == nullptr)  // 如果创建实例失败
	{
	    PX4_ERR("alloc failed");  // 打印错误信息，表明内存分配失败
	    return nullptr;  // 返回空指针
	}
	return instance;  // 返回创建成功的实例指针
}

/**
 * @brief 任务2执行函数
 * 执行累加操作，与任务3竞争访问共享计数器
 * @param argc 命令行参数数量（未使用）
 * @param argv 命令行参数数组（未使用）
 * @return 始终返回0表示成功
 */
void* SkyApp::run2(void *arg) {
	SkyApp *instance = SkyApp::_object.load();
	px4_prctl(PR_SET_NAME, "test_thread1", px4_getpid());  // 将当前线程的名称设置为 "test_thread1"，px4_getpid 用于获取当前进程ID
	while (!instance->should_exit())  // 当任务不应退出时，持续循环执行

	{
	    PX4_INFO("thread1 is running!");
	    pthread_cond_broadcast(&cond);   //唤醒所有等待该条件变量的线程（这里主要是唤醒线程2）
	    px4_sleep(1);  // 休眠1秒（实现每隔1s执行）
	}
	return nullptr;

    }


/**
 * @brief 任务3执行函数
 * 执行累加操作，与任务2竞争访问共享计数器
 * @param argc 命令行参数数量（未使用）
 * @param argv 命令行参数数组（未使用）
 * @return 始终返回0表示成功
 */
void* SkyApp::run3(void *arg)
	{	SkyApp *instance = SkyApp::_object.load();
		px4_prctl(PR_SET_NAME, "test_thread2", px4_getpid());  //// 设置线程名称为test_thread2
		while (!instance->should_exit())  // 当任务不应退出时循环
		{
		    pthread_mutex_lock(&mutex);  // 对互斥锁加锁，保护共享资源（这里虽未操作共享资源，但遵循条件变量使用规范）
		    pthread_cond_wait(&cond, &mutex);  // 线程2阻塞等待线程1的通知，等待条件变量，同时解锁mutex。当被唤醒时，会重新加锁mutex
		    pthread_mutex_unlock(&mutex);  // 解锁互斥锁
		    PX4_INFO("thread2 is running!");
		}
		return nullptr;

	    }


/**
 * @brief 创建并启动任务
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 成功返回0，失败返回PX4_ERROR
 */
int SkyApp::task_spawn(int argc, char *argv[]) {
	SkyApp::_task_id = px4_task_spawn_cmd("test_app", SCHED_DEFAULT, SCHED_PRIORITY_MIN + 5, 1024, (px4_main_t)&run_trampoline, (char *const *)argv);  // 创建任务，指定任务名、调度策略、优先级、堆栈大小、执行函数和参数
	if (SkyApp::_task_id < 0)  // 任务创建失败处理
	{
	    SkyApp::_task_id = -1;
	    return -1;
	}
	return 0;
    }

/**
 * @brief 处理自定义命令
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 处理结果
 */
int SkyApp::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

/**
 * @brief 打印使用说明
 * @param reason 错误原因（可选）
 * @return 始终返回0
 */


int SkyApp::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PX4_INFO("Usage: sky_app <command>");
    PX4_INFO("Commands:");
    PX4_INFO("  start    - Start the sky_app tasks");
    PX4_INFO("  stop     - Stop the sky_app tasks");
    PX4_INFO("  status   - Show task status");

    return 0;
}


/**
 * @brief 主任务运行函数
 * 监控子任务状态并定期报告进度
 */
void SkyApp::run() {
	PX4_INFO("Hello Sky!");
	pthread_t pth1, pth2;  // 定义两个线程标识符，用于存储线程ID
	pthread_attr_t pth_attr; // 定义线程属性对象，用于设置线程的堆栈大小、优先级等属性
	pthread_attr_init(&pth_attr);  // 初始化线程属性对象
	pthread_attr_setstacksize(&pth_attr, 64);  // 设置线程堆栈大小为64字节
	struct sched_param param;  // 定义调度参数结构，用于设置线程优先级
	pthread_attr_getschedparam(&pth_attr, &param);  // 获取线程当前的调度参数到param
	param.sched_priority = SCHED_PRIORITY_MIN + 5;  // 设置线程优先级为最低优先级加5
	pthread_attr_setschedparam(&pth_attr, &param);   // 将设置好的调度参数应用到线程属性中
	pthread_create(&pth1, &pth_attr, run2, nullptr);  // 创建线程1，指定线程属性、执行函数和参数
	pthread_create(&pth2, &pth_attr, run3, nullptr);  // 创建线程2
	pthread_attr_destroy(&pth_attr);  // 销毁线程属性对象，释放相关资源
	pthread_join(pth1, nullptr);  // 等待线程1执行完毕
	pthread_join(pth2, nullptr);  // 等待线程2执行完毕
    }





/**
 * @brief 清理子任务
 */
void SkyApp::cleanup_tasks() {
    pthread_cond_destroy(&cond);  // 销毁条件变量
    pthread_mutex_destroy(&mutex);  // 销毁互斥锁
}

/**
 * @brief 清理所有资源
 */
void SkyApp::cleanup_resources() {
    cleanup_tasks();
    pthread_mutex_destroy(&mutex);  // 销毁互斥锁
}

/**
 * @brief 重写状态打印函数
 * @return 成功返回0
 */
int SkyApp::print_status() {
    PX4_INFO("SkyApp Status:");
    PX4_INFO("  Main task ID: %d",SkyApp::_task_id);


    return 0;
}

/**
 * @brief 模块主入口函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 执行结果
 */
int sky_app_main(int argc, char *argv[]) {
    return SkyApp::main(argc, argv);
}
