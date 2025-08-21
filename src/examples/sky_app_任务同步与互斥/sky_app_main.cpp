#include "sky_app_main.h"

extern "C" __EXPORT int sky_app_main(int argc, char *argv[]);  // 导出模块主入口
using namespace time_literals;             // 使用时间字面量命名空间(如1_s, 1_ms等)

// 静态常量定义
static const unsigned int MAX_CNT = 10000;  // 每个子任务的累加次数上限（优化：减少次数以避免过度占用CPU）
static const unsigned int STACK_SIZE = 2048; // 任务栈大小（优化：增加栈大小以确保安全）

// 共享资源
static unsigned int cnt = 0;               // 共享计数器，两个子任务会对其进行累加操作
static px4_sem_t sem;                      // 信号量，用于保护共享资源cnt
static bool sem_initialized = false;       // 信号量初始化标志

// 任务管理
static px4_task_t task2_id = -1;           // 任务2的ID
static px4_task_t task3_id = -1;           // 任务3的ID

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
    // 初始化信号量（仅初始化一次）
    if (!sem_initialized) {
        if (px4_sem_init(&sem, 0, 1) != 0) {
            PX4_ERR("Failed to initialize semaphore");
            return nullptr;
        }
        sem_initialized = true;
        PX4_INFO("Semaphore initialized successfully");
    }

    SkyApp *instance = new SkyApp();
    if (instance == nullptr) {
        PX4_ERR("Memory allocation failed");
        return nullptr;
    }

    PX4_INFO("SkyApp instance created successfully");
    return instance;
}

/**
 * @brief 任务2执行函数
 * 执行累加操作，与任务3竞争访问共享计数器
 * @param argc 命令行参数数量（未使用）
 * @param argv 命令行参数数组（未使用）
 * @return 始终返回0表示成功
 */
int SkyApp::run2(int argc, char *argv[]) {
    (void)argc; // 明确标记未使用的参数
    (void)argv;

    PX4_INFO("Task 2 started, will increment counter %u times", MAX_CNT);

    // 循环执行MAX_CNT次累加操作
    for (unsigned int i = 0; i < MAX_CNT; ++i) {
        // 申请信号量，进入临界区，确保互斥访问cnt
	px4_sem_wait(&sem);
        // 临界区代码：对共享资源cnt进行读写操作
        unsigned int local_cnt = cnt;  // 读取当前cnt值到局部变量
        ++local_cnt;                   // 局部变量加1
        cnt = local_cnt;               // 将结果写回共享变量

        PX4_INFO("Task 2: iteration %u, cnt = %u", i, cnt);
        // 释放信号量，离开临界区

	px4_usleep(1000); // 每100次操作休眠1ms
	px4_sem_post(&sem);
    }


    PX4_INFO("Task 2 completed, final cnt = %u", cnt);
    return 0; // 返回成功状态
}


/**
 * @brief 任务3执行函数
 * 执行累加操作，与任务2竞争访问共享计数器
 * @param argc 命令行参数数量（未使用）
 * @param argv 命令行参数数组（未使用）
 * @return 始终返回0表示成功
 */
int SkyApp::run3(int argc, char *argv[]) {
    (void)argc; // 明确标记未使用的参数
    (void)argv;

    PX4_INFO("Task 3 started, will increment counter %u times", MAX_CNT);

    // 循环执行MAX_CNT次累加操作
    for (unsigned int i = 0; i < MAX_CNT; ++i) {
        // 申请信号量，进入临界区，确保互斥访问cnt
	px4_sem_wait(&sem);
        // 临界区代码：对共享资源cnt进行读写操作
        unsigned int local_cnt = cnt;  // 读取当前cnt值到局部变量
        ++local_cnt;                   // 局部变量加1
        cnt = local_cnt;               // 将结果写回共享变量
        // 释放信号量，离开临界区

	px4_usleep(1000); // 每100次操作休眠1ms
	PX4_INFO("Task 3: iteration %u, cnt = %u", i, cnt);
	px4_sem_post(&sem);
    }


    PX4_INFO("Task 3 completed, final cnt = %u", cnt);
    return 0; // 返回成功状态
}


/**
 * @brief 创建并启动任务
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 成功返回0，失败返回PX4_ERROR
 */
int SkyApp::task_spawn(int argc, char *argv[]) {
    PX4_INFO("Starting SkyApp tasks...");
    px4_sem_init(&sem, 0, 1);
    // 重置共享计数器
    cnt = 0;

    // 创建任务2
    task2_id = px4_task_spawn_cmd(
        "sky_app_task2",           // 任务名称
        SCHED_DEFAULT,             // 默认调度策略
        SCHED_PRIORITY_MIN + 6,    // 优先级（比主任务稍高）
        STACK_SIZE,                // 栈大小
        (px4_main_t)SkyApp::run2,  // 任务入口函数
        (char *const *)argv        // 参数
    );

    if (task2_id < 0) {
        PX4_ERR("Failed to create task 2");
        return PX4_ERROR;
    }

    PX4_INFO("Task 2 created with ID: %d", task2_id);

    // 创建任务3
    task3_id = px4_task_spawn_cmd(
        "sky_app_task3",           // 任务名称
        SCHED_DEFAULT,             // 默认调度策略
        SCHED_PRIORITY_MIN + 6,    // 优先级（比主任务稍高）
        STACK_SIZE,                // 栈大小
        (px4_main_t)SkyApp::run3,  // 任务入口函数
        (char *const *)argv        // 参数
    );

    if (task3_id < 0) {
        PX4_ERR("Failed to create task 3");
        // 清理已创建的任务2
        if (task2_id > 0) {
            px4_task_delete(task2_id);
            task2_id = -1;
        }
        return PX4_ERROR;
    }

    PX4_INFO("Task 3 created with ID: %d", task3_id);

        PX4_INFO("All tasks created successfully");
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
    PX4_INFO("SkyApp main task started");
    PX4_INFO("Monitoring tasks: task2_id=%d, task3_id=%d", task2_id, task3_id);

    unsigned int loop_count = 0;
    const unsigned int REPORT_INTERVAL = 5; // 每5秒报告一次状态

    // 主监控循环
    while (!should_exit()) {
        hrt_abstime loop_start = hrt_absolute_time();

        // 每隔一定时间报告状态
        if (loop_count % REPORT_INTERVAL == 0) {
            px4_sem_wait(&sem); // 安全访问共享计数器
            unsigned int current_cnt = cnt;
            px4_sem_post(&sem);

            PX4_INFO("SkyApp status - Loop: %u, Shared counter: %u, Time: %u s",
                    loop_count, current_cnt, (unsigned int)(loop_start / 1_s));

            // 检查任务是否仍在运行（简单检查）
            if (task2_id > 0 && task3_id > 0) {
                PX4_INFO("Both tasks are active");
            }
        }

        loop_count++;

        // 计算循环耗时并休眠剩余时间，保证1秒周期
        hrt_abstime elapsed = hrt_elapsed_time(&loop_start);
        hrt_abstime sleep_time = (elapsed < 1_s) ? (1_s - elapsed) : 0;

        if (sleep_time > 0) {
            px4_usleep(sleep_time);
        }
    }

    PX4_INFO("SkyApp main task stopping...");
    cleanup_tasks();
}




/**
 * @brief 清理子任务
 */
void SkyApp::cleanup_tasks() {
    if (task2_id > 0) {
        PX4_INFO("Stopping task 2 (ID: %d)", task2_id);
        px4_task_delete(task2_id);
        task2_id = -1;
    }

    if (task3_id > 0) {
        PX4_INFO("Stopping task 3 (ID: %d)", task3_id);
        px4_task_delete(task3_id);
        task3_id = -1;
    }
}

/**
 * @brief 清理所有资源
 */
void SkyApp::cleanup_resources() {
    cleanup_tasks();

    if (sem_initialized) {
        px4_sem_destroy(&sem);
        sem_initialized = false;
        PX4_INFO("Semaphore destroyed");
    }
}

/**
 * @brief 重写状态打印函数
 * @return 成功返回0
 */
int SkyApp::print_status() {
    px4_sem_wait(&sem);
    unsigned int current_cnt = cnt;
    px4_sem_post(&sem);

    PX4_INFO("SkyApp Status:");
    PX4_INFO("  Main task ID: %d", _task_id);
    PX4_INFO("  Task 2 ID: %d", task2_id);
    PX4_INFO("  Task 3 ID: %d", task3_id);
    PX4_INFO("  Shared counter: %u", current_cnt);
    PX4_INFO("  Semaphore initialized: %s", sem_initialized ? "Yes" : "No");

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
