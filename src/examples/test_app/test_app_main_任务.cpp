/**
 * @file test_app_main.cpp
 * @brief PX4测试应用程序示例
 *
 * 这是一个简单的PX4应用程序示例，演示了如何：
 * 1. 创建和管理任务
 * 2. 处理命令行参数
 * 3. 使用高分辨率定时器
 * 4. 实现基本的启动/停止/状态查询功能
 *
 * 用法：
 *   test_app start   - 启动应用
 *   test_app stop    - 停止应用
 *   test_app status  - 查看状态
 *   test_app -h      - 显示帮助
 *
 * @author PX4 Development Team
 */

// 包含必要的头文件
#include <string.h>                        // 字符串操作函数
#include <px4_log.h>                       // PX4日志系统
#include <px4_platform_common/tasks.h>     // PX4任务管理
#include <drivers/drv_hrt.h>               // 高分辨率定时器

// 导出主函数，使其可以从外部调用
extern "C" __EXPORT int test_app_main(int argc, char *argv[]);
using namespace time_literals;             // 使用时间字面量命名空间(如1_s, 1_ms等)

// 全局变量定义
static px4_task_t _task_id = -1;           // 任务ID，-1表示未运行
static bool _task_should_exit = false;     // 任务退出标志

/**
 * 检查应用程序是否正在运行
 * @return true: 正在运行, false: 未运行
 */
bool is_running()
{
    return _task_id != -1;  // 通过任务ID判断是否运行
}

/**
 * 应用程序主要运行函数
 * 在独立任务中执行，周期性输出时间信息
 */
void run(void)
{
    // 输出启动信息
    PX4_INFO("Hello from test_app");

    // 主循环，直到收到退出信号
    while (!_task_should_exit) {
        // 获取当前绝对时间（微秒）
        hrt_abstime now = hrt_absolute_time();

        // 输出当前时间（转换为秒）
        PX4_INFO("Hello from test_app %u", (unsigned int)(now/1_s));

        // 计算从获取时间到现在经过的时间
        hrt_abstime time_count = hrt_elapsed_time(&now);
        PX4_INFO("Time count %u", (unsigned int)time_count);

        // 休眠剩余时间，保证总周期为1秒
        px4_usleep(1_s - time_count);
    }
}


/**
 * 任务跳板函数
 * 用于在新任务中启动run函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return PX4_OK
 */
int run_trampoline(int argc, char *argv[])
{
	_task_should_exit = false;  // 清除退出标志
	run();                      // 调用主运行函数
	return PX4_OK;
}

/**
 * 创建并启动任务
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0:成功, PX4_ERROR:失败
 */
int task_spawn(int argc, char *argv[])
{
	// 创建新任务
	// 任务名: "test_app", 默认调度策略, 默认优先级, 栈大小1024字节
	_task_id = px4_task_spawn_cmd("test_app", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024, (px4_main_t)&run_trampoline, (char *const *)argv);

	// 检查任务创建是否成功
	if (_task_id < 0) {
		_task_id = -1;
		return PX4_ERROR;
	}

	PX4_INFO("Test app task spawned with ID %d", _task_id);

	return 0;
}

/**
 * 启动命令处理函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0:成功, -1:已在运行或启动失败
 */
int start_command_base(int argc, char *argv[])
{
	int ret = 0;

	// 检查应用是否已经在运行
	if (is_running()) {
		ret = -1;
		PX4_INFO("Test app is already running");
	} else {
		// 尝试启动任务
		ret = task_spawn(argc, argv);
		if (ret < 0) {
			PX4_ERR("Failed to spawn test app");
		}
	}

	return ret;
}

/**
 * 状态查询命令处理函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
int status_command(int argc, char *argv[])
{
	// 检查并报告应用运行状态
	if (is_running()) {
		PX4_INFO("Test app is running");
	} else {
		PX4_INFO("Test app is not running");
	}

	return 0;
}

/**
 * 停止命令处理函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
int stop_command(int argc, char *argv[])
{
	// 设置退出标志，让主循环退出
	_task_should_exit = true;

	// 等待2秒让任务正常退出
	px4_sleep(2);

	// 重置任务ID
	_task_id = -1;
	PX4_INFO("Test app stopped");
	return 0;
}


/**
 * 自定义命令处理函数（当前仅输出错误信息）
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 0
 */
int custom_command(int argc, char *argv[])
{
	PX4_ERR("unknown command");
	return 0;
}

/**
 * 打印使用帮助信息
 * @param reason 调用原因（当前未使用）
 * @return 0
 */
int print_usage(const char *reason)
{
	PX4_INFO("Usage: test_app {start|stop|status|custom}");
	return 0;
}

/**
 * 测试应用主函数 - 程序入口点
 * 解析命令行参数并调用相应的处理函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 各命令函数的返回值
 */
int test_app_main(int argc, char *argv[])
{
	// 检查是否需要显示帮助信息
	// 条件：无参数、-h、--help、info或拼写错误的"usage"
	if (argc <= 1 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0 ||
		strcmp(argv[1], "info") == 0 || strcmp(argv[1], "heluasge") == 0) {
		return print_usage(nullptr);
	}

	// 处理启动命令
	if (strcmp(argv[1], "start") == 0) {
		return start_command_base(argc - 1, argv + 1);
	}

	// 处理停止命令
	if (strcmp(argv[1], "stop") == 0) {
		return stop_command(argc - 1, argv + 1);
	}

	// 处理状态查询命令
	if (strcmp(argv[1], "status") == 0) {
		return status_command(argc - 1, argv + 1);
	}

	// 处理未识别的命令（调用自定义命令处理函数）
	int ret = custom_command(argc - 1, argv + 1);
	return ret;
}
