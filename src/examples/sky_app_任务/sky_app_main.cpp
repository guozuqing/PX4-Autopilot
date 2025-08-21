#include "sky_app_main.h"

extern "C" __EXPORT int sky_app_main(int argc, char *argv[]);  // 导出模块主入口
using namespace time_literals;             // 使用时间字面量命名空间(如1_s, 1_ms等)
SkyApp::SkyApp() {}  // 空构造



SkyApp *SkyApp::instantiate(int argc, char *argv[]) {

	SkyApp *instance = new SkyApp();
	if (instance == nullptr)  // 如果创建实例失败
	{
	    PX4_ERR("alloc failed");  // 打印错误信息，表明内存分配失败
	    return nullptr;  // 返回空指针
	}
	return instance;  // 返回创建成功的实例指针
}




int SkyApp::task_spawn(int argc, char *argv[]) {
        // 创建任务：名称 test_app，默认调度策略，优先级最低+5，栈 1024 字节，中转函数 run_trampoline
	SkyApp::_task_id = px4_task_spawn_cmd(
		"sky_app",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MIN + 5,
		1024,
		(px4_main_t)&SkyApp::run_trampoline,  // 静态中转函数，调用成员 run
		(char *const *)argv
	    );
	    if (SkyApp::_task_id < 0) {  // 任务创建失败
		SkyApp::_task_id = -1;
		return PX4_ERROR;
	    }
	    return 0;
	}


int SkyApp::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command!");  // 未知命令时打印帮助
}

int SkyApp::print_usage(const char *reason) {
	PX4_INFO("missing command: %s\n", reason);
    return 0;
}


void SkyApp::run() {
        // 输出启动信息
	PX4_INFO("Hello from sky_app");

	// 主循环，直到收到退出信号
	while (!should_exit()) {
	    // 获取当前绝对时间（微秒）
	    hrt_abstime now = hrt_absolute_time();

	    // 输出当前时间（转换为秒）
	    PX4_INFO("Hello from sky_app %u", (unsigned int)(now/1_s));

	    // 计算从获取时间到现在经过的时间
	    hrt_abstime time_count = hrt_elapsed_time(&now);
	    PX4_INFO("Time count %u", (unsigned int)time_count);

	    // 休眠剩余时间，保证总周期为1秒
	    px4_usleep(1_s - time_count);
	}
	return;
}




int sky_app_main(int argc, char *argv[]) {
    return SkyApp::main(argc, argv);
}
