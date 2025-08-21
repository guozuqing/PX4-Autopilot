#pragma once  // 防止头文件重复包含，确保同一头文件在编译单元中仅处理一次

#include <px4_platform_common/module.h>  // 引入 ModuleBase 模板类，同时隐含包含 tasks、log、time 等常用头文件
#include <px4_platform_common/posix.h>   // 引入线程操作相关的头文件，提供线程创建、属性管理等接口
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
/**
 * @class SkyApp
 * @brief 多任务并发示例应用
 *
 * 演示如何使用PX4模块系统创建多个并发任务，并使用信号量保护共享资源。
 * 该应用创建两个子任务，它们并发地对共享计数器进行累加操作。
 */
class SkyApp : public ModuleBase<SkyApp>,public px4::ScheduledWorkItem
{
public:
    /**
     * @brief 构造函数
     * 初始化SkyApp实例
     */
    SkyApp();

    /**
     * @brief 析构函数
     * 清理资源，确保任务正确停止
     */
     virtual ~SkyApp() = default;

    // ========== ModuleBase 必需的静态方法 ==========

    /**
     * @brief 创建并启动任务
     * @param argc 命令行参数数量
     * @param argv 命令行参数数组
     * @return 成功返回0，失败返回PX4_ERROR
     */
    static int task_spawn(int argc, char *argv[]);

    /**
     * @brief 创建SkyApp实例
     * @param argc 命令行参数数量
     * @param argv 命令行参数数组
     * @return 成功返回实例指针，失败返回nullptr
     */
    static SkyApp *instantiate(int argc, char *argv[]);

    /**
     * @brief 处理自定义命令
     * @param argc 命令行参数数量
     * @param argv 命令行参数数组
     * @return 处理结果
     */
    static int custom_command(int argc, char *argv[]);

    /**
     * @brief 打印使用说明
     * @param reason 错误原因（可选）
     * @return 始终返回0
     */
    static int print_usage(const char *reason = nullptr);

    // ========== 核心运行方法 ==========

    /**
     * @brief 主任务运行函数
     * 监控子任务状态并定期报告进度
     */
    void Run() override;



    void init();

};
