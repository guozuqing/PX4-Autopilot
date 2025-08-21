#pragma once  // 防止头文件重复包含，确保同一头文件在编译单元中仅处理一次

#include <px4_platform_common/module.h>  // 引入 ModuleBase 模板类，同时隐含包含 tasks、log、time 等常用头文件
#include <px4_platform_common/sem.h>     // 引入信号量支持

/**
 * @class SkyApp
 * @brief 多任务并发示例应用
 *
 * 演示如何使用PX4模块系统创建多个并发任务，并使用信号量保护共享资源。
 * 该应用创建两个子任务，它们并发地对共享计数器进行累加操作。
 */
class SkyApp : public ModuleBase<SkyApp>
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
    virtual ~SkyApp();

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
    void run() override;

    /**
     * @brief 重写状态打印函数
     * @return 成功返回0
     */
    int print_status() override;

    // ========== 子任务方法 ==========

        /**
     * @brief 任务2执行函数
     * 执行累加操作，与任务3竞争访问共享计数器
     * @param argc 命令行参数数量（未使用）
     * @param argv 命令行参数数组（未使用）
     * @return 任务执行结果，成功返回0
     */
    static int run2(int argc, char *argv[]);

    /**
     * @brief 任务3执行函数
     * 执行累加操作，与任务2竞争访问共享计数器
     * @param argc 命令行参数数量（未使用）
     * @param argv 命令行参数数组（未使用）
     * @return 任务执行结果，成功返回0
     */
    static int run3(int argc, char *argv[]);

private:
    // ========== 资源清理方法 ==========

    /**
     * @brief 清理子任务
     */
    static void cleanup_tasks();

    /**
     * @brief 清理所有资源
     */
    static void cleanup_resources();
};
