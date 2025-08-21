#pragma once  // 防止头文件被重复包含

#include <px4_platform_common/module.h>          // PX4 模块基类 (ModuleBase) 定义
#include <px4_platform_common/module_params.h>   // PX4 参数管理基类 (ModuleParams) 定义

/**
 * @class TestApp
 * @brief 示例应用模块，展示 PX4 模块生命周期管理与参数更新机制
 *
 * 继承：
 * - ModuleBase<TestApp>  : 提供模块初始化、运行、销毁等生命周期管理接口
 * - ModuleParams         : 提供参数自动注册、缓存和更新功能
 */
class TestApp : public ModuleBase<TestApp>, public ModuleParams
{
public:
    TestApp();                        ///< 构造函数
    virtual ~TestApp() = default;     ///< 虚析构函数，保证多态析构安全

    /** @brief 启动任务入口 */
    static int task_spawn(int argc, char *argv[]);

    /** @brief 创建模块实例 */
    static TestApp *instantiate(int argc, char *argv[]);

    /** @brief 处理自定义命令 */
    static int custom_command(int argc, char *argv[]);

    /** @brief 打印帮助信息 */
    static int print_usage(const char *reason = nullptr);

    /** @brief 模块主运行函数（任务循环入口） */
    void run() override;

private:
    /**
     * @brief 更新参数
     * @param force 若为 true，则强制重新加载系统参数（忽略缓存）
     */
    void parameters_update(bool force = false);

    /**
     * @brief 模块参数定义
     *
     * DEFINE_PARAMETERS 宏会：
     * - 创建参数对象实例（ParamInt / ParamFloat 等）
     * - 在构造函数中注册参数
     * - 自动生成 updateParamsImpl() 用于参数更新回调
     */
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::TEST_APP_INT>)   value_int,   ///< INT32 类型参数 TEST_APP_INT
        (ParamFloat<px4::params::TEST_APP_FLOAT>) value_float ///< FLOAT 类型参数 TEST_APP_FLOAT
    );

// protected:
//     /**
//      * @brief 参数更新回调
//      *
//      * 每次 parameter_update 主题触发时调用，更新类内缓存值。
//      */
//     void updateParamsImpl() final
//     {
//         value_int.update();    ///< 封装 param_get()，更新缓存
//         value_float.update();  ///< 封装 param_get()，更新缓存
//     }
 };
