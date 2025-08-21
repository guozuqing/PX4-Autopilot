/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file module.h
 * @brief PX4模块基类定义文件
 *
 * 本文件定义了PX4模块系统的基础架构，提供了模块生命周期管理的统一接口。
 * 主要包含：
 * - ModuleBase模板基类：为所有PX4模块提供统一的启动、停止、状态查询功能
 * - 模块文档和帮助系统：提供统一的命令行帮助格式
 * - 线程安全机制：确保模块启动和停止过程的线程安全
 */

#pragma once

// 系统头文件
#include <pthread.h>        // POSIX线程支持，用于模块间的互斥锁
#include <unistd.h>         // UNIX标准定义，提供基本系统调用
#include <stdbool.h>        // C99布尔类型支持

// PX4平台通用头文件
#include <px4_platform_common/atomic.h>    // 原子操作支持，确保多线程安全
#include <px4_platform_common/time.h>      // 高精度时间函数
#include <px4_platform_common/log.h>       // PX4日志系统（PX4_INFO, PX4_ERR等）
#include <px4_platform_common/tasks.h>     // PX4任务管理（px4_task_spawn_cmd等）
#include <systemlib/px4_macros.h>          // PX4通用宏定义

#ifdef __cplusplus

#include <cstring>          // C++字符串操作函数

/**
 * @brief 模块启动和关闭过程的全局互斥锁
 *
 * 该互斥锁用于防止模块启动和关闭过程中的竞态条件。
 * 虽然理论上可以为每个模块实例创建单独的互斥锁，但为了减少内存占用，
 * 这里使用单一的全局互斥锁。这种设计是安全的，因为模块的启动通常是顺序进行的，
 * 不会产生严重的锁竞争。
 *
 * 保护的操作包括：
 * - 模块实例的创建和销毁
 * - 任务ID的设置和清除
 * - 模块状态的查询和修改
 */
extern pthread_mutex_t px4_modules_mutex;

/**
 * @class ModuleBase
 * @brief PX4模块基类，为所有模块提供统一的生命周期管理
 *
 * 该基类实现了模块的通用功能，包括'start'、'stop'和'status'命令的处理。
 * 目前不支持允许多实例的模块（如mavlink模块）。
 *
 * 该类使用奇异递归模板模式（CRTP - Curiously Recurring Template Pattern）实现。
 * 这种模式允许基类中拥有针对每个模块类型都不同的静态对象，
 * 并能从基类调用派生类的静态方法。
 *
 * @note 派生类必须实现的方法：
 *
 * 当模块运行在独立线程中时：
 *
 *      static int task_spawn(int argc, char *argv[])
 *      {
 *              // 调用px4_task_spawn_cmd()，使用&run_trampoline作为启动方法
 *              // 可选：等待_object不为null，表示任务已初始化（使用超时机制）
 *              // 设置_task_id并返回0
 *              // 出错时返回!= 0（且_task_id必须为-1）
 *      }
 *
 *      static T *instantiate(int argc, char *argv[])
 *      {
 *              // 此方法在新线程中被run_trampoline()调用
 *              // 解析命令行参数
 *              // 创建新的对象T并返回
 *              // 出错时返回nullptr
 *      }
 *
 *      static int custom_command(int argc, char *argv[])
 *      {
 *              // 支持自定义命令
 *              // 如果不支持任何自定义命令，直接返回：
 *              return print_usage("unrecognized command");
 *      }
 *
 *      static int print_usage(const char *reason = nullptr)
 *      {
 *              // 使用PRINT_MODULE_*系列方法打印帮助信息
 *      }
 *
 * 当模块运行在工作队列中时：
 * - custom_command和print_usage方法同上
 *
 *      static int task_spawn(int argc, char *argv[]) {
 *              // 解析命令行参数
 *              // 设置_object（在此处或从work_queue()回调中设置）
 *              // 调用work_queue()（使用自定义循环跳板函数）
 *              // 可选：等待_object不为null，表示任务已初始化（使用超时机制）
 *              // 设置_task_id为task_id_is_work_queue并返回0
 *              // 出错时返回!= 0（且_task_id必须为-1）
 *      }
 */
template<class T>
class ModuleBase
{
public:
	/**
	 * @brief 构造函数
	 * 初始化模块基类，设置退出标志为false
	 */
	ModuleBase() : _task_should_exit{false} {}

	/**
	 * @brief 虚析构函数
	 * 确保派生类对象能够正确析构
	 */
	virtual ~ModuleBase() {}

	/**
	 * @brief 模块主入口点
	 *
	 * 这是模块的主要入口函数，应该直接从模块的main方法调用。
	 * 该函数解析命令行参数，并根据第一个参数执行相应的操作：
	 * - 无参数或-h/help/info/usage：显示帮助信息
	 * - start：启动模块
	 * - stop：停止模块
	 * - status：查询模块状态
	 * - 其他：调用custom_command处理自定义命令
	 *
	 * @param argc 命令行参数数量
	 * @param argv 命令行参数数组指针
	 * @return 成功返回0，失败返回-1
	 */
	static int main(int argc, char *argv[])
	{
		// 检查是否需要显示帮助信息
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {
			return T::print_usage();  // 调用派生类的帮助信息打印方法
		}

		// 处理start命令
		if (strcmp(argv[1], "start") == 0) {
			// 传递'start'参数，因为后续px4_getopt()会忽略第一个参数
			return start_command_base(argc - 1, argv + 1);
		}

		// 处理status命令
		if (strcmp(argv[1], "status") == 0) {
			return status_command();
		}

		// 处理stop命令
		if (strcmp(argv[1], "stop") == 0) {
			return stop_command();
		}

		// 处理自定义命令
		lock_module(); // 加锁，因为该方法可能访问_object
		int ret = T::custom_command(argc - 1, argv + 1);  // 调用派生类的自定义命令处理方法
		unlock_module();  // 解锁

		return ret;
	}

	/**
	 * @brief 线程跳板函数入口点
	 *
	 * 当模块运行在独立线程中时，这是px4_task_spawn_cmd()的入口点。
	 * 该函数执行以下操作：
	 * 1. 实例化模块对象（调用T::instantiate()）
	 * 2. 将对象指针存储到_object中
	 * 3. 调用对象的run()方法执行主循环
	 * 4. 清理：删除对象并重置状态
	 *
	 * @param argc 任务参数数量
	 * @param argv 任务参数数组指针
	 * @return 成功返回0，失败返回-1
	 */
	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

		// 此时不需要任务名称，跳过第一个参数
		argc -= 1;
		argv += 1;

		// 调用派生类的实例化方法创建对象
		T *object = T::instantiate(argc, argv);
		_object.store(object);  // 原子操作存储对象指针

		if (object) {
			// 对象创建成功，执行主循环
			object->run();
		} else {
			// 对象创建失败，记录错误
			PX4_ERR("failed to instantiate object");
			ret = -1;
		}

		// 清理资源并退出
		exit_and_cleanup();

		return ret;
	}

	/**
	 * @brief 启动命令处理函数
	 *
	 * 处理'command start'命令，检查模块是否已经在运行，
	 * 如果没有运行则调用T::task_spawn()启动模块。
	 * 整个过程在互斥锁保护下进行，确保线程安全。
	 *
	 * @param argc 任务参数数量
	 * @param argv 任务参数数组指针
	 * @return 成功返回0，失败返回-1
	 */
	static int start_command_base(int argc, char *argv[])
	{
		int ret = 0;
		lock_module();  // 加锁保护

		// 检查模块是否已经在运行
		if (is_running()) {
			ret = -1;
			PX4_ERR("Task already running");  // 模块已在运行，输出错误信息
		} else {
			// 模块未运行，调用派生类的任务生成方法
			ret = T::task_spawn(argc, argv);

			if (ret < 0) {
				PX4_ERR("Task start failed (%i)", ret);  // 启动失败，输出错误码
			}
		}

		unlock_module();  // 解锁
		return ret;
	}

	/**
	 * @brief 停止命令处理函数
	 *
	 * 处理'command stop'命令，检查模块是否正在运行，
	 * 如果正在运行则请求模块停止并等待任务完成。
	 * 包含超时机制，最多等待5秒，超时后强制终止任务。
	 *
	 * @return 成功返回0，失败返回-1
	 */
	static int stop_command()
	{
		int ret = 0;
		lock_module();  // 加锁保护

		// 检查模块是否正在运行
		if (is_running()) {
			T *object = _object.load();  // 原子操作获取对象指针

			if (object) {
				// 请求对象停止运行
				object->request_stop();

				unsigned int i = 0;

				// 等待任务完成，带超时机制
				do {
					unlock_module();  // 临时解锁
					px4_usleep(10000); // 等待10毫秒
					lock_module();     // 重新加锁

					// 超时检查（最多等待5秒）
					if (++i > 500 && _task_id != -1) {
						PX4_ERR("timeout, forcing stop");  // 超时，强制停止

						// 如果不是工作队列任务，则删除任务
						if (_task_id != task_id_is_work_queue) {
							px4_task_delete(_task_id);
						}

						_task_id = -1;  // 标记任务已停止

						// 清理对象
						delete _object.load();
						_object.store(nullptr);

						ret = -1;  // 返回错误码
						break;
					}
				} while (_task_id != -1);  // 继续等待直到任务ID变为-1

			} else {
				// 极少发生的情况，仅可能在工作队列中出现：
				// 如果启动线程没有等待工作队列初始化完成，
				// 且在工作队列中_object分配失败，
				// 且exit_and_cleanup()没有被调用，则将_task_id设为无效
				_task_id = -1;
			}
		}

		unlock_module();  // 解锁
		return ret;
	}

	/**
	 * @brief 状态命令处理函数
	 *
	 * 处理'command status'命令：检查模块是否正在运行，
	 * 如果正在运行则调用print_status()显示详细状态信息，
	 * 否则显示"not running"。
	 *
	 * @return 成功返回0，失败返回-1
	 */
	static int status_command()
	{
		int ret = -1;
		lock_module();  // 加锁保护

		// 检查模块是否正在运行且对象有效
		if (is_running() && _object.load()) {
			T *object = _object.load();  // 获取对象指针
			ret = object->print_status();  // 调用对象的状态打印方法
		} else {
			// 模块未运行
			PX4_INFO("not running");
		}

		unlock_module();  // 解锁
		return ret;
	}

	/**
	 * @brief 打印模块状态信息
	 *
	 * 如果模块正在运行，则打印状态信息。
	 * 派生类可以重写此方法以提供更具体的信息。
	 * 默认实现只是简单地打印"running"。
	 *
	 * @return 成功返回0，失败返回-1
	 */
	virtual int print_status()
	{
		PX4_INFO("running");  // 默认状态信息
		return 0;
	}

	/**
	 * @brief 模块主循环方法
	 *
	 * 用于运行在独立线程中的模块，由run_trampoline()调用。
	 * 派生类必须重写此方法实现模块的核心功能。
	 * 该方法必须在should_exit()返回true时退出循环并返回。
	 *
	 * 典型的实现模式：
	 * while (!should_exit()) {
	 *     // 执行模块的核心工作
	 *     px4_usleep(1000); // 适当的休眠以避免过度占用CPU
	 * }
	 */
	virtual void run()
	{
		// 默认空实现，派生类需要重写
	}

	/**
	 * @brief 检查模块是否正在运行
	 *
	 * 通过检查任务ID是否为-1来判断模块的运行状态。
	 * 任务ID为-1表示模块未运行或已停止。
	 *
	 * @return 模块正在运行返回true，否则返回false
	 */
	static bool is_running()
	{
		return _task_id != -1;  // -1表示无效任务ID
	}

protected:

	/**
	 * @brief 请求模块停止
	 *
	 * 设置退出标志，通知模块应该停止运行。
	 * 可以从模块外部或内部线程调用。
	 * 使用原子操作确保线程安全。
	 */
	virtual void request_stop()
	{
		_task_should_exit.store(true);  // 原子操作设置退出标志

	}

	/**
	 * @brief 检查模块是否应该退出
	 *
	 * 在模块线程内部使用，检查是否收到了停止请求。
	 * 模块的主循环应该定期调用此方法检查退出条件。
	 *
	 * @return 应该退出返回true，否则返回false
	 */
	bool should_exit() const
	{
		return _task_should_exit.load();  // 原子操作读取退出标志
	}

	/**
	 * @brief 退出模块并清理资源
	 *
	 * 从模块线程内部调用，负责清理模块对象和重置状态。
	 * 对于工作队列模块，当should_exit()返回true时，
	 * 需要在派生类的cycle方法中调用此函数。
	 *
	 * 清理步骤：
	 * 1. 加锁保护临界区
	 * 2. 删除模块对象
	 * 3. 重置对象指针为nullptr
	 * 4. 设置任务ID为-1，通知等待线程模块已退出
	 * 5. 解锁
	 */
	static void exit_and_cleanup()
	{
		// 在此处加锁的原因：
		// - 如果启动失败且我们比父线程更快，父线程会设置_task_id，
		//   随后会看起来任务正在运行
		// - 删除对象必须在锁保护下进行
		lock_module();

		// 删除模块对象
		delete _object.load();
		_object.store(nullptr);  // 重置对象指针

		// 设置任务ID为-1，通知可能正在等待模块退出的线程可以继续
		_task_id = -1;
		unlock_module();
	}

	/**
	 * @brief 等待对象初始化完成
	 *
	 * 等待_object从新线程中被初始化。可以从task_spawn()中调用。
	 * 通过轮询方式检查对象是否已创建，带有超时机制。
	 *
	 * @param timeout_ms 超时时间（毫秒），默认1000ms
	 * @return 成功返回0，超时或其他错误返回-1
	 */
	static int wait_until_running(int timeout_ms = 1000)
	{
		int i = 0;

		// 轮询等待对象创建完成
		do {
			px4_usleep(2000);  // 每次等待2毫秒
		} while (!_object.load() && ++i < timeout_ms / 2);  // 检查对象是否已创建

		// 检查是否超时
		if (i >= timeout_ms / 2) {
			PX4_ERR("Timed out while waiting for thread to start");  // 超时错误
			return -1;
		}

		return 0;  // 成功
	}

	/**
	 * @brief 获取模块对象实例
	 *
	 * 返回当前模块的对象实例指针。
	 * 如果模块未运行，则返回nullptr。
	 *
	 * @return 模块对象指针，模块未运行时返回nullptr
	 */
	static T *get_instance()
	{
		return (T *)_object.load();  // 原子操作获取对象指针并转换类型
	}

	/**
	 * @var _object 模块实例指针（如果模块正在运行）
	 * @note 每个模板类型都有一个独立的实例
	 *
	 * 使用原子指针确保多线程访问安全。
	 * 当模块运行时，此指针指向有效的模块对象；
	 * 当模块停止时，此指针为nullptr。
	 */
	static px4::atomic<T *> _object;

	/**
	 * @var _task_id 任务句柄
	 *
	 * 任务ID的含义：
	 * - -1：无效任务，表示模块未运行
	 * - -2：工作队列任务（task_id_is_work_queue）
	 * - 其他正值：有效的任务ID，表示模块正在运行
	 */
	static int _task_id;

	/**
	 * @var task_id_is_work_queue 工作队列任务标识值
	 *
	 * 当模块运行在工作队列而不是独立线程时，
	 * _task_id会被设置为此值(-2)。
	 */
	static constexpr const int task_id_is_work_queue = -2;

private:
	/**
	 * @brief 加锁模块互斥锁
	 *
	 * 获取全局模块互斥锁，保护模块的启动、停止等关键操作。
	 * 所有模块共享同一个互斥锁以减少内存占用。
	 */
	static void lock_module()
	{
		pthread_mutex_lock(&px4_modules_mutex);
	}

	/**
	 * @brief 解锁模块互斥锁
	 *
	 * 释放全局模块互斥锁。
	 */
	static void unlock_module()
	{
		pthread_mutex_unlock(&px4_modules_mutex);
	}

	/**
	 * @var _task_should_exit 任务退出标志
	 *
	 * 原子布尔变量，用于指示任务是否应该退出。
	 * 当外部请求模块停止时，此标志被设置为true。
	 * 模块的主循环应定期检查此标志以决定是否退出。
	 */
	px4::atomic_bool _task_should_exit{false};
};

// 模板类静态成员定义

/**
 * @brief 模块对象指针的静态定义
 *
 * 每个模板实例化都有独立的对象指针，初始化为nullptr。
 * 这确保了不同类型的模块有各自独立的实例管理。
 */
template<class T>
px4::atomic<T *> ModuleBase<T>::_object{nullptr};

/**
 * @brief 任务ID的静态定义
 *
 * 每个模板实例化都有独立的任务ID，初始化为-1（表示无效任务）。
 * 当模块启动时，此值会被设置为有效的任务ID或工作队列标识。
 */
template<class T>
int ModuleBase<T>::_task_id = -1;


#endif /* __cplusplus */


__BEGIN_DECLS

/**
 * @brief 模块文档和命令使用帮助方法
 *
 * 这些方法用于生成统一格式的模块帮助信息。
 * 通过Tools/px_process_module_doc.py脚本提取并处理，
 * 因此必须保持同步。
 *
 * 这些宏和函数提供了标准化的方式来描述模块的：
 * - 功能描述
 * - 命令参数
 * - 使用示例
 * - 参数说明
 */

#ifdef __PX4_NUTTX
/**
 * @brief 在NuttX上禁用模块描述以减少Flash使用量
 *
 * @note 由于GCC的一个bug (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55971)，
 *       我们无法使用宏，但GCC会通过这个空的内联方法移除字符串。
 *
 * @param description 模块提供的功能和重要参数描述
 */
static inline void PRINT_MODULE_DESCRIPTION(const char *description) {}
#else

/**
 * @brief 打印模块文档（也将用于在线文档）
 *
 * 使用Markdown语法，应该包含以下部分：
 * - ### Description（描述）
 *   模块提供的功能和重要参数
 * - ### Implementation（实现）
 *   高层实现概述
 * - ### Examples（示例）
 *   CLI接口使用示例（如果比较复杂的话）
 *
 * 除了Markdown语法外，以'$ '开头的行可以用来标记命令：
 * $ module start -p param
 *
 * @param description 模块描述文档字符串
 */
__EXPORT void PRINT_MODULE_DESCRIPTION(const char *description);
#endif

/**
 * @brief 打印命令名称
 *
 * @param executable_name 在脚本和CLI中使用的命令名称
 * @param category 模块类别，可选值：driver（驱动）, estimator（估计器）,
 *                 controller（控制器）, system（系统）, communication（通信）,
 *                 command（命令）, template（模板）
 */
__EXPORT void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);

/**
 * @brief 指定子类别（可选）
 *
 * @param subcategory 子类别名称，例如：如果类别是'driver'，
 *                    子类别可以是'distance_sensor'（距离传感器）
 */
__EXPORT void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory);

/**
 * @brief 打印没有子命令的简单命令名称
 *
 * 用于不需要子命令的简单模块（参见PRINT_MODULE_USAGE_NAME()）。
 *
 * @param executable_name 可执行文件名称
 * @param category 模块类别
 */
__EXPORT void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);


/**
 * @brief 打印命令及其功能的简短描述
 *
 * @param name 命令名称
 * @param description 命令功能描述
 */
__EXPORT void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);

/**
 * @brief 打印命令名称（不带描述）
 *
 * 这是PRINT_MODULE_USAGE_COMMAND_DESCR的简化版本，
 * 只打印命令名称而不提供描述。
 */
#define PRINT_MODULE_USAGE_COMMAND(name) \
	PRINT_MODULE_USAGE_COMMAND_DESCR(name, NULL);

/**
 * @brief 打印默认命令：stop和status
 *
 * 所有模块都应该支持的标准命令：
 * - stop：停止模块
 * - status：打印状态信息
 */
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() \
	PRINT_MODULE_USAGE_COMMAND("stop"); \
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");

/**
 * @brief 打印I2C或SPI驱动的默认参数
 *
 * 用于显示I2C或SPI驱动程序的标准参数选项。
 *
 * @param i2c_support 如果驱动支持I2C则为true
 * @param spi_support 如果驱动支持SPI则为true
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support);

/**
 * @brief 打印可配置的I2C地址参数
 *
 * 显示通过-a <address>选项配置I2C地址的帮助信息。
 *
 * @param default_address 默认I2C地址
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address);

/**
 * @brief 打印I2C保持运行标志参数
 *
 * 显示-k标志的帮助信息，该标志用于指示I2C驱动
 * 在某些错误条件下继续运行。
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG(void);

/**
 * @note 以下所有PRINT_MODULE_USAGE_PARAM_*方法都适用于
 *       前一个PRINT_MODULE_USAGE_COMMAND_DESCR()调用。
 *       即参数描述与最近定义的命令相关联。
 */

/**
 * @brief 打印整数类型参数
 *
 * @param option_char 选项字符（如'p'对应-p选项）
 * @param default_val 参数默认值（如果不适用则设为-1）
 * @param min_val 参数最小值
 * @param max_val 参数最大值
 * @param description 参数使用说明
 * @param is_optional 如果此参数是可选的则为true
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
		const char *description, bool is_optional);

/**
 * @brief 打印浮点数类型参数
 *
 * @note 参见PRINT_MODULE_USAGE_PARAM_INT()的说明
 *
 * @param option_char 选项字符
 * @param default_val 参数默认值（如果不适用则设为NaN）
 * @param min_val 参数最小值
 * @param max_val 参数最大值
 * @param description 参数使用说明
 * @param is_optional 如果此参数是可选的则为true
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
		const char *description, bool is_optional);

/**
 * @brief 打印标志类型参数（不带值）
 *
 * 用于打印开关类型的参数，如-v（详细模式）等。
 *
 * @note 参见PRINT_MODULE_USAGE_PARAM_INT()的说明
 *
 * @param option_char 选项字符
 * @param description 参数使用说明
 * @param is_optional 如果此参数是可选的则为true
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);

/**
 * @brief 打印字符串类型参数
 *
 * @param option_char 选项字符
 * @param default_val 默认值，可以为nullptr
 * @param values 有效值，可以是以下形式之一：
 *               - nullptr：未指定，或任何值都有效
 *               - "<file>"或"<file:dev>"：文件或设备文件（如串口设备）
 *               - "<topic_name>"：uORB主题名称
 *               - "<value1> [<value2>]"：值列表
 *               - "on|off"：用"|"分隔的有效字符串集合
 * @param description 参数使用说明
 * @param is_optional 如果此参数是可选的则为true
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
		const char *description, bool is_optional);

/**
 * @brief 打印注释
 *
 * 打印适用于后续参数或命令的注释。
 * 例如用来表示某个参数适用于多个或所有命令。
 *
 * @param comment 注释内容
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);


/**
 * @brief 打印参数定义
 *
 * 用于打印不具有典型-p <val>形式的参数，
 * 例如'param set <param> <value>'这样的位置参数。
 *
 * @param values 参数格式，例如"<file>"、"<param> <value>"或"<value1> [<value2>]"
 * @param description 参数使用说明
 * @param is_optional 如果此参数是可选的则为true
 */
__EXPORT void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);


__END_DECLS
