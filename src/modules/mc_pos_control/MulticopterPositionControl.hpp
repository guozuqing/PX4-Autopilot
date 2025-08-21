/****************************************************************************
 * @file MulticopterPositionControl.hpp
 * @brief 多旋翼位置控制器头文件
 *
 * 版权所有 (c) 2013-2020 PX4开发团队。保留所有权利。
 *
 * 在满足以下条件的情况下，允许以源代码和二进制形式重新分发和使用，
 * 无论是否进行修改：
 *
 * 1. 源代码的重新分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的重新分发必须在随分发提供的文档和/或其他材料中
 *    重现上述版权声明、此条件列表和以下免责声明。
 * 3. 未经事先书面许可，不得使用PX4名称或其贡献者的名称来支持或
 *    推广从此软件派生的产品。
 *
 * 本软件由版权持有者和贡献者"按原样"提供，不提供任何明示或暗示的保证，
 * 包括但不限于对适销性和特定用途适用性的暗示保证。在任何情况下，
 * 版权所有者或贡献者均不对任何直接、间接、偶然、特殊、惩罚性或后果性损害
 * （包括但不限于采购替代商品或服务；使用、数据或利润损失；或业务中断）
 * 承担责任，无论此类损害是如何引起的，也无论基于何种责任理论，
 * 无论是合同责任、严格责任还是侵权责任（包括疏忽或其他）。
 *
 ****************************************************************************/

/**
 * @brief 多旋翼位置控制器
 *
 * 该模块实现了多旋翼飞行器的位置控制功能，包括：
 * - 位置环控制（XYZ轴）
 * - 速度环控制
 * - 轨迹跟踪
 * - 起飞和降落控制
 * - 故障安全处理
 */

#pragma once

// 位置控制相关的核心模块
#include "PositionControl/PositionControl.hpp"  // 核心位置控制算法
#include "Takeoff/Takeoff.hpp"                  // 起飞控制逻辑
#include "GotoControl/GotoControl.hpp"          // 目标点导航控制

// 系统和驱动头文件
#include <drivers/drv_hrt.h>                    // 高分辨率定时器
#include <lib/mathlib/math/filter/AlphaFilter.hpp>     // 低通滤波器
#include <lib/mathlib/math/filter/NotchFilter.hpp>     // 陷波滤波器（去除特定频率噪声）
#include <lib/mathlib/math/WelfordMean.hpp>     // Welford在线均值计算
#include <lib/perf/perf_counter.h>              // 性能计数器，用于调试和优化
#include <lib/slew_rate/SlewRateYaw.hpp>        // 偏航角度变化率限制
#include <lib/systemlib/mavlink_log.h>          // MAVLink日志系统

// PX4平台通用组件
#include <px4_platform_common/px4_config.h>     // PX4配置定义
#include <px4_platform_common/defines.h>        // 通用宏定义
#include <px4_platform_common/module.h>         // 模块基类
#include <px4_platform_common/module_params.h>  // 参数系统
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>  // 调度工作项
#include <px4_platform_common/posix.h>          // POSIX兼容层
#include <px4_platform_common/tasks.h>          // 任务管理

// uORB消息系统
#include <uORB/Publication.hpp>                 // 发布者基类
#include <uORB/Subscription.hpp>                // 订阅者基类
#include <uORB/SubscriptionCallback.hpp>        // 带回调的订阅者

// uORB主题定义
#include <uORB/topics/hover_thrust_estimate.h>  // 悬停推力估计
#include <uORB/topics/parameter_update.h>       // 参数更新通知
#include <uORB/topics/trajectory_setpoint.h>    // 轨迹设定点
#include <uORB/topics/vehicle_attitude_setpoint.h>  // 飞行器姿态设定点
#include <uORB/topics/vehicle_constraints.h>    // 飞行器约束条件
#include <uORB/topics/vehicle_control_mode.h>   // 飞行器控制模式
#include <uORB/topics/vehicle_land_detected.h>  // 着陆检测状态
#include <uORB/topics/vehicle_local_position.h> // 飞行器本地位置
#include <uORB/topics/vehicle_local_position_setpoint.h>  // 飞行器本地位置设定点

using namespace time_literals;  // 启用时间字面量（如1_s, 100_ms等）

/**
 * @brief 多旋翼位置控制器主类
 *
 * 继承关系：
 * - ModuleBase: 提供模块管理功能（启动、停止、状态查询等）
 * - ModuleParams: 提供参数管理功能（自动订阅参数更新）
 * - ScheduledWorkItem: 提供定时调度功能（定期执行控制循环）
 */
class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief 构造函数
	 * @param vtol 是否为垂直起降飞行器，默认为false（多旋翼模式）
	 */
	MulticopterPositionControl(bool vtol = false);

	/**
	 * @brief 析构函数，清理资源
	 */
	~MulticopterPositionControl() override;

	/**
	 * @brief 创建任务实例（ModuleBase接口）
	 * @param argc 命令行参数个数
	 * @param argv 命令行参数数组
	 * @return 成功返回0，失败返回负值
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief 处理自定义命令（ModuleBase接口）
	 * @param argc 命令行参数个数
	 * @param argv 命令行参数数组
	 * @return 成功返回0，失败返回负值
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief 打印使用帮助信息（ModuleBase接口）
	 * @param reason 调用原因，可选
	 * @return 总是返回0
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief 初始化控制器
	 * @return 成功返回true，失败返回false
	 */
	bool init();

private:
	/**
	 * @brief 主控制循环函数（ScheduledWorkItem接口）
	 * 定期被调度器调用，执行位置控制算法
	 */
	void Run() override;

	// ========== 控制模块 ==========
	TakeoffHandling _takeoff;  /**< 起飞处理：状态机和斜坡控制，确保飞行器平稳起飞 */

	// ========== uORB发布者 ==========
	orb_advert_t _mavlink_log_pub{nullptr};  /**< MAVLink日志发布句柄 */

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};        /**< 起飞状态发布者 */
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};  /**< 飞行器姿态设定点发布者 */
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};  /**< 飞行器本地位置设定点发布者 */

	// ========== uORB订阅者 ==========
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};  /**< 飞行器本地位置订阅者（带回调触发） */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};  /**< 参数更新订阅者（1秒间隔检查） */

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};     /**< 悬停推力估计订阅者 */
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};         /**< 轨迹设定点订阅者 */
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};         /**< 飞行器约束条件订阅者 */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};       /**< 飞行器控制模式订阅者 */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};     /**< 着陆检测状态订阅者 */

	// ========== 时间戳管理 ==========
	hrt_abstime _time_stamp_last_loop{0};               /**< 上一次控制循环的时间戳，用于计算dt */
	hrt_abstime _time_position_control_enabled{0};      /**< 位置控制激活的时间戳 */

	// ========== 控制设定点和状态 ==========
	trajectory_setpoint_s _setpoint{PositionControl::empty_trajectory_setpoint};  /**< 当前轨迹设定点 */
	vehicle_control_mode_s _vehicle_control_mode{};     /**< 飞行器控制模式状态 */

	/**< 飞行器约束条件，包含速度限制和起飞请求 */
	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,           /**< 时间戳 */
		.speed_up = NAN,         /**< 上升速度限制（m/s），NAN表示无限制 */
		.speed_down = NAN,       /**< 下降速度限制（m/s），NAN表示无限制 */
		.want_takeoff = false,   /**< 是否请求起飞 */
	};

	/**< 着陆检测状态，用于判断飞行器是否在地面 */
	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,          /**< 时间戳 */
		.freefall = false,       /**< 是否处于自由落体状态 */
		.ground_contact = true,  /**< 是否接触地面 */
		.maybe_landed = true,    /**< 可能已着陆 */
		.landed = true,          /**< 确认已着陆 */
	};

	// ========== 参数定义 ==========
	DEFINE_PARAMETERS(
		// 位置控制参数
		(ParamFloat<px4::params::MPC_XY_P>)         _param_mpc_xy_p,          /**< XY轴位置比例增益 */
		(ParamFloat<px4::params::MPC_Z_P>)          _param_mpc_z_p,           /**< Z轴位置比例增益 */
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,  /**< XY轴速度控制P增益（加速度单位） */
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,  /**< XY轴速度控制I增益（加速度单位） */
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,  /**< XY轴速度控制D增益（加速度单位） */
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,   /**< Z轴速度控制P增益（加速度单位） */
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,   /**< Z轴速度控制I增益（加速度单位） */
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,   /**< Z轴速度控制D增益（加速度单位） */
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,    /**< XY轴最大速度限制 */
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,   /**< 自动模式上升速度 */
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,  /**< Z轴最大上升速度 */
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,   /**< 自动模式下降速度 */
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,  /**< Z轴最大下降速度 */
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,   /**< 空中最大倾斜角度 */
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,     /**< 悬停推力值 */
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,       /**< 是否使用悬停推力估计 */
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>)  _param_mpc_acc_decouple,  /**< 是否启用加速度解耦 */

		// 滤波器参数
		(ParamFloat<px4::params::MPC_VEL_LP>)       _param_mpc_vel_lp,        /**< 速度低通滤波器截止频率 */
		(ParamFloat<px4::params::MPC_VEL_NF_FRQ>)   _param_mpc_vel_nf_frq,    /**< 速度陷波滤波器中心频率 */
		(ParamFloat<px4::params::MPC_VEL_NF_BW>)    _param_mpc_vel_nf_bw,     /**< 速度陷波滤波器带宽 */
		(ParamFloat<px4::params::MPC_VELD_LP>)      _param_mpc_veld_lp,       /**< 速度导数低通滤波器截止频率 */

		// 起飞/降落参数
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,  /**< 解锁后电机加速时间 */
		(ParamBool<px4::params::COM_THROW_EN>)      _param_com_throw_en,      /**< 是否启用抛投起飞 */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,    /**< 平滑起飞斜坡时间常数 */
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,     /**< 起飞速度 */
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,    /**< 降落速度 */

		// 手动控制参数
		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,    /**< 手动模式最大速度 */
		(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,  /**< 手动模式后退最大速度 */
		(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,  /**< 手动模式侧向最大速度 */
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,     /**< XY轴巡航速度 */
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,     /**< 降落减速高度阈值 */
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,      /**< 高度控制模式 */
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,   /**< 降落和平滑起飞最大倾斜角 */
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,       /**< 最小推力值 */
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,       /**< 最大推力值 */
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,   /**< XY控制推力余量 */

		// 动力学参数
		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,  /**< 飞行器响应性参数 */
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,       /**< 水平加速度 */
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,  /**< 最大下降加速度 */
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,    /**< 最大上升加速度 */
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,   /**< 最大水平加速度 */
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,     /**< 自动模式加加速度限制 */
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,      /**< 最大加加速度限制 */
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,     /**< 手动模式最大偏航角速度 */
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,     /**< 手动偏航控制时间常数 */

		// 速度限制参数
		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,    /**< 所有模式XY轴速度限制 */
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all,     /**< 所有模式Z轴速度限制 */

		// 误差和偏航参数
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>)   _param_mpc_xy_err_max,    /**< XY轴最大位置误差 */
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,  /**< 自动模式最大偏航角速度 */
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc   /**< 自动模式偏航角加速度 */
	);

	// ========== 统计和滤波器 ==========
	math::WelfordMean<float> _sample_interval_s{};              /**< 采样间隔统计计算（Welford在线算法） */

	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};          /**< XY轴速度低通滤波器 */
	AlphaFilter<float> _vel_z_lp_filter{};                      /**< Z轴速度低通滤波器 */

	math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{}; /**< XY轴速度陷波滤波器（去除振动） */
	math::NotchFilter<float> _vel_z_notch_filter{};             /**< Z轴速度陷波滤波器（去除振动） */

	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};    /**< XY轴速度导数低通滤波器 */
	AlphaFilter<float> _vel_deriv_z_lp_filter{};                /**< Z轴速度导数低通滤波器 */

	// ========== 控制器模块 ==========
	GotoControl _goto_control;     /**< 平滑目标点导航控制器 */
	PositionControl _control;      /**< 核心PID位置控制器 */

	// ========== 状态和调试信息 ==========
	hrt_abstime _last_warn{0};                      /**< 上次发送警告消息的时间戳 */
	bool _hover_thrust_initialized{false};          /**< 悬停推力是否已初始化 */

	// ========== 常量定义 ==========
	/**< 轨迹数据超时时间（微秒），超过此时间认为数据无效 */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/**< 平滑起飞高度阈值，低于此高度时关闭偏航控制并限制倾斜角 */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	/**< 最大安全倾斜角（度），超过此值tanf函数会出现数值问题 */
	static constexpr float MAX_SAFE_TILT_DEG = 89.f;

	// ========== 变化率限制器 ==========
	SlewRate<float> _tilt_limit_slew_rate;          /**< 倾斜角限制变化率控制器 */

	// ========== EKF重置计数器 ==========
	uint8_t _vxy_reset_counter{0};                  /**< XY轴速度EKF重置计数器 */
	uint8_t _vz_reset_counter{0};                   /**< Z轴速度EKF重置计数器 */
	uint8_t _xy_reset_counter{0};                   /**< XY轴位置EKF重置计数器 */
	uint8_t _z_reset_counter{0};                    /**< Z轴位置EKF重置计数器 */
	uint8_t _heading_reset_counter{0};              /**< 航向EKF重置计数器 */

	// ========== 性能监控 ==========
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};  /**< 控制循环执行时间性能计数器 */

	// ========== 私有成员函数 ==========

	/**
	 * @brief 更新本地参数缓存
	 *
	 * 当参数发生变化时更新控制器的本地参数副本。
	 * 可以通过force参数强制更新所有参数。
	 *
	 * @param force 是否强制更新参数（忽略参数是否已更改）
	 */
	void parameters_update(bool force);

	/**
	 * @brief 检查位置/速度状态的有效性并设置飞行器状态
	 *
	 * 验证来自EKF的位置和速度估计是否有效，并将其转换为
	 * 控制器所需的PositionControlStates格式。
	 *
	 * @param local_pos 本地位置消息（包含位置、速度、加速度估计）
	 * @param dt_s 时间步长（秒）
	 * @return PositionControlStates 处理后的控制状态
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * @brief 生成故障安全设定点
	 *
	 * 当没有可执行的设定点时，生成一个安全的设定点来填补空白。
	 * 用于处理过渡期间没有生成合适设定点或接收到的设定点无效的情况。
	 * 这种情况应该只在过渡期间短暂出现，而不是在模式操作期间或设计中出现。
	 *
	 * @param now 当前时间戳
	 * @param states 当前飞行器状态
	 * @param warn 是否发送警告消息
	 * @return trajectory_setpoint_s 生成的故障安全设定点
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * @brief 根据EKF重置增量调整现有设定点并更新本地计数器
	 *
	 * 当EKF进行重置时（例如传感器融合或GPS信号丢失恢复），需要相应地
	 * 调整控制设定点以保持控制的连续性。此函数处理这些调整。
	 *
	 * @param[in] vehicle_local_position 包含EKF重置增量和计数器的结构体
	 * @param[out] setpoint 需要调整的轨迹设定点结构体
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
};
