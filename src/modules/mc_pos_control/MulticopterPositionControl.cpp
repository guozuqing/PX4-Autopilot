/**
 * @file MulticopterPositionControl.cpp
 * @brief 多旋翼位置控制模块的实现
 *
 * 本文件实现了多旋翼飞行器的位置控制功能，是PX4飞行控制系统中至关重要的模块之一。
 * 它采用双环控制策略：
 * 1. 外环 - 位置控制：通过比例控制器(P控制器)处理位置误差，生成速度设定点。
 * 2. 内环 - 速度控制：通过比例-积分-微分控制器(PID控制器)处理速度误差，生成加速度指令。
 * 最终输出为推力向量，该向量被分解为推力方向(通过旋转矩阵确定多旋翼的姿态)和推力大小(多旋翼的实际推力)。
 *
 * 主要功能包括：
 * - 三维位置控制：精确控制飞行器在X、Y、Z轴上的位置
 * - 轨迹跟踪：平滑地跟随预设轨迹或实时生成的路径
 * - 起飞和降落处理：实现平滑的起飞过程和安全的降落逻辑
 * - 故障安全机制：处理无效设定点或状态估计问题，确保飞行安全
 * - 参数管理：动态更新控制参数以适应不同飞行条件
 * - 滤波处理：对速度和加速度数据进行滤波，减少噪声影响
 *
 * 本模块不直接使用欧拉角进行控制计算，欧拉角仅用于更人性化的控制输入和日志记录。
 *
 * @author PX4开发团队
 * @copyright Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 */

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

/**
 * @brief 构造函数
 * @param vtol 是否为垂直起降飞行器，默认为false（多旋翼模式）
 *
 * 构造函数初始化了位置控制模块的基本组件和参数：
 * - ModuleParams：继承自参数管理基类，用于管理控制参数
 * - ScheduledWorkItem：继承自调度工作项基类，用于定时执行控制循环，指定模块名称和调度配置
 * - _vehicle_attitude_setpoint_pub：根据是否为VTOL模式选择不同的姿态设定点发布主题
 *
 * 初始化步骤：
 * 1. 设置默认采样间隔为0.01秒（100Hz频率）
 * 2. 强制更新一次参数，确保初始参数正确加载
 * 3. 设置倾斜角度限制的变化率为0.2，用于平滑过渡倾斜限制
 * 4. 发布起飞状态主题，准备向其他模块报告起飞状态
 */
MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	_sample_interval_s.update(0.01f); // 100 Hz默认采样频率
	parameters_update(true); // 初始化时强制更新参数
	_tilt_limit_slew_rate.setSlewRate(.2f); // 设置倾斜限制变化率
	_takeoff_status_pub.advertise(); // 发布起飞状态
}

/**
 * @brief 析构函数
 *
 * 析构函数负责释放资源：
 * - 释放性能计数器_cycle_perf，用于监控控制循环的执行时间
 */
MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf); // 释放性能计数器资源
}

/**
 * @brief 初始化函数
 * @return 初始化成功返回true，否则返回false
 *
 * 初始化位置控制模块：
 * 1. 注册本地位置数据的回调函数，以便接收最新的位置和速度反馈
 * 2. 记录初始时间戳，用于后续计算时间间隔
 * 3. 立即调度控制循环的执行
 *
 * 如果回调注册失败，将记录错误信息并返回false
 */
bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed"); // 注册回调失败
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time(); // 记录上次循环的时间戳
	ScheduleNow(); // 立即调度执行

	return true;
}

/**
 * @brief 更新参数
 * @param force 是否强制更新参数
 *
 * 该函数负责检查并更新控制参数：
 * - 如果有新的参数更新或强制更新标志为true，则从存储中获取最新参数
 * - 计算当前采样频率，用于滤波器参数设置
 * - 配置速度陷波滤波器：如果设置了有效的中心频率和带宽，则应用参数；否则禁用滤波器
 * - 配置速度低通滤波器：如果设置了有效的截止频率，则应用参数；否则禁用滤波
 * - 配置速度导数低通滤波器：同上
 * - 根据系统响应参数调整加速度、倾斜角等控制参数
 * - 应用各种参数约束，确保参数在安全范围内
 * - 更新控制器的增益、推力限制等参数
 *
 * 参数更新逻辑还包括：
 * - 响应性调整：根据系统响应参数调整加速度、倾斜角等
 * - 速度限制统一：如果设置了统一速度参数，则应用到各个方向
 * - 推力范围检查：确保悬停推力在最小和最大推力之间
 * - 起飞和降落速度限制：确保不超过最大上升/下降速度
 */
void MulticopterPositionControl::parameters_update(bool force)
{
	// 检查是否有参数更新
	if (_parameter_update_sub.updated() || force) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		ModuleParams::updateParams();

		float sample_freq_hz = 1.f / _sample_interval_s.mean(); // 计算采样频率

		// 速度陷波滤波器设置，用于去除特定频率的噪声（如电机振动频率）
		if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
			_vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get()); // 设置XY轴速度陷波滤波器参数
			_vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get()); // 设置Z轴速度陷波滤波器参数
		} else {
			_vel_xy_notch_filter.disable(); // 禁用滤波器，如果参数无效
			_vel_z_notch_filter.disable();
		}

		// 速度xy/z低通滤波器，用于平滑速度数据，减少高频噪声
		if (_param_mpc_vel_lp.get() > 0.f) {
			_vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get()); // 设置XY轴速度低通滤波器截止频率
			_vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get()); // 设置Z轴速度低通滤波器截止频率
		} else {
			// 禁用滤波，如果截止频率无效
			_vel_xy_lp_filter.setAlpha(1.f);
			_vel_z_lp_filter.setAlpha(1.f);
		}

		// 速度导数xy/z低通滤波器，用于平滑加速度数据（速度的导数）
		if (_param_mpc_veld_lp.get() > 0.f) {
			_vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get()); // 设置XY轴速度导数低通滤波器截止频率
			_vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get()); // 设置Z轴速度导数低通滤波器截止频率
		} else {
			// 禁用滤波，如果截止频率无效
			_vel_deriv_xy_lp_filter.setAlpha(1.f);
			_vel_deriv_z_lp_filter.setAlpha(1.f);
		}

		// 响应性参数调整部分
		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// 使其在较低端更敏感，通过平方增加非线性响应
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			// 线性插值调整水平加速度参数，从1到15
			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			// 根据响应性调整手动控制的响应时间常数
			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);
			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			// 根据响应性调整空中最大倾斜角
			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);
			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
					(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		// 统一水平速度参数设置
		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		// 统一垂直速度参数设置
		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		// 如果有参数变更，通知系统
		if (num_changed > 0) {
			param_notify_changes();
		}

		// 确保空中最大倾斜角不超过安全值
		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
				    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		// 确保降落倾斜角不超过空中最大倾斜角
		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
				    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		// 设置位置控制增益，XY轴使用相同增益，Z轴独立
		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		// 设置速度控制增益，包括比例、积分和微分项
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		// 设置水平推力余量
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		// 是否解耦水平和垂直加速度
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());
		// 设置GotoControl相关参数，用于导航控制
		_goto_control.setParamMpcAccHor(_param_mpc_acc_hor.get());
		_goto_control.setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
		_goto_control.setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
		_goto_control.setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
		_goto_control.setParamMpcXyCruise(_param_mpc_xy_cruise.get());
		_goto_control.setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
		_goto_control.setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
		_goto_control.setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
		_goto_control.setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
		_goto_control.setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());

		// 检查设计参数是否在绝对最大约束范围内
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
				    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		// 确保手动速度不超过最大速度
		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
				    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		// 确保手动后退速度不超过手动前进速度
		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
				    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		// 确保手动侧向速度不超过手动前进速度
		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
				    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		// 确保自动上升速度不超过最大上升速度
		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
				    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		// 确保自动下降速度不超过最大下降速度
		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
				    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		// 确保悬停推力在最小和最大推力之间
		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
				    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		// 如果不使用悬停推力估计或尚未初始化，则使用参数值
		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// 初始化参数向量并强制约束
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get())); // 起飞速度不超过最大上升速度
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get())); // 降落速度不超过最大下降速度

		// 设置起飞相关参数
		_takeoff.setSpoolupTime(_param_com_spoolup_time.get()); // 设置电机启动时间
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get()); // 设置起飞斜坡时间
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get()); // 生成初始斜坡值
	}
}

/**
 * @brief 设置飞行器状态
 * @param vehicle_local_position 当前本地位置数据
 * @param dt_s 时间间隔（秒）
 * @return 飞行器状态结构体
 *
 * 该函数将从EKF或其他状态估计器获取的飞行器位置、速度、加速度和航向数据组织成标准格式：
 * - 位置：仅在数据有效且有限的情况下设置X、Y、Z坐标，否则设置为NAN
 * - 速度：对XY和Z轴分别处理，应用陷波滤波器和低通滤波器以减少噪声
 * - 加速度：通过速度的变化率计算，并应用低通滤波器平滑
 * - 航向：直接从输入数据获取
 *
 * 如果数据无效，将重置相关滤波器以防止在数据恢复时出现加速度尖峰
 */
PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const float dt_s)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// 仅在有效且有限的情况下设置位置状态
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy; // 设置XY位置
	} else {
		states.position(0) = states.position(1) = NAN; // 无效数据设置为NAN
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z; // 设置Z位置
	} else {
		states.position(2) = NAN; // 无效数据设置为NAN
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState(); // 获取前一次的XY速度状态

		// vel xy notch filter, then low pass filter
		// 首先应用陷波滤波器去除特定频率噪声，然后应用低通滤波器平滑数据
		states.velocity.xy() = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));

		// vel xy derivative low pass filter
		// 计算加速度（速度变化率），并应用低通滤波器平滑
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt_s);
	} else {
		states.velocity(0) = states.velocity(1) = NAN; // 无效数据设置为NAN
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		// 重置滤波器以防止在重新获得速度时出现加速度尖峰
		_vel_xy_lp_filter.reset({});
		_vel_xy_notch_filter.reset();
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {

		const float vel_z_prev = _vel_z_lp_filter.getState(); // 获取前一次的Z速度状态

		// vel z notch filter, then low pass filter
		// 首先应用陷波滤波器去除特定频率噪声，然后应用低通滤波器平滑数据
		states.velocity(2) = _vel_z_lp_filter.update(_vel_z_notch_filter.apply(vehicle_local_position.vz));

		// vel z derivative low pass filter
		// 计算加速度（速度变化率），并应用低通滤波器平滑
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((_vel_z_lp_filter.getState() - vel_z_prev) / dt_s);
	} else {
		states.velocity(2) = NAN; // 无效数据设置为NAN
		states.acceleration(2) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		// 重置滤波器以防止在重新获得速度时出现加速度尖峰
		_vel_z_lp_filter.reset({});
		_vel_z_notch_filter.reset();
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = vehicle_local_position.heading; // 设置航向角

	return states;
}

/**
 * @brief 主控制循环
 *
 * 这是位置控制模块的核心执行函数，以固定频率（通常为250Hz）运行，处理以下任务：
 * 1. 检查退出条件：如果需要退出，取消回调注册并清理资源
 * 2. 重新调度：设置备份调度，确保即使当前循环未完成也能在100ms后再次执行
 * 3. 更新参数：非强制更新控制参数
 * 4. 获取最新数据：从本地位置订阅中获取最新的位置和速度数据
 * 5. 计算时间间隔：根据时间戳计算本次循环与上次循环的时间差，限制在2ms到40ms之间
 * 6. 更新控制模式：检查并更新飞行器控制模式，记录位置控制启用时间
 * 7. 处理悬停推力估计：如果启用，更新悬停推力值
 * 8. 设置飞行器状态：将位置、速度、加速度数据组织成标准格式
 * 9. 处理导航设定点：如果有导航设定点，更新轨迹设定点
 * 10. 调整EKF重置：根据EKF重置情况调整设定点
 * 11. 故障安全处理：如果位置控制启用但没有新设定点，生成故障安全设定点
 * 12. 执行控制逻辑：如果位置控制启用且有有效设定点，执行控制算法
 * 13. 发布控制输出：发布位置设定点和姿态设定点
 * 14. 更新起飞状态：发布最新的起飞状态和倾斜限制
 * 15. 性能监控：记录控制循环的执行时间
 */
void MulticopterPositionControl::Run()
{
	// ========================= [0] 退出路径（资源清理 + 立即返回） =========================
	// should_exit() 由 WorkQueue/Module 框架控制：当模块需要停止（比如任务卸载/重启）时返回 true。
	// 这里必须先注销回调，再做清理，防止之后仍有回调触发造成野指针。
	if (should_exit()) {
		_local_pos_sub.unregisterCallback(); // 从 uORB 的 vehicle_local_position 取消注册回调；避免后续还被触发唤醒
		exit_and_cleanup();                  // 释放本类内部分配的资源（perf、发布器等）
		return;                              // 结束本次执行
	}

	// ========================= [1] 备份调度（容错：100ms 后必进一次） =========================
	// 正常情况下，我们依赖 vehicle_local_position 的“回调触发”（新样本到来）唤醒控制循环。
	// 但为防止回调丢失/阻塞，这里设置“兜底调度”：100ms 后无论如何再执行一次 Run()。
	// 目的：保证即便输入数据异常，也能以 >=10Hz 的频率做最基本的故障安全处理。
	ScheduleDelayed(100_ms);

	// ========================= [2] 参数更新（非强制） =========================
	// 若参数 topic 有更新，这里会拉取最新参数，并据此更新：
	// - 各类滤波器（notch / LPF / 导数 LPF）的离散参数（和采样频率相关）
	// - 速度/加速度/倾角/jerk/推力/起降等上层参数
	// - 响应性映射（SYS_VEHICLE_RESP -> 一系列 MPC_*）
	// 注意：force=false，避免每次循环都做昂贵的参数重配置。
	parameters_update(false);

	// ========================= [3] 性能计时起点（perf counter） =========================
	// 用于飞测分析：统计每次控制循环的执行耗时、最值、均值等。
	perf_begin(_cycle_perf);

	vehicle_local_position_s vehicle_local_position; // 局部承载“本地位置/速度/状态重置”等 EKF 输出

	// ========================= [4] 主数据路径：仅当真的收到“新位置样本”才继续 =========================
	// _local_pos_sub.update() 会检查是否有新数据到来；有则拷贝到 vehicle_local_position 并返回 true。
	if (_local_pos_sub.update(&vehicle_local_position)) {

		// ------------------------- [4.1] 计算 dt（秒），并做数值限幅 -------------------------
		// 时间戳采用 “timestamp_sample”（传感器/EKF 的采样时间，单位：微秒 μs），而非 wall clock：
		// 这样可以最大限度与数据“相位对齐”，减少调度抖动/队列延迟对控制相位的影响。
		// 限幅到 [0.002, 0.04] s（[2, 40] ms），原因：
		// - 下限：避免极小 dt 引起的导数暴噪、斜坡/限速瞬时超大步长（数值不稳定）
		// - 上限：避免极大 dt 引起的积分一次性积太多、控制突变（安全）
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);

		// 记录本次样本的时间戳，为下一次 dt 计算提供基准。
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		// ------------------------- [4.2] 更新“采样间隔在线统计量” -------------------------
		// _sample_interval_s 是一个 WelfordMean（在线均值估计器），用于估算“平均采样周期”。
		// 后续在 parameters_update() 内会使用 1/mean() 作为采样频率，去重新配置滤波器（notch/LPF）的离散参数，
		// 使滤波器的“物理截止/中心频率”在实际回调频率波动时仍基本保持稳定。
		_sample_interval_s.update(dt);

		// ========================= [5] 控制模式边沿检测（启/停位置控制） =========================
		// 仅当 vehicle_control_mode topic 真的有更新（updated() == true）时再读取，避免多余拷贝。
		if (_vehicle_control_mode_sub.updated()) {
			// 在读取新消息前，预先保存“旧的启用状态”，用于做边沿比较（上升沿/下降沿）
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			// 真正把最新消息拷贝到 _vehicle_control_mode，成功返回 true
			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				// 上升沿：从“未启用” -> “启用”
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// 记录“位置控制启用的时间戳”（来自该消息的 timestamp，单位 μs）
					// 后续会用它来判断：启用之后是否收到过“新的轨迹设定点”（若没有，就触发故障安全 setpoint）
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				// 下降沿：从“启用” -> “未启用”
				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// 控制停用时，主动清空当前 setpoint，避免后续不小心继续用旧意图
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}

		// ========================= [6] 落地状态更新（land detector） =========================
		// 用于起飞状态机/地面保护：如接地时清积分、限制倾角、零推力起步等。
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// ========================= [7] 悬停推力估计（HTE）接入（可选） =========================
		// 参数 MPC_USE_HTE 控制是否使用“在线悬停推力估计”来更新控制器中的 hover_thrust。
		// 优点：适应电池电压下降、温度变化、载荷变化、空气密度不同等情况。
		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte; // 承载 HTE 消息（包含估计值、方差、创新量、有效标志等）

			// 若有新 HTE 数据则拷贝到 hte
			if (_hover_thrust_estimate_sub.update(&hte)) {
				// 仅在估计有效时才更新控制器内部悬停推力（避免滤波器启动瞬态/无效数据干扰）
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust); // hover_thrust ∈ [0,1]（推力归一化）
				}
			}
		}

		// ========================= [8] 构建控制状态（PositionControlStates） =========================
		// set_vehicle_states() 内部完成：
		// - pos/vel 有效性判断（invalid -> NAN）
		// - vel 的 notch -> LPF 组合滤波（XY、Z 分开），及其导数 LPF（作为“加速度”估计）
		// - 无效数据时重置滤波器，防止恢复时出现尖峰
		// - yaw <- vehicle_local_position.heading
		PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};

		// ========================= [9] Goto（到点）支持：如有 goto 目标则发布轨迹 setpoint =========================
		// checkForSetpoint() 会检查是否存在 goto 目标以及位置控制是否启用；
		// 若需要，则调用 update() 根据 dt 和当前 states 生成/更新“轨迹设定点”（发布到 uORB）。
		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample,
					   _vehicle_control_mode.flag_multicopter_position_control_enabled)) {
			_goto_control.update(dt, states.position, states.yaw);
		}

		// ========================= [10] 从 uORB 读取“轨迹设定点”（上层/Offboard/Goto 发布） =========================
		_trajectory_setpoint_sub.update(&_setpoint);

		// ========================= [11] EKF 重置对齐：setpoint & 滤波器状态同步 =========================
		// EKF 可能因为漂移/跳变/初始化等进行 reset，并给出 delta_* 与 reset_counters。
		// 为避免 setpoint 与新参考不一致导致瞬时误差/积分冲击：这里将 setpoint pos/vel/yaw 加上 delta_*，
		// 并对内部 LPF 状态也做相同偏置修正（reset notch），保证状态/期望“同参考系”。
		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		// ========================= [12] Failsafe 入口：启用了位置控制但没收到新 setpoint =========================
		// 判断条件：setpoint.timestamp 仍然早于“启用时刻”，且当前样本时间已经晚于启用时刻。
		// 说明：自启用以来从未收到过新的 setpoint（可能上层任务未接管/链路断开），必须进入安全模式。
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				// 生成“保守的安全 setpoint”：一般为“停住 XY、下降/盲降”的组合
				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		// ========================= [13] 主控制路径（必须：位置控制启用 + setpoint 是新的） =========================
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// ------------------------- [13.1] 刷新“飞行器约束” -------------------------
			// _vehicle_constraints 通常由上层任务/任务状态机设置，包含：
			// - speed_up/speed_down：竖直速度上/下限（幅值，NED 约定：上升对应负速度，但这里存的是“幅值(正)”）
			// - want_takeoff：是否请求起飞
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// 修复 speed_up 非法（NaN/过大）情况，防止起飞斜坡卡住或超范围
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			// ------------------------- [13.2] Offboard 起飞意图检测 -------------------------
			// 仅在 Offboard 模式才进行：避免上层偶然地发送旧/非上升类指令引发误起飞。
			// want_takeoff 条件 = 已解锁 + setpoint 足够新（小于 1 秒） + {pos.z 目标更小 | vel.z < 0 | acc.z < 0}
			// 注意 NED：z 向下为正 ⇒ “上升”意味着 z 方向期望为“负号”。
			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {
					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {
					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {
					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// Offboard 下，覆盖竖直速度上下限为参数上限，避免上位机给过紧约束导致“起飞斜坡受阻”
				_vehicle_constraints.speed_up   = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// ------------------------- [13.3] 起飞状态机推进 -------------------------
			// 参数：
			// - armed: 电机是否解锁
			// - landed: 是否接地
			// - want_takeoff: 是否请求起飞（上一节计算）
			// - speed_up: 起飞斜坡允许的最大上升速度（幅值）
			// - skip_takeoff: 是否启用“抛投”（跳过起飞斜坡）
			// - timestamp_sample: 当前样本时间
			bool skip_takeoff = _param_com_throw_en.get();
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
					    _vehicle_constraints.want_takeoff,
					    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			// 起飞阶段判定：
			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup); // 尚未进入斜坡阶段
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight); // 已经离地飞行
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);    // 已飞但仍接地（触地/刮地）

			// 未飞行阶段：固定使用参数 MPC_THR_HOVER（而非 HTE），避免起飞过程中因 HTE 抖动影响离地判据
			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// rampup 期间：为了保证“斜坡”是 z 通道唯一驱动，不允许 z 加速度前馈参与（避免斜坡被覆盖）
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN; // NaN 表示禁用加速度前馈（让 update() 内部忽略）
			}

			// 地面/接地阶段保护：清空 setpoint（无位置/速度意图），并施强向下加速度（NED 正向），效果≈“零推力”
			// 同时清积分，避免地面长时间停留造成积分器累积，一旦离地就过冲。
			if (not_taken_off || flying_but_ground_contact) {
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // 100 m/s² 向下加速度（NED 正），迫使不拉升
				_control.resetIntegral();                                 // 清除全通道积分
			}

			// ------------------------- [13.4] 倾角限制（随起飞阶段切换，并做斜率平滑） -------------------------
			// 起飞前：用 MPC_TILTMAX_LND（落地倾角上限，保守）；
			// 空中：用 MPC_TILTMAX_AIR（空中允许的较大倾角）。
			// 通过 _tilt_limit_slew_rate.update() 做“有界变化率”，避免设定的倾角上限本身突然跳变。
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
					     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			// ------------------------- [13.5] 起飞斜坡：根据 dt 推进 ramp，得到允许的“上升速度幅值” -------------------------
			// 若 constraints.speed_up 无效，则回退到 MPC_Z_VEL_MAX_UP；speed_down 若无效，则回退到 MPC_Z_VEL_MAX_DN。
			const float speed_up = _takeoff.updateRamp(
				dt,
				PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get()
			);
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down
			                                 : _param_mpc_z_vel_max_dn.get();

			// ------------------------- [13.6] 推力限幅：飞行前允许最小推力=0（零推力起步），飞行后=THR_MIN -------------------------
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			// ------------------------- [13.7] 速度限幅：XY 与 EKF vxy_max 取较小值，Z 用上/下幅值 -------------------------
			float max_speed_xy = _param_mpc_xy_vel_max.get();
			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max); // EKF 根据质量/环境可能动态限速
			}
			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // 上升速度“幅值”上限（最终由控制器内部处理符号）
				math::max(speed_down, 0.f)                          // 下降速度“幅值”下限（非负）
			);

			// ------------------------- [13.8] 送入输入设定点（上层/Goto/Offboard） -------------------------
			_control.setInputSetpoint(_setpoint);

			// ------------------------- [13.9] 竖直速度细化：未控高度但控速度时，vz 用 z_deriv 与 vz 融合 -------------------------
			// 条件：pos.z 未控（NAN）、vel.z 有效且非零、EKF 提供 z_deriv&vz 且标记有效；
			// 做法：按 |vel_sp.z|/MPC_LAND_SPEED 归一的权重在 z_deriv 与 vz 之间插值；
			// 直觉：速度小更信 raw vz（稳定），速度大更信 z_deriv（偏置小、无跳步），改善着陆/上升过零附近的连续性。
			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {

				float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = vehicle_local_position.z_deriv * weighting
				                   + vehicle_local_position.vz      * (1.f - weighting);
			}

			// ------------------------- [13.10] 若 XY 未控（既没 vel.xy 也没 pos.xy），清 XY 积分防过冲 -------------------------
			if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
			    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
				_control.resetIntegralXY();
			}

			// ------------------------- [13.11] 送入当前状态（pos/vel/acc/yaw） -------------------------
			_control.setState(states);

			// ------------------------- [13.12] 执行控制器更新（P 外环位姿 + PID 速度 -> 推力向量） -------------------------
			// 失败（返回 false）通常意味着输入无效或内部保护触发：进入 failsafe，再用默认限速重试一次。
			if (!_control.update(dt)) {
				// 清约束，避免继承不确定的限制
				_vehicle_constraints = {0, NAN, NAN, false, {}};

				// 设一个“告警”的 failsafe setpoint（停住/盲降）
				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));

				// 用默认速度限幅（参数上限）重算一次，尽最大努力给出安全的姿态/推力
				_control.setVelocityLimits(_param_mpc_xy_vel_max.get(),
				                           _param_mpc_z_vel_max_up.get(),
				                           _param_mpc_z_vel_max_dn.get());
				_control.update(dt);
			}

			// ------------------------- [13.13] 发布“内部位置设定点”（含 PID 校正） -------------------------
			// 目的：让其他模块（如 LandDetector）可读取“真实控制意图”（而非仅仅输入 setpoint）
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// ------------------------- [13.14] 发布“姿态设定点” -------------------------
			// 内容：期望姿态（四元数/Euler） + 总推力（标量），下游姿态控制器/混控器负责驱动电机达成
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// ========================= [14] 控制未启用路径（维持起飞状态机一致性 + 清积分） =========================
			// 即使位置控制未启用，也推进一次起飞状态机，避免卡在“需被跳过”的阶段（比如非高度模式下）
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
					    vehicle_local_position.timestamp_sample);
			_control.resetIntegral(); // 全通道积分清零，避免未来接管时由于历史积分造成过冲
		}

		// ========================= [15] takeoff_status 发布（当阶段或倾角限制变化） =========================
		// 对外通告起飞阶段（枚举）与当前倾角上限（弧度），便于地面站/其他模块感知。
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit    = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp     = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	// ========================= [16] 性能计时收尾 =========================
	perf_end(_cycle_perf);
}



/**
 * @brief 生成故障安全设定点
 * @param now 当前时间戳
 * @param states 当前飞行器状态
 * @param warn 是否需要警告
 * @return 故障安全轨迹设定点
 *
 * 当位置控制启用但没有有效的轨迹设定点时，调用此函数生成故障安全设定点：
 * - 限制警告频率，每2秒最多警告一次
 * - 如果水平速度有效，停止水平移动（速度设为0）
 * - 如果水平速度无效，以着陆速度下降
 * - 如果垂直速度有效且可以停止，不移动垂直方向
 * - 如果垂直速度无效，紧急下降，推力略低于悬停推力
 */
trajectory_setpoint_s MulticopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// 限制警告频率
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints"); // 无效设定点警告
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// 不要沿xy移动
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait"); // 故障安全：停止并等待
		}
	} else {
		// 以着陆速度下降，因为我们无法停止
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land"); // 故障安全：盲降
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// 如果可以在所有维度上停止，则不要沿z移动
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}
	} else {
		// 紧急下降，推力略低于悬停推力
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent"); // 故障安全：盲降
		}
	}

	return failsafe_setpoint;
}

/**
 * @brief 调整设定点以适应EKF重置
 * @param vehicle_local_position 当前本地位置数据
 * @param setpoint 轨迹设定点
 *
 * 当EKF（扩展卡尔曼滤波器）发生重置时，位置和速度估计会发生跳变，需要调整设定点以保持控制连续性：
 * - 检查设定点时间戳是否早于当前位置时间戳，确保调整有效
 * - 根据重置计数器检测是否发生了重置，如果是则调整速度和位置设定点
 * - 调整滤波器状态以适应速度跳变
 * - 保存最新的重置计数器值，用于下次检测
 */
void MulticopterPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0]; // 调整X速度
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1]; // 调整Y速度
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz; // 调整Z速度
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0]; // 调整X位置
			setpoint.position[1] += vehicle_local_position.delta_xy[1]; // 调整Y位置
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z; // 调整Z位置
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading); // 调整航向角
		}
	}

	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + Vector2f(vehicle_local_position.delta_vxy)); // 重置XY速度低通滤波器
		_vel_xy_notch_filter.reset(); // 重置XY速度陷波滤波器
	}

	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + vehicle_local_position.delta_vz); // 重置Z速度低通滤波器
		_vel_z_notch_filter.reset(); // 重置Z速度陷波滤波器
	}

	// 保存最新的重置计数器
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

/**
 * @brief 任务生成函数
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 成功返回PX4_OK，否则返回PX4_ERROR
 *
 * 该函数负责创建位置控制任务实例：
 * - 检查命令行参数，如果包含"vtol"，则设置为垂直起降模式
 * - 动态分配位置控制对象
 * - 如果分配成功，存储对象指针，设置任务ID，并初始化对象
 * - 如果初始化成功，返回成功状态；否则清理资源并返回错误状态
 */
int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true; // 设置为垂直起降模式
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol); // 创建位置控制实例

	if (instance) {
		_object.store(instance); // 存储实例指针
		_task_id = task_id_is_work_queue; // 设置任务ID

		if (instance->init()) {
			return PX4_OK; // 初始化成功
		}
	} else {
		PX4_ERR("alloc failed"); // 内存分配失败
	}

	delete instance; // 清理资源
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR; // 返回错误状态
}

/**
 * @brief 自定义命令处理
 * @param argc 参数数量
 * @param argv 参数数组
 * @return 处理结果
 *
 * 处理自定义命令，目前仅支持打印用法信息
 */
int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 处理未知命令
}

/**
 * @brief 打印用法信息
 * @param reason 原因字符串
 * @return 始终返回0
 *
 * 打印模块的用法信息，包括描述、命令和参数说明
 */
int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 打印警告原因
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller"); // 模块名称和类别
	PRINT_MODULE_USAGE_COMMAND("start"); // 支持的命令
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true); // 参数说明
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // 默认命令

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
