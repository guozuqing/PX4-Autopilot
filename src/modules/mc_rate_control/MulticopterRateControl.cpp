/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

// 构造函数：初始化角速度控制模块
// vtol: 是否为垂直起降(VTOL)模式，决定力矩/推力话题发布到虚拟MC通道还是真实通道
MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),                          // 无父模块
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl), // 注册到角速度控制工作队列
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")) // 性能计数器，用于统计主循环耗时
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING; // 默认为旋翼机类型

	parameters_updated();              // 首次加载所有参数
	_controller_status_pub.advertise(); // 注册控制器状态发布者
}

// 析构函数：释放性能计数器资源
MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

// 初始化：注册角速度数据的回调函数
// 当陀螺仪有新数据时，系统会自动调用 Run() 函数
bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

// ==================== 参数更新函数 ====================
// 当 QGC 修改参数或系统启动时调用，将所有参数传递给底层控制器
void
MulticopterRateControl::parameters_updated()
{
	// ---- 传统 PID 增益 ----
	// 总增益 K 用于将并联形式 (P + I/s + sD) 转换为理想形式 (K * [1 + 1/sTi + sTd])
	// 最终: Kp = K*P, Ki = K*I, Kd = K*D
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	//-------------------------- 单参数控制相关参数 --------------------------

	// 单参数控制律反馈增益 P1(速度层), P2(加速度层)，每轴独立
	_rate_control.setFeedbackGains(
		Vector3f(_param_rate_feedr_p1.get(), _param_rate_feedp_p1.get(), _param_rate_feedy_p1.get()),
		Vector3f(_param_rate_feedr_p2.get(), _param_rate_feedp_p2.get(), _param_rate_feedy_p2.get()));

	// ESO 观测器增益: beta1(角速度校正), beta2(扰动校正), ceta1/ceta2(自适应系数)
	_rate_control.setEsoGains(
		Vector3f(_param_rate_esor_beta1.get(), _param_rate_esop_beta1.get(), _param_rate_esoy_beta1.get()),
		Vector3f(_param_rate_esor_beta2.get(), _param_rate_esop_beta2.get(), _param_rate_esoy_beta2.get()),
		Vector3f(_param_rate_esor_ceta1.get(), _param_rate_esop_ceta1.get(), _param_rate_esoy_ceta1.get()),
		Vector3f(_param_rate_esor_ceta2.get(), _param_rate_esop_ceta2.get(), _param_rate_esoy_ceta2.get()));

	// 执行器模型参数: T(一阶惯性时间常数), b(控制增益)
	_rate_control.setActGains(
		Vector3f(_param_rate_act_T_r.get(), _param_rate_act_T_p.get(), _param_rate_act_T_y.get()),
	        Vector3f(_param_rate_act_b_r.get(), _param_rate_act_b_p.get(), _param_rate_act_b_y.get()));

	// TD 跟踪微分器参数: P1(备用), P2(速度跟踪带宽), P3(加速度跟踪带宽)
	_rate_control.setTdGains(
		_param_rate_td_p1.get(), _param_rate_td_p2.get(), _param_rate_td_p3.get());

	// 控制模式: 0=全PID, 1-5=ESO/PID按轴混合
	const int new_mode = _param_rate_eso_ctrl_mode.get();
	_rate_control.setRateCtrlMode(new_mode);

	// 模式变化时打印日志，方便调试确认
	if (new_mode != _last_rate_ctrl_mode) {
		static const char *mode_desc[] = {
			"PID (all axes)",
			"Roll=ESO, Pitch=PID, Yaw=PID",
			"Roll=PID, Pitch=ESO, Yaw=PID",
			"Roll=PID, Pitch=PID, Yaw=ESO",
			"Roll=ESO, Pitch=ESO, Yaw=PID",
			"Roll=ESO, Pitch=ESO, Yaw=ESO"
		};

		if (new_mode >= 0 && new_mode <= 5) {
			PX4_INFO("Rate ctrl mode changed: %d -> %d [%s]", _last_rate_ctrl_mode, new_mode, mode_desc[new_mode]);
		} else {
			PX4_WARN("Rate ctrl mode changed: %d -> %d [unknown, fallback PID]", _last_rate_ctrl_mode, new_mode);
		}

		_last_rate_ctrl_mode = new_mode;
	}
	//----------------------------------------------------------------------

	// PID 积分器限幅
	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	// PID 前馈增益
	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// ACRO(手动特技)模式下的最大角速度限制 (deg/s → rad/s)
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	// Yaw 轴力矩输出低通滤波截止频率，减少旋翼加速引起的高频振荡
	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

// ==================== 主循环 Run() ====================
// 由陀螺仪数据更新触发（~250Hz），是整个角速度控制的入口
void
MulticopterRateControl::Run()
{
	// 检查模块是否应该退出
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf); // 开始计时

	// ---- 参数热更新检查 ----
	// 当 QGC 或命令行修改参数时，此处会触发重新加载
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update); // 消费更新事件

		updateParams();       // 更新 ModuleParams 基类中的参数缓存
		parameters_updated(); // 将新参数传递给底层控制器
	}

	// ---- 陀螺仪数据更新 → 执行控制 ----
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// 计算时间步长 dt，并限制在 [0.125ms, 20ms] 范围内防止异常
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};                   // 实际角速度 (rad/s)
		const Vector3f angular_accel{angular_velocity.xyz_derivative}; // 实际角加速度 (rad/s²，仅PID D项用)

		// 更新其他订阅话题
		_vehicle_control_mode_sub.update(&_vehicle_control_mode); // 控制模式(手动/自稳/自动等)

		// 更新着陆检测状态(用于 ESO/TD 重置和积分器控制)
		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status); // 飞行器状态(机型、解锁等)

		// ---- 获取期望角速度设定值 ----
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// 手动 ACRO 模式：从遥控器摇杆生成角速度设定值
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// 摇杆量经 superexpo 曲线映射，实现中心精细、边缘灵敏的手感
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);                   // 映射到实际角速度 (rad/s)
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f; // 油门 [-1,1] → 推力 [0,-1] (NED坐标系Z轴朝下)
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;                       // ACRO 模式无水平推力

				// 发布角速度设定值，供日志记录和其他模块使用
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			// 非ACRO模式：从姿态控制器获取角速度设定值
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				// 对无效值做保护：NaN 时使用当前实际角速度
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// ---- 运行角速度控制器 ----
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// 未解锁或非旋翼机模式时，清零 PID 积分器防止累积
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// 从控制分配器获取饱和状态，用于 PID 积分器的抗饱和（anti-windup）
			// 当电机无法完全实现请求的力矩时，会报告未分配的力矩
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive; // 正方向饱和标志 [Roll, Pitch, Yaw]
				Vector<bool, 3> saturation_negative; // 负方向饱和标志

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;  // 正方向力矩不够

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;  // 负方向力矩不够
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				// 通知控制器当前饱和方向，PID 积分器在饱和方向上停止积累
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// ★★★ 核心：调用角速度控制器 update() ★★★
			// 内部完成: ESO观测 → TD跟踪 → 单参数控制律 → PID → 模式选择 → ESO预测
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// Yaw 轴力矩低通滤波，抑制旋翼加减速引起的高频振荡
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// 发布控制器内部状态(ESO/TD/PID各项输出)，供日志分析
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// ---- 发布推力和力矩设定值 → 控制分配器 → 电机 ----
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);  // 推力 (来自姿态控制器或ACRO模式)
			// 力矩输出做 NaN 保护，异常时输出 0 防止失控
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// 电池电压补偿：电压下降时自动增大输出，保持一致的控制手感
			// 原理: 低电压时电机效率下降，需要更大的 PWM 占空比
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale; // scale > 1 表示电压低于标称值
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						// 缩放后限幅到 [-1, 1] 防止超出控制分配器范围
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			// 发布推力和力矩到控制分配器
			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			// 更新执行器控制功率统计(用于日志和监控)
			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf); // 结束计时
}

// 更新执行器控制功率统计
// 每 500ms 计算一次平均控制功率 = Σ(torque² × dt) / Σdt，反映控制活动强度
void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	// 累积每轴力矩的平方 × 时间（能量积分）
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	// 每 500ms 发布一次平均功率
	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time; // 平均功率
			_control_energy[i] = 0.f; // 重置累积
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

// 任务启动入口：创建模块实例并初始化
// 命令行使用: mc_rate_control start [vtol]
int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true; // VTOL 模式使用虚拟 MC 通道发布力矩/推力
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue; // 使用工作队列而非独立线程

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

// 自定义命令处理（当前无自定义命令）
int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// 打印模块使用说明
int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// 模块主入口函数（NuttX 导出符号）
extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
