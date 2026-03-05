/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

using namespace matrix;

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

//------------------------------------------------------------------------------------------
void RateControl::setFeedbackGains(const Vector3f &FP1, const Vector3f &FP2)
{
	_feedback_p1 = FP1;
	_feedback_p2 = FP2;
}

void RateControl::setEsoGains(const Vector3f &beta1, const Vector3f &beta2, const Vector3f &ceta1, const Vector3f &ceta2)
{
    acfly_eso_roll.setEsoParameters(beta1(0), beta2(0), ceta1(0), ceta2(0));
    acfly_eso_pitch.setEsoParameters(beta1(1), beta2(1), ceta1(1), ceta2(1));
    acfly_eso_yaw.setEsoParameters(beta1(2), beta2(2), ceta1(2), ceta2(2));
}

void RateControl::setActGains(const Vector3f &T, const Vector3f &b)
{
    acfly_eso_roll.setActParameters(T(0), b(0));
    acfly_eso_pitch.setActParameters(T(1), b(1));
    acfly_eso_yaw.setActParameters(T(2), b(2));
}

//------------------------------------------------------------------------------------------

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

// ==================== 角速度控制主函数 ====================
// 每个控制周期(~250Hz)由 MulticopterRateControl::Run() 调用一次
// 输入:
//   rate          — 陀螺仪测量的实际角速度 [Roll, Pitch, Yaw] (rad/s)
//   rate_sp       — 姿态控制器或ACRO模式给出的期望角速度 (rad/s)
//   angular_accel — 角加速度估计值 (rad/s²)，仅传统PID的D项使用
//   dt            — 本周期时间步长 (s)
//   landed        — 着陆标志，着陆时会重置ESO/TD状态并简化控制律
// 输出:
//   torque_setpoint — 三轴力矩指令 → 控制分配器 → 电机
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed,
			     const Vector3f &td_rate_sp, const Vector3f &td_rate_accel,
			     hrt_abstime timestamp_sample)
{
	// ========== 第①步：计算角速度误差（PID路径使用）==========
	Vector3f rate_error = rate_sp - rate;

	// ========== 第②步：ESO 闭环校正 ==========
	// 用本周期陀螺仪测量值修正上一周期的开环预测
	// ESO内部流程: 计算观测误差(测量-8步前预测) → 自适应增益 → 校正z1/z2/历史队列

	// Anti-windup: pass directional saturation to ESO (PID-style)
	// Positive saturation → z2 can only decrease; negative → z2 can only increase
	acfly_eso_roll.setSaturation(_control_allocator_saturation_positive(0), _control_allocator_saturation_negative(0));
	acfly_eso_pitch.setSaturation(_control_allocator_saturation_positive(1), _control_allocator_saturation_negative(1));
	acfly_eso_yaw.setSaturation(_control_allocator_saturation_positive(2), _control_allocator_saturation_negative(2));

	// ESO closed-loop correction using gyro timestamp_sample for delay alignment
	acfly_eso_roll.run(rate(0), dt, landed, timestamp_sample);
	acfly_eso_pitch.run(rate(1), dt, landed, timestamp_sample);
	acfly_eso_yaw.run(rate(2), dt, landed, timestamp_sample);

	// 获取ESO估计的角速度 z1 (rad/s)
	float angular_rate_ESO_roll = acfly_eso_roll.getEstimatedAngularRate();
	float angular_rate_ESO_pitch = acfly_eso_pitch.getEstimatedAngularRate();
	float angular_rate_ESO_yaw = acfly_eso_yaw.getEstimatedAngularRate();

	// 获取执行器惯性响应 z_inertia (rad/s²)
	float z_inertia_roll = acfly_eso_roll.getEstimatedMainPower();
	float z_inertia_pitch = acfly_eso_pitch.getEstimatedMainPower();
	float z_inertia_yaw = acfly_eso_yaw.getEstimatedMainPower();

	Vector3f torque_setpoint{};

	// ========== 第③步：使用外部 TD 输出（来自姿态控制器的 AttitudeTd）==========
	// td_rate_sp:    姿态 TD 的 x2 — 平滑后的期望角速度 (rad/s)
	// td_rate_accel: 姿态 TD 的 x3 — 期望角加速度 (rad/s²)
	// TD现在在参考域: td_x2 = α×LPF(W×euler_rate), 纯前馈(不含测量态q)
	const Vector3f &td_x2 = td_rate_sp;
	const Vector3f &td_x3 = td_rate_accel;

	// Cache for status reporting
	_td_rate_sp_cached = td_rate_sp;
	_td_rate_accel_cached = td_rate_accel;

	// ========== 第④步：经典ADRC控制律 — 计算力矩指令 ==========
	//
	// 结构: 标准二阶状态反馈 + 单次扰动补偿 (消除旧版α双重放大)
	//
	// 旧版问题: Ta1 = P2×(Tv1-α) + P2×(td_x3-α)
	//   → α误差被 2×P2 放大，高增益下极易正反馈振荡
	//
	// 新版 (经典ADRC形式):
	//   rate_err  = (td_x2 + rate_sp) - z1        速度误差
	//   accel_err = td_x3 - α                     加速度误差 (α=z_inertia+z2)
	//   Ta1 = P1×P2 × rate_err + (P1+P2) × accel_err
	//
	// 特征多项式: s² + (P1+P2)s + P1×P2 = (s+P1)(s+P2)
	// 闭环极点: -P1, -P2 (独立可调，稳定性有保证)
	// α只被放大(P1+P2)倍，而非旧版的2×P2倍

	// ---- 速度误差 ----
	// rate_err = rate_sp - z1 (TD 不进入误差，只做前馈)
	float rate_err_roll  = rate_sp(0) - angular_rate_ESO_roll;
	float rate_err_pitch = rate_sp(1) - angular_rate_ESO_pitch;
	float rate_err_yaw   = rate_sp(2) - angular_rate_ESO_yaw;

	// ---- 加速度误差 ----
	// accel_err = -z2 (仅扰动, 不含 z_inertia 避免二次补偿)
	// z_inertia 已在力矩输出公式中直接补偿: torque = (zi + T·Ta1) / b
	float accel_err_roll  = -acfly_eso_roll.getEstimatedDisturbance();
	float accel_err_pitch = -acfly_eso_pitch.getEstimatedDisturbance();
	float accel_err_yaw   = -acfly_eso_yaw.getEstimatedDisturbance();

	// ---- 经典ADRC控制律 ----
	// Ta1 = P1×P2 × rate_err + (P1+P2) × accel_err
	// 特征多项式: (s+P1)(s+P2), 闭环极点 -P1, -P2
	float Ta1_roll  = _feedback_p1(0) * _feedback_p2(0) * rate_err_roll
			+ (_feedback_p1(0) + _feedback_p2(0)) * accel_err_roll;
	float Ta1_pitch = _feedback_p1(1) * _feedback_p2(1) * rate_err_pitch
			+ (_feedback_p1(1) + _feedback_p2(1)) * accel_err_pitch;
	float Ta1_yaw   = _feedback_p1(2) * _feedback_p2(2) * rate_err_yaw
			+ (_feedback_p1(2) + _feedback_p2(2)) * accel_err_yaw;

	// ---- 获取执行器模型参数 ----
	// T: 执行器一阶惯性时间常数 (s)，描述电机+螺旋桨的响应延迟
	// b: 控制输入到角加速度的增益 (rad/s²/归一化力矩)
	float T_roll = acfly_eso_roll.getT();
	float T_pitch = acfly_eso_pitch.getT();
	float T_yaw = acfly_eso_yaw.getT();

	float b_roll = acfly_eso_roll.getB();
	float b_pitch = acfly_eso_pitch.getB();
	float b_yaw = acfly_eso_yaw.getB();

	// b值保护: 防止除零，异常时回退到1.0（此时力矩输出=角加速度指令）
	if (b_roll < 0.001f) { b_roll = 1.0f; }
	if (b_pitch < 0.001f) { b_pitch = 1.0f; }
	if (b_yaw < 0.001f) { b_yaw = 1.0f; }

	// ---- 执行器模型反解：角加速度指令 → 归一化力矩 ----
	// torque = (z_inertia + T × Ta1) / b
	// z_inertia: 执行器惯性补偿 (扰动前馈)
	// T × Ta1:  考虑执行器延迟的角加速度指令
	if (!landed) {
		torque_acfly_setpoint(0) = (z_inertia_roll + T_roll * Ta1_roll) / b_roll;
		torque_acfly_setpoint(1) = (z_inertia_pitch + T_pitch * Ta1_pitch) / b_pitch;
		torque_acfly_setpoint(2) = (z_inertia_yaw + T_yaw * Ta1_yaw) / b_yaw;
	} else {
		torque_acfly_setpoint(0) = T_roll * Ta1_roll / b_roll;
		torque_acfly_setpoint(1) = T_pitch * Ta1_pitch / b_pitch;
		torque_acfly_setpoint(2) = T_yaw * Ta1_yaw / b_yaw;
	}

	// ---- TD 纯前馈: 叠加在 ADRC 输出之上 ----
	const float k_ff_rate = 0.15f;
	const float k_ff_acc  = 0.05f;

	if (!landed) {
		for (int i = 0; i < 3; i++) {
			torque_acfly_setpoint(i) += k_ff_rate * td_x2(i) + k_ff_acc * td_x3(i);
		}
	}

	// 安全限幅: ESO力矩输出限制在[-1, 1]范围内
	for (int i = 0; i < 3; i++) {
		torque_acfly_setpoint(i) = math::constrain(torque_acfly_setpoint(i), -1.0f, 1.0f);
	}

	// DEBUG: print ESO key signals every ~1s (250 cycles)
	if (++_debug_counter >= 250) {
		_debug_counter = 0;
		PX4_INFO("ESO R: rerr=%.4f aerr=%.4f Ta1=%.1f torq=%.4f z1=%.4f z2=%.2f zi=%.2f",
			 (double)rate_err_roll, (double)accel_err_roll,
			 (double)Ta1_roll, (double)torque_acfly_setpoint(0),
			 (double)angular_rate_ESO_roll,
			 (double)acfly_eso_roll.getEstimatedDisturbance(),
			 (double)z_inertia_roll);
	}

	// ========== 第⑤步：传统 PID 并行计算 ==========
	// torque_pid = Kp×误差 + 积分 - Kd×角加速度 + Kff×前馈
	// PID始终运行，与ESO路径并行，以便随时可切换回传统模式
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// ========== 第⑥步：模式选择器 — 决定每轴使用 ESO 还是 PID ==========
	// RATE_CTRL_MODE 参数控制，支持逐轴递进验证
	// 设计意图: 先在单轴调通ESO，确认效果后逐步扩展，降低调试风险
	//
	//   模式0: 全PID (传统模式，安全回退)
	//   模式1: Roll=ESO, Pitch=PID, Yaw=PID (Roll单轴验证)
	//   模式2: Roll=PID, Pitch=ESO, Yaw=PID (Pitch单轴验证)
	//   模式3: Roll=PID, Pitch=PID, Yaw=ESO (Yaw单轴验证)
	//   模式4: Roll=ESO, Pitch=ESO, Yaw=PID (双轴验证)
	//   模式5: Roll=ESO, Pitch=ESO, Yaw=ESO (全ESO)
	//
/* 	if (_rate_ctrl_mode == 0) {
		// PID mode: all axes use PID output
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	} else {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll uses ESO
		torque_setpoint(1) = torque(1);  // Pitch uses PID
		torque_setpoint(2) = torque(2);  // Yaw uses PID
	} */
	if (_rate_ctrl_mode == 0) {
		// 模式0: 全部使用PID输出
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	}
	else if (_rate_ctrl_mode == 1) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll使用ESO
		torque_setpoint(1) = torque(1);                  // Pitch使用PID
		torque_setpoint(2) = torque(2);                  // Yaw使用PID
	}
	else if (_rate_ctrl_mode == 2) {
		torque_setpoint(0) = torque(0);                  // Roll使用PID
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch使用ESO
		torque_setpoint(2) = torque(2);                  // Yaw使用PID
	}
	else if (_rate_ctrl_mode == 3) {
		torque_setpoint(0) = torque(0);                  // Roll使用PID
		torque_setpoint(1) = torque(1);                  // Pitch使用PID
		torque_setpoint(2) = torque_acfly_setpoint(2);   // Yaw使用ESO
	}
	else if (_rate_ctrl_mode == 4) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll使用ESO
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch使用ESO
		torque_setpoint(2) = torque(2);                  // Yaw使用PID
	}
	else if (_rate_ctrl_mode == 5) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll使用ESO
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch使用ESO
		torque_setpoint(2) = torque_acfly_setpoint(2);   // Yaw使用ESO
	}
	else {
		// 未知模式: 安全回退到全PID
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	}
	// ========== 第⑦步：更新PID积分器（仅飞行中）==========
	// 积分器有三重保护:
	//   1. 控制分配饱和反馈 — 输出饱和时停止积分
	//   2. 非线性衰减 — 大误差时积分自动弱化
	//   3. 硬限幅 — constrain(-lim, +lim)
	// 注意: 所有模式下积分器都在运行，即使某轴使用ESO
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	// ========== 第⑧步：ESO 开环预测 — 为下一周期做准备 ==========
	// 将本周期最终输出的力矩告诉ESO，ESO据此预测下一周期的状态:
	//   z_inertia += dt/T × (b×u - z_inertia)  // 执行器惯性模型
	//   z1 += (z_inertia + z2) × dt            // 角速度开环预测
	// 下一次 run() 被调用时，ESO会用新的陀螺仪测量值校正这个预测，形成闭环
	// 注意: 所有模式下都更新ESO（包括纯PID模式），保持ESO状态热备
	// ESO open-loop prediction using ACTUAL applied torque.
	// When saturated, the actual torque is less than requested.
	// Compute actual: desired minus unallocated (clamped to [-1,1]).
	// The saturation flags tell us direction, and the torque is already clamped to [-1,1],
	// so the torque_setpoint after our clamp IS the actual applied torque.
	hrt_abstime now_us = hrt_absolute_time();
	acfly_eso_roll.updateControlInput(torque_setpoint(0), now_us);
	acfly_eso_pitch.updateControlInput(torque_setpoint(1), now_us);
	acfly_eso_yaw.updateControlInput(torque_setpoint(2), now_us);

	return torque_setpoint;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);

	//------------------------------------------------------------------------------------------
	// Report detailed ESO and TD states (Roll axis only)
	rate_ctrl_status.td_trate_roll = _td_rate_sp_cached(0);
	rate_ctrl_status.td_trate_pitch = _td_rate_sp_cached(1);
	rate_ctrl_status.td_trate_yaw = _td_rate_sp_cached(2);
	rate_ctrl_status.td_tdrate_roll = _td_rate_accel_cached(0);
	rate_ctrl_status.td_tdrate_pitch = _td_rate_accel_cached(1);
	rate_ctrl_status.td_tdrate_yaw = _td_rate_accel_cached(2);

	rate_ctrl_status.eso_rate_roll = acfly_eso_roll.getEstimatedAngularRate();
	rate_ctrl_status.eso_rate_pitch = acfly_eso_pitch.getEstimatedAngularRate();
	rate_ctrl_status.eso_rate_yaw = acfly_eso_yaw.getEstimatedAngularRate();
	rate_ctrl_status.eso_drate_roll = acfly_eso_roll.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_drate_pitch = acfly_eso_pitch.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_drate_yaw = acfly_eso_yaw.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_inertia_roll = acfly_eso_roll.getEstimatedMainPower() / acfly_eso_roll.getB();
	rate_ctrl_status.eso_inertia_pitch = acfly_eso_pitch.getEstimatedMainPower() / acfly_eso_pitch.getB();
	rate_ctrl_status.eso_inertia_yaw = acfly_eso_yaw.getEstimatedMainPower() / acfly_eso_yaw.getB();

	rate_ctrl_status.eso_dis_roll = acfly_eso_roll.getEstimatedDisturbance();
	rate_ctrl_status.eso_dis_pitch = acfly_eso_pitch.getEstimatedDisturbance();
	rate_ctrl_status.eso_dis_yaw = acfly_eso_yaw.getEstimatedDisturbance();

	rate_ctrl_status.rate_cmd_roll = torque_acfly_setpoint(0);
	rate_ctrl_status.rate_cmd_pitch = torque_acfly_setpoint(1);
	rate_ctrl_status.rate_cmd_yaw = torque_acfly_setpoint(2);
	rate_ctrl_status.rate_pidcmd_roll = target_darate(0);
	rate_ctrl_status.rate_pidcmd_pitch = target_darate(1);
	rate_ctrl_status.rate_pidcmd_yaw = target_darate(2);
	//------------------------------------------------------------------------------------------
}
