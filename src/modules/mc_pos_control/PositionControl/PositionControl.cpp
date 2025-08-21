/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 * @brief PositionControl 类实现文件 —— 详细中文注释
 *
 * 本文件实现多旋翼位置控制器的核心逻辑：
 *   1) 位置 P 控制生成速度期望修正；
 *   2) 速度 PID 控制生成加速度期望；
 *   3) 加速度到推力/姿态的映射及限幅；
 *   4) 各类饱和与抗积分风up（ARW）策略。
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

// 空轨迹 setpoint：所有字段 NaN/0，表示“当前不受控/未设定”
const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

//=============================== 参数设置 ===============================//

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;   // 速度环比例增益
	_gain_vel_i = I;   // 速度环积分增益（用于抵消稳态偏差）
	_gain_vel_d = D;   // 速度环微分增益（抑制动态误差/改善相位）
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal; // 水平速度上限（用于前馈+位置环）
	_lim_vel_up = vel_up;                 // 垂直上升速度上限
	_lim_vel_down = vel_down;             // 垂直下降速度上限
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// 确保始终有足够的推力矢量长度来推断姿态（避免完全为 0 导致姿态不可解）
	_lim_thr_min = math::max(min, 10e-4f); // 下限至少留一点余量
	_lim_thr_max = max;                    // 推力上限（归一化）
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	// 当需要优先保证垂直推力（如强爬升）时，水平控制仍保留的推力裕度
	_lim_thr_xy_margin = margin;
}

//=========================== 悬停推力平滑更新 ===========================//

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// 推导：
	// 推力方程 T = a_sp * Th / g - Th，其中 a_sp 为期望加速度，Th 为悬停推力，g 为重力常数。
	// 若将 Th 替换为 Th'，希望保持 T 不变：
	//   a_sp' * Th' / g - Th' = a_sp * Th / g - Th  =>  a_sp' = (a_sp - g) * Th / Th' + g
	// 于是可将 (a_sp' - a_sp) 加到积分器上，吸收Th变更带来的影响，从而避免输出突变。
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
	       + CONSTANTS_ONE_G - _acc_sp(2);
}

//=============================== 状态/输入 ===============================//

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;      // 当前位置
	_vel = states.velocity;      // 当前速度
	_yaw = states.yaw;           // 当前偏航角
	_vel_dot = states.acceleration; // 速度导数（或加速度估计）
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);      // 期望位置，NaN 表示该轴不受控
	_vel_sp = Vector3f(setpoint.velocity);      // 期望速度（可作为前馈）
	_acc_sp = Vector3f(setpoint.acceleration);  // 期望加速度
	_yaw_sp = setpoint.yaw;                     // 期望偏航
	_yawspeed_sp = setpoint.yawspeed;           // 期望偏航角速度
}

//============================= 主循环更新 =============================//

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid(); // 校验输入 setpoint/状态是否成对/有限

	if (valid) {
		_positionControl();   // 位置 P 控制：由位置误差生成速度期望修正
		_velocityControl(dt); // 速度 PID 控制：生成加速度/推力期望，并做饱和/抗积分风up

		// 若未给定偏航速率，默认 0；若未给定偏航角，则保持当前偏航
		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: 改进禁用偏航控制的策略
	}

	// 输出必须包含有效的加速度与推力 setpoint，否则视为出错
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

//============================= 位置 P 控制 =============================//

void PositionControl::_positionControl()
{
	// 位置比例控制：vel_sp_position = (pos_sp - pos) .* Kp_pos
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

	// 将“速度前馈”叠加到由位置误差生成的速度修正上
	// 若 vel_sp 或 pos_sp 中存在 NaN，则不生效（ControlMath 内已处理）
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);

	// 后续约束需要有限数，若存在 NaN 将其置 0（仅作为参考向量，不改变 _vel_sp 的 NaN 语义）
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// 限制水平速度：优先保证“基于位置误差的速度分量”，再叠加前馈分量，最后整体限幅
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);

	// 限制垂直速度（z 轴）：向上为负（NED 约定），因此上升限为 -_lim_vel_up，下降限为 +_lim_vel_down
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

//============================= 速度 PID 控制 =============================//

void PositionControl::_velocityControl(const float dt)
{
	// 垂直方向的积分项限幅，避免过大（以 1g 为界）
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID：a_sp_from_vel = Kp*(vel_sp-vel) + I - Kd*vel_dot
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// 若某些 setpoint/状态为 NaN，则相应通道不提供控制输入
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// 将期望加速度映射为推力，并进行倾角/推力限幅
	_accelerationControl();

	// -------- 抗积分风up（垂直方向）--------
	// 当推力在 z 方向达到上下界且速度误差继续推动同向积分时，冻结该方向的误差，抑制积分增长
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// -------- 垂直优先 / 水平保留裕度 --------
	const Vector2f thrust_sp_xy(_thr_sp);           // 当前推力的水平分量
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// 预留给水平控制的裕度（取最小值：当前水平需求 vs 预留上限）
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// 依据预留的水平裕度，饱和可用的最大向下推力（NED：z 负向为上，故这里为负号）
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// 在优先保证 z 推力后，反算剩余可用于水平的推力上限
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;
	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// 水平推力饱和（按比例缩放至可用圆盘）
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// -------- 跟踪型 Anti-Windup（水平）--------
	// 当输出因饱和无法达到期望加速度时，利用 ARW 纠正 vel_error，避免积分继续将输出推向饱和边界。
	// 参考：Anti-Reset Windup for PID controllers, L. Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// 仅在信号被饱和时启动 ARW 校正（期望加速度的模方 > 实际可产生的加速度模方）
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0); // 简化实现：与 Kp 成反比
		const Vector2f acc_sp_xy = _acc_sp.xy();
		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// 防止积分项出现 NaN：若误差出现 NaN，置零
	ControlMath::setZeroIfNanVector3f(vel_error);

	// 积分更新（I = I + Ki * e * dt）
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

//========================== 加速度→推力/姿态 ==========================//

void PositionControl::_accelerationControl()
{
	// 1) 设定特定力（specific force）在 z 轴上的基线：默认仅 -g，用于姿态解算；
	//    若不解耦，则叠加期望 z 加速度，提高水平加速度跟踪精度。
	float z_specific_force = -CONSTANTS_ONE_G; // NED 中，重力方向为 +z，特定力取反
	if (!_decouple_horizontal_and_vertical_acceleration) {
		z_specific_force += _acc_sp(2);
	}

	// 2) 期望机体 z 轴方向（body_z）：与期望加速度相反方向（推力方向），并单位化
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

	// 3) 倾角限制：将 body_z 限制在与世界 z 轴的锥面内（最大倾角 _lim_tilt）
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	// 4) 将期望加速度映射为推力：假设“悬停推力 = 对抗 1g 的推力”
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// 5) 将推力投影到计划的机体姿态上：collective_thrust = Tz / cos(theta)
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);

	// 6) 最终推力向量：沿 body_z 方向分配总推力
	_thr_sp = body_z * collective_thrust;
}

//============================== 输入校验 ==============================//

bool PositionControl::_inputValid()
{
	bool valid = true;

	// 每个轴 x/y/z 至少应提供一种 setpoint（位置/速度/加速度其中之一），否则该轴不可控
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x 与 y 必须成对提供（例如只给 x 位置而不給 y 位置会导致平面控制不一致）
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// 对每个被控制的状态量，其估计值必须有效（有限）
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}
		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

//============================== 输出打包 ==============================//

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	// 直接导出用于日志/下游的 setpoint（包含 PID 与前馈叠加后的“最终值”）
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration); // 期望加速度
	_thr_sp.copyTo(local_position_setpoint.thrust);       // 期望推力（归一化）
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	// 将推力向量与偏航设定转换为姿态 setpoint
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp; // 偏航角速度设定
}
