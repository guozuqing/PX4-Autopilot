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

// 速度环（world/NED系）：将速度期望 _vel_sp 与当前速度 _vel 的误差，
// 通过 PID 形成期望加速度 _acc_sp，再由加速度—推力映射与姿态/推力限幅，得到最终推力 _thr_sp。
// 同时实现垂直方向的抗积分风up、推力分配时的“垂直优先/水平留裕度”、以及水平通道的 Anti-Reset-Windup(ARW)。
void PositionControl::_velocityControl(const float dt)
{
	// ---------- 1) 垂直积分项限幅 ----------
	// 目的：限制 I 项在 z 轴（NED：z 向下为正）上的绝对值，避免积分过大导致推力饱和与恢复缓慢。
	// 这里以 1g 为界（单位：m/s^2 对等映射到 I 项语义，具体见 I 的单位定义与缩放关系）。
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// ---------- 2) 速度 PID 生成期望加速度 ----------
	// 误差：期望速度 - 当前速度
	Vector3f vel_error = _vel_sp - _vel;

	// 经典 PID 的“PD+I”结构（在加速度域）：
	// acc_sp_from_vel = Kp * (vel_sp - vel) + I - Kd * vel_dot
	// 说明：
	//  - _gain_vel_p / _gain_vel_d / _gain_vel_i 为向量按轴增益（可各向异性）。
	//  - _vel_dot 为速度的导数（加速度的估计），用于 D 项抑制快速变化与前馈外扰。
	Vector3f acc_sp_velocity =
		vel_error.emult(_gain_vel_p)          // P：按轴比例增益
		+ _vel_int                             // I：积分量（上一周期累积）
		- _vel_dot.emult(_gain_vel_d);         // D：按轴微分增益作用在 vel_dot 上

	// ---------- 3) 写入期望加速度（跳过 NaN 通道） ----------
	// 某些轴上若 setpoint 或状态为 NaN，则不对该轴施加控制（便于上层按轴屏蔽/切换）。
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// ---------- 4) 加速度→推力映射（含倾角/推力限幅） ----------
	// 将 _acc_sp 转换为姿态/推力目标：内部包含
	//  - 计算机体 z 轴方向 body_z（与合加速度反向一致）
	//  - 倾角限制（最大倾斜角 _lim_tilt）
	//  - z 向推力标量（结合 _hover_thrust 与 cos 投影）
	//  - 得到最终推力向量 _thr_sp（机体系 z 方向 * collective_thrust）
	_accelerationControl();

	// ============ 垂直方向 Anti-Windup（冻结误差，抑制积分继续增长）============
	// 背景：若推力在 z 方向已饱和到上下界，而 vel_error(2) 仍在“推动”积分朝同一方向继续增大，
	// 则会造成严重 windup（积分越积越大、脱离饱和后需要长时间回退）。
	// 处理：当 z 向推力触界且误差同向时，将 z 向误差置零，不再喂给积分器。
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||   // 已达上界（注意：NED向上推力为负），误差还想再“向上”推
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {   // 已达下界，误差还想再“向下”推
		vel_error(2) = 0.f;  // 冻结 z 轴误差，抑制积分增长
	}

	// ============ 推力分配策略：垂直优先 / 水平保留裕度 ============
	// 目的：确保在推力矢量受总幅值限制时，优先满足垂直（高度/爬升率）需求，
	// 同时为水平控制保留一定可用推力的“安全边际”（_lim_thr_xy_margin）。
	const Vector2f thrust_sp_xy(_thr_sp);           // 当前推力的水平分量（从 _thr_sp 取 x/y）
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max); // 总推力上限的平方（避免反复开方）

	// 预留给水平控制的裕度：在“当前水平需求”和“设定的水平上限 margin”之间取较小值
	// 解释：如果当前水平需求已经很小，就按真实需求预留；若很大，则最多只保留 margin，避免过度挤占垂直通道。
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

	// 在总推力上限下，扣除保留给水平的这部分，得到 z 分量可用的最大幅值（平方域）
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// 用上述 z 可用幅值上限，约束 z 向推力（NED 中“向上”为负值，故取负根界限）
	// 意图：避免 z 向被水平分量“挤”得不够用。这里是“先保证 z”，再看还能给水平留多少。
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// 反算：在确定了 z 分量之后，总推力剩余可用于水平分量的上限（圆盘投影半径）
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;
	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// 若当前水平推力需求超出剩余的可用上限，则按比例缩放到该圆盘边界
	// 等价于把 (thr_x, thr_y) 投影/收缩到半径 = thrust_max_xy 的圆内
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// ============ 水平通道的 ARW（Anti-Reset Windup, 跟踪型）============
	// 背景：当输出因饱和无法达到期望加速度时，若仍按原误差积分，I 项会推着输出“顶在饱和边”越积越大；
	// 处理：使用跟踪型 ARW，将“产生的实际加速度”与“期望加速度”之差反馈到误差中，抵消积分的过度增长。
	// 映射关系：thr -> acc 近似用 acc = thr * (g / hover_thrust)
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// 仅在“期望加速度模方 > 输出可产生的加速度模方”且存在饱和迹象时启动 ARW
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		// arw_gain 简化选型：与 Kp 成反比（Kp 大时，本身对误差响应强，ARW 可弱；Kp 小时反之）
		const float arw_gain = 2.f / _gain_vel_p(0);  // 这里用 x 轴 Kp 代表水平；可按需要改为分量化
		const Vector2f acc_sp_xy = _acc_sp.xy();

		// 将 (acc_sp - acc_produced) 的差值按 arw_gain 回灌到速度误差，削弱“无法实现的那部分需求”对积分的推动
		// 注意：回灌到 vel_error（而非直接改 I），便于后续统一用 I += e*Ki*dt 更新，思路更清晰
		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// ---------- 5) NaN 防护 ----------
	// 若误差因上游处理出现 NaN，这里置零，避免把 NaN 扩散到积分器与推力映射。
	ControlMath::setZeroIfNanVector3f(vel_error);

	// ---------- 6) 积分更新 ----------
	// 标准离散积分：I(k+1) = I(k) + Ki * e(k) * dt
	// 说明：
	//  - z 轴的 e(2) 可能已被 AWU 冻结为 0（上一段），从而抑制 windup；
	//  - 水平轴的 e(0/1) 可能已被 ARW 调整，以减轻“无法实现的加速度”导致的 windup。
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}


//========================== 加速度→推力/姿态 ==========================//

// 将期望加速度 _acc_sp（世界系/NED）映射为期望推力向量 _thr_sp（沿机体z轴）并施加倾角/推力限幅。
// 约定：NED坐标（x前、y右、z向下）；“向上”为 z 负方向。_hover_thrust 为抵消 1g 时的归一化推力。
void PositionControl::_accelerationControl()
{
	// --- 1) 构造 z 轴“特定力”基线，用于姿态解算（决定 body_z 朝向） ---
	// specific force = 加速度计测量 - 重力。在 NED 中重力沿 +z，因此对姿态估计使用的“特定力”基线取 -g（指向上）。
	// 若未解耦（_decouple_horizontal_and_vertical_acceleration == false），
	//    则把期望 z 加速度也叠加进来，有助于 body_z 更贴近“总合加速度”的反方向，提高水平加速度跟踪。
	float z_specific_force = -CONSTANTS_ONE_G; // [m/s^2] 取负表示“向上”
	if (!_decouple_horizontal_and_vertical_acceleration) {
		// 注意：_acc_sp(2) 在 NED 中“向上”为负值；例如想强力上升，_acc_sp(2) 更负，使 z_specific_force 更“向上”。
		z_specific_force += _acc_sp(2);
	}

	// --- 2) 期望机体 z 轴方向 body_z ---
	// 物理解读：推力方向与需要的“合加速度”相反（推力用于产生期望加速度并抵消重力），
	// 因此用 (-ax, -ay, -z_specific_force)，再单位化得到机体 z 轴朝向（在世界系表达）。
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

	// --- 3) 倾角限制（最大倾角 _lim_tilt） ---
	// 将 body_z 限制在以世界 z 轴为锥轴、开角为 _lim_tilt 的圆锥内，避免过大俯仰/横滚。
	// 影响：限制水平可用加速度（倾角越小，水平分量越小）；防止 cos 过小导致推力投影爆增。
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	// --- 4) 将期望 z 向加速度映射为 NED z 向推力（标量） ---
	// 线性近似：若 hover_thrust 抵消 1g，则期望 z 加速度 a_z 需要的附加推力 ~ a_z * (hover_thrust/g)。
	// 在 NED 中，“向上推力”为负值（因为推力沿 body_z，且 body_z 与世界 z 夹角小于90°时，z 分量为负）。
	// 推导：thrust_ned_z = a_z * (hover_thrust / g) - hover_thrust
	// 若 a_z = 0（只想维持高度），则 thrust_ned_z = -hover_thrust（大小等于悬停推力、方向“向上”）。
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// --- 5) 姿态投影与集体推力（collective_thrust） ---
	// 为了实现上述 z 向推力分量 thrust_ned_z，在给定姿态 body_z 下，总推力 T 应满足：
	//   thrust_ned_z = T * (e_z^T * body_z) = T * cos(theta)
	// 其中 e_z = (0,0,1) 是世界 z 轴，cos(theta) = dot(e_z, body_z)。
	// 因此：T = thrust_ned_z / cos(theta)。
	// 同时施加下限：T 不能小于 -_lim_thr_min（注意负号；NED中“向上”为负），防止桨叶完全卸载等。
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z)); // = cos(theta)，theta 为机体z与世界z的夹角
	// 注意：cos_ned_body ∈ (0,1]，受 _lim_tilt 限制不应接近0；若接近0会放大T（危险）。
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);

	// --- 6) 生成最终推力向量 ---
	// 推力方向沿 body_z，大小为 collective_thrust（通常为负，表示“向上”）。
	// 该 _thr_sp 将用于后续推力分配/混控（再结合总幅值上限、水平/垂直分配策略等）。
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
