/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
 * @file AttitudeTd.cpp
 *
 * Fourth-order tracking differentiator for attitude control.
 * Based on the same nonlinear smoothKp design as RateTd, extended to 4 layers.
 *
 * Domain: rotation vector (body frame) — no Euler singularities.
 * Chain: ref_d → x1(rot_vec) → x2(body_rate) → x3(body_accel) → x4(jerk) → T5(snap)
 *
 * Each layer uses the same pattern:
 *   1. Compute tracking error e = target - state
 *   2. Apply nonlinear gain smoothKp to get the control signal for the next layer
 *   3. The last layer's output T5 drives x4 via Euler integration
 */

#include "AttitudeTd.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void AttitudeTd::setParameters(float P1, float P2, float P3, float P4)
{
	_P1 = P1;
	_P2 = P2;
	_P3 = P3;
	_P4 = P4;

	// Set physically meaningful saturation radii for each layer.
	// smoothKp output saturates at ±r, preventing 4-layer cascade divergence.
	// With r=1e12 (default), smoothKp is purely linear → unstable for high P.
	_r1 = 10.0f;   // angular velocity limit  [rad/s]   (~573 deg/s)
	_r2 = 50.0f;   // angular accel limit     [rad/s²]
	_r3 = 200.0f;  // jerk limit              [rad/s³]
	_r4 = 500.0f;  // snap limit              [rad/s⁴]
}

void AttitudeTd::reset()
{
	_x1.zero();
	_x2.zero();
	_x3.zero();
	_x4.zero();
	_T5.zero();
}

// ==================== 4阶TD跟踪微分器 ====================
// 输入: ref_d — 旋转向量 (body frame) [rad]
//   ref_d = 2 × canonical(q⁻¹ × qd).imag()
//   在小角度下 ≈ 姿态误差角度, 时间导数 ≈ 机体系角速度
// 四层状态:
//   _x1 — 平滑旋转向量 (rad)，rate_sp = Kp × x1 即姿态P修正
//   _x2 — 期望机体角速度 (rad/s)，直接给ESO (无需W变换)
//   _x3 — 期望机体角加速度 (rad/s²)，直接给ESO (无需Ẇ计算)
//   _x4 — jerk (rad/s³)，不使用
//   _T5 — snap，驱动信号
//
// 积分链:
//   x1' = x2
//   x2' = x3
//   x3' = x4
//   x4' = T5
void AttitudeTd::track(const Vector3f &ref_d, float dt, bool landed)
{
	if (landed || !_initialized) {
		// Initialize x1 to current input to avoid jump at takeoff
		_x1 = ref_d;
		_x2.zero();
		_x3.zero();
		_x4.zero();
		_T5.zero();
		_initialized = true;
	}

	if (dt < 1e-6f || landed) {
		return;
	}

	// ========== 第一层：旋转向量跟踪 ==========
	// e1 = ref_d - x1 (旋转向量误差)
	Vector3f e1 = ref_d - _x1;
	Vector3f e1_dot = -_x2; // de1/dt = -x2 (ref_d视为常数)

	float e1_len = safeSqrt(e1.norm_squared());
	float e1_dot_scalar = 0.0f;

	if (!isZero(e1_len)) {
		// 标量导数: d|e1|/dt = (e1 · e1_dot) / |e1|
		e1_dot_scalar = (e1(0) * e1_dot(0) + e1(1) * e1_dot(1) + e1(2) * e1_dot(2)) / e1_len;
	}

	AttTdSmoothKpD1 d1 = smoothKp1(e1_len, e1_dot_scalar, _P1, _r1);

	// 构造速度指令 T2 及其导数 T2_dot
	Vector3f T2{};
	Vector3f T2_dot{};

	if (!isZero(e1_len * e1_len)) {
		Vector3f n = e1 * (1.0f / e1_len);
		Vector3f n_dot = (e1_dot * e1_len - e1 * e1_dot_scalar) / (e1_len * e1_len);
		T2 = n * d1.d0;
		T2_dot = n * d1.d1 + n_dot * d1.d0;
	}

	// ========== 第二层：速度跟踪 ==========
	// e2 = T2 - x2 (速度误差)
	Vector3f e2 = T2 - _x2;
	Vector3f e2_dot = T2_dot - _x3;

	float e2_len = safeSqrt(e2.norm_squared());
	float e2_dot_scalar = 0.0f;

	if (!isZero(e2_len)) {
		e2_dot_scalar = (e2(0) * e2_dot(0) + e2(1) * e2_dot(1) + e2(2) * e2_dot(2)) / e2_len;
	}

	AttTdSmoothKpD1 d2 = smoothKp1(e2_len, e2_dot_scalar, _P2, _r2);

	// 构造加速度指令 T3 及其导数 T3_dot
	Vector3f T3{};
	Vector3f T3_dot{};

	if (!isZero(e2_len * e2_len)) {
		Vector3f n = e2 * (1.0f / e2_len);
		Vector3f n_dot = (e2_dot * e2_len - e2 * e2_dot_scalar) / (e2_len * e2_len);
		T3 = n * d2.d0;
		T3_dot = n * d2.d1 + n_dot * d2.d0;
	}

	// ========== 第三层：加速度跟踪 ==========
	// e3 = T3 - x3 (加速度误差)
	Vector3f e3 = T3 - _x3;
	Vector3f e3_dot = T3_dot - _x4;

	float e3_len = safeSqrt(e3.norm_squared());
	float e3_dot_scalar = 0.0f;

	if (!isZero(e3_len)) {
		e3_dot_scalar = (e3(0) * e3_dot(0) + e3(1) * e3_dot(1) + e3(2) * e3_dot(2)) / e3_len;
	}

	AttTdSmoothKpD1 d3 = smoothKp1(e3_len, e3_dot_scalar, _P3, _r3);

	// 构造jerk指令 T4 及其导数 T4_dot
	Vector3f T4{};
	Vector3f T4_dot{};

	if (!isZero(e3_len * e3_len)) {
		Vector3f n = e3 * (1.0f / e3_len);
		Vector3f n_dot = (e3_dot * e3_len - e3 * e3_dot_scalar) / (e3_len * e3_len);
		T4 = n * d3.d0;
		T4_dot = n * d3.d1 + n_dot * d3.d0;
	}

	// ========== 第四层：jerk跟踪 ==========
	// e4 = T4 - x4 (jerk误差)
	Vector3f e4 = T4 - _x4;
	float e4_len = safeSqrt(e4.norm_squared());
	float d4 = smoothKp0(e4_len, _P4, _r4);

	_T5.zero();

	if (!isZero(e4_len)) {
		Vector3f n = e4 * (1.0f / e4_len);
		_T5 = n * d4;
	}

	_T5 += T4_dot;

	// ========== 欧拉积分：更新四层状态 ==========
	// x1' = x2      (角度 += 角速度 × dt)
	// x2' = x3      (角速度 += 角加速度 × dt)
	// x3' = x4      (角加速度 += jerk × dt)
	// x4' = T5      (jerk += snap × dt)
	_x1 += _x2 * dt;
	_x2 += _x3 * dt;
	_x3 += _x4 * dt;
	_x4 += _T5 * dt;
}

float AttitudeTd::smoothKp0(float error, float P, float r)
{
	float x = P / r * error;
	float tanh_x = tanhf(x);
	return r * tanh_x;
}

AttTdSmoothKpD1 AttitudeTd::smoothKp1(float error, float error_dot, float P, float r)
{
	AttTdSmoothKpD1 result;

	float x = P / r * error;
	float tanh_x = tanhf(x);

	result.d0 = r * tanh_x;

	float sech2_x = 1.0f - tanh_x * tanh_x;
	result.d1 = P * sech2_x * error_dot;

	return result;
}

float AttitudeTd::safeSqrt(float x) const
{
	if (x <= 0.0f) {
		return 0.0f;
	}

	return sqrtf(x);
}

bool AttitudeTd::isZero(float x) const
{
	return fabsf(x) < 1e-6f;
}
