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
 * @file rate_td.cpp
 */

#include "rate_td.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateTd::setTD3Parameters(float P1, float P2, float P3)
{
	_P1 = P1;
	_P2 = P2;
	_P3 = P3;
}

void RateTd::reset()
{
	_x1.zero();
	_x2.zero();
	_x3.zero();
}

/* Vector2f RateTd::track2(const Vector2f &expect, float dt, const bool landed)
{

	if (landed)
	{
	    reset();
	}

	float e_1_n;
	float e_1;

	// Calculate tracking error and its derivative
	Vector2f e2 = expect - _x2;
	Vector2f e2_1 = -_x3;
	float e2_length = safeSqrt(e2.norm_squared());
	e_1_n = e2(0) * e2_1(0) + e2(1) * e2_1(1);

	if (!isZero(e2_length)) {
		e_1 = e_1_n / e2_length;

	} else {
		e_1 = 0.0f;
	}

	SmoothKpD1 d2 = smoothKp1(e2_length, e_1, _P2, _r3);

	// Calculate T3 and its derivative
	Vector2f T3;
	Vector2f T3_1;

	if (!isZero(e2_length * e2_length)) {
		Vector2f n = e2 * (1.0f / e2_length);
		Vector2f n_1 = (e2_1 * e2_length - e2 * e_1) / (e2_length * e2_length);
		T3 = n * d2.d0;
		T3_1 = n * d2.d1 + n_1 * d2.d0;
	}

	// Calculate acceleration tracking
	Vector2f e3 = T3 - _x3;
	float e3_length = safeSqrt(e3.norm_squared());
	float d3 = smoothKp0(e3_length, _P3, _r4);

	_T4.zero();

	if (!isZero(e3_length)) {
		Vector2f n = e3 * (1.0f / e3_length);
		_T4 = n * d3;
	}

	_T4 += T3_1;

	// Integrate states
	_x1 += _x2 * dt;
	_x2 += _x3 * dt;
	_x3 += _T4 * dt;

	return _x2;
} */

// ==================== TD 跟踪微分器（三层结构）====================
// 将可能含阶跃的期望角速度 expect 平滑为连续轨迹
// 三层状态:
//   _x1 — 位置层（角度积分，本函数未直接使用）
//   _x2 — 速度层（平滑后的期望角速度，控制律直接使用）
//   _x3 — 加速度层（期望角加速度，控制律的前馈项）
//   _T4 — jerk层（加速度的变化率，用于积分更新_x3）
//
// 核心思想: 用非线性增益 smoothKp 实现「远处快追、近处柔停」
//   误差大 → 增益大 → 快速收敛
//   误差小 → 增益小 → 光滑过渡，无超调
Vector3f RateTd::track2(const Vector3f &expect, float dt, const bool landed)
{
	// 着陆时清零所有TD状态，防止地面残留轨迹影响起飞
	if (landed)
	{
		reset();
	}

	float e_1_n;  // e2模长的导数的分子（点积）
	float e_1;    // e2模长的时间导数 d|e2|/dt

	// ========== 第一层：速度跟踪误差 ==========
	// e2: 速度层误差向量 = 期望角速度 - 平滑后的期望角速度
	// e2_1: e2的时间导数 = -_x3（因为 expect 视为常数，dx2/dt = x3）
	Vector3f e2   = expect - _x2;
	Vector3f e2_1 = -_x3;

	// |e2|: 速度误差的模长（三轴合成标量）
	float e2_length = safeSqrt(e2.norm_squared());

	// 计算 |e2| 的时间导数: d|e2|/dt = (e2 · e2_1) / |e2|
	// 分子 e_1_n = e2 · e2_1（点积）
	e_1_n = e2(0) * e2_1(0)
	      + e2(1) * e2_1(1)
	      + e2(2) * e2_1(2);

	if (!isZero(e2_length)) {
		e_1 = e_1_n / e2_length;  // 正常情况: d|e2|/dt
	} else {
		e_1 = 0.0f;               // 误差为零时导数也为零
	}

	// 非线性增益函数 smoothKp1:
	// 输入: 误差模长 |e2|、其导数、增益参数 P2、半径 r3
	// 输出: d2.d0 = Kp(|e2|) × |e2|（增益×误差 = 目标加速度标量）
	//        d2.d1 = 上述的时间导数（用于计算T3_1）
	// 特性: |e2|大时近似线性快追，|e2|小时光滑衰减
	SmoothKpD1 d2 = smoothKp1(e2_length, e_1, _P2, _r3);

	// ========== 标量→向量：构造加速度指令 T3 及其导数 T3_1 ==========
	// smoothKp1 输出的是标量，需要乘以误差方向向量 n 还原为三轴向量
	//   T3   = n × d2.d0   — 目标加速度向量（速度层输出）
	//   T3_1 = d(T3)/dt    — T3的时间导数（用于jerk层前馈）
	//
	// 向量微分: d(n×f)/dt = n×f' + n'×f (乘积法则)
	//   n   = e2 / |e2|          — 误差方向单位向量
	//   n_1 = dn/dt              — 方向变化率（垂直于n的分量）
	Vector3f T3;
	Vector3f T3_1;

	if (!isZero(e2_length * e2_length)) {

		// n: 误差方向单位向量
		Vector3f n   = e2 * (1.0f / e2_length);
		// n_1: n的时间导数，用商法则 d(e2/|e2|)/dt
		Vector3f n_1 = (e2_1 * e2_length - e2 * e_1)
		               / (e2_length * e2_length);

		T3   = n * d2.d0;                     // 目标加速度向量
		T3_1 = n * d2.d1 + n_1 * d2.d0;       // T3的时间导数（乘积法则）
	}

	// ========== 第二层：加速度跟踪 ==========
	// e3: 加速度误差 = 目标加速度T3 - 当前TD加速度_x3
	Vector3f e3 = T3 - _x3;
	float e3_length = safeSqrt(e3.norm_squared());

	// smoothKp0: 非线性增益（只返回标量值，不含导数）
	// 输入: |e3|、增益参数 P3、半径 r4
	// 输出: d3 = Kp(|e3|) × |e3|（jerk反馈量标量）
	float d3 = smoothKp0(e3_length, _P3, _r4);

	// 构造jerk指令 _T4 = 反馈项 + 前馈项
	_T4.zero();

	if (!isZero(e3_length)) {
		Vector3f n = e3 * (1.0f / e3_length);  // 加速度误差方向
		_T4 = n * d3;                          // 反馈项: 方向 × 标量增益
	}

	// 前馈项: T3的时间导数（目标加速度的变化率）
	// 物理含义: 即使加速度误差为零，目标加速度在变化也需要jerk跟随
	_T4 += T3_1;

	// ========== 欧拉积分：更新三层状态 ==========
	// _x1' = _x2      (角度 += 角速度 × dt)
	// _x2' = _x3      (角速度 += 角加速度 × dt)  ← 控制律使用
	// _x3' = _T4      (角加速度 += jerk × dt)    ← 控制律前馈
	_x1 += _x2 * dt;
	_x2 += _x3 * dt;   // 输出: 平滑后的期望角速度
	_x3 += _T4 * dt;   // 输出: 期望角加速度

	return _x2;  // 返回平滑后的期望角速度（控制律第一层使用）
}


float RateTd::smoothKp0(float error, float P, float r)
{
	// Implement smooth control gain function using hyperbolic tangent
	// Original: r * tanh(P/r * error)
	// tanh(x) = 2/(1+e^(-2x)) - 1
	float x = P / r * error;
	float tanh_x = tanhf(x);
	return r * tanh_x;
}

SmoothKpD1 RateTd::smoothKp1(float error, float error_dot, float P, float r)
{
	// Implement smooth control gain with derivative using hyperbolic tangent
	// d0 = r * tanh(P/r * error)
	// d1 = P * (1 - tanh^2(P/r * error)) * error_dot
	SmoothKpD1 result;

	float x = P / r * error;
	float tanh_x = tanhf(x);

	// d0 = r * tanh(x)
	result.d0 = r * tanh_x;

	// Derivative: d(tanh(x))/dx = 1 - tanh^2(x)
	// d1 = P * (1 - tanh^2(x)) * error_dot
	float sech2_x = 1.0f - tanh_x * tanh_x;
	result.d1 = P * sech2_x * error_dot;

	return result;
}

float RateTd::safeSqrt(float x) const
{
	if (x <= 0.0f) {
		return 0.0f;
	}

	return sqrtf(x);
}

bool RateTd::isZero(float x) const
{
	return fabsf(x) < 1e-6f;
}
