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

Vector3f RateTd::track2(const Vector3f &expect, float dt, const bool landed)
{
	if (landed)
	{
		reset();
	}

	float e_1_n;
	float e_1;

	// ----------- e2 & e2_1 -----------
	Vector3f e2   = expect - _x2;
	Vector3f e2_1 = -_x3;

	float e2_length = safeSqrt(e2.norm_squared());

	e_1_n = e2(0) * e2_1(0)
	      + e2(1) * e2_1(1)
	      + e2(2) * e2_1(2);

	if (!isZero(e2_length)) {
		e_1 = e_1_n / e2_length;
	} else {
		e_1 = 0.0f;
	}

	SmoothKpD1 d2 = smoothKp1(e2_length, e_1, _P2, _r3);

	// ----------- T3 & T3_1 -----------
	Vector3f T3;
	Vector3f T3_1;

	if (!isZero(e2_length * e2_length)) {

		Vector3f n   = e2 * (1.0f / e2_length);
		Vector3f n_1 = (e2_1 * e2_length - e2 * e_1)
		               / (e2_length * e2_length);

		T3   = n * d2.d0;
		T3_1 = n * d2.d1 + n_1 * d2.d0;
	}

	// ----------- acceleration tracking -----------
	Vector3f e3 = T3 - _x3;
	float e3_length = safeSqrt(e3.norm_squared());

	float d3 = smoothKp0(e3_length, _P3, _r4);

	_T4.zero();

	if (!isZero(e3_length)) {
		Vector3f n = e3 * (1.0f / e3_length);
		_T4 = n * d3;
	}

	_T4 += T3_1;

	// ----------- integrate states -----------
	_x1 += _x2 * dt;
	_x2 += _x3 * dt;
	_x3 += _T4 * dt;

	return _x2;
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
