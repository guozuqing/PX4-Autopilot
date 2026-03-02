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
 * @file rate_td.hpp
 *
 * Third-order tracking differentiator (TD3) for 2D signal processing.
 * Implements a smooth nonlinear tracking differentiator for state estimation.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

/**
 * Helper structure for smooth control gain with first derivative
 */
struct SmoothKpD1
{
	float d0; ///< Gain value
	float d1; ///< First derivative of gain
};

/**
 * @class RateTd
 *
 * Third-order tracking differentiator for 2D vector tracking.
 * Provides smooth tracking with velocity and acceleration estimation.
 */
class RateTd
{
public:
	RateTd() = default;
	~RateTd() = default;

	/**
	 * Set tracking differentiator parameters
	 * @param P1 First parameter (currently unused in track2)
	 * @param P2 Second parameter for tracking gain
	 * @param P3 Third parameter for acceleration gain
	 */
	void setTD3Parameters(float P1, float P2, float P3);

	/**
	 * Reset all states to zero
	 */
	void reset();

	/**
	 * Get position state x1
	 * @return position vector
	 */
	matrix::Vector3f getX1() const { return _x1; }

	/**
	 * Get velocity state x2
	 * @return velocity vector
	 */
	matrix::Vector3f getX2() const { return _x2; }

	/**
	 * Get acceleration state x3
	 * @return acceleration vector
	 */
	matrix::Vector3f getX3() const { return _x3; }

	matrix::Vector3f getT4() const { return _T4; }
	/**
	 * Track a 2D signal with the third-order tracking differentiator
	 * @param expect Expected/desired 2D signal to track
	 * @param dt Time step for integration
	 * @return Current velocity estimate (x2)
	 */
	matrix::Vector3f track2(const matrix::Vector3f &expect, float dt, const bool landed);
private:
	/**
	 * Smooth control gain function (zero-order)
	 * @param error Error magnitude
	 * @param P Parameter
	 * @param r Smoothness factor
	 * @return Smooth gain value
	 */
	float smoothKp0(float error, float P, float r);

	/**
	 * Smooth control gain function with first derivative
	 * @param error Error magnitude
	 * @param error_dot Error rate
	 * @param P Parameter
	 * @param r Smoothness factor
	 * @return Smooth gain with derivative
	 */
	SmoothKpD1 smoothKp1(float error, float error_dot, float P, float r);

	/**
	 * Safe square root function
	 * @param x Input value
	 * @return Square root (returns 0 for negative values)
	 */
	float safeSqrt(float x) const;

	/**
	 * Check if value is approximately zero
	 * @param x Value to check
	 * @return true if value is near zero
	 */
	bool isZero(float x) const;

	// Parameters
	float _P1{0.0f};
	float _P2{0.0f};
	float _P3{0.0f};
	float _r2{1e12f};
	float _r3{1e12f};
	float _r4{1e12f};

	// States
	matrix::Vector3f _x1{}; ///< Position state
	matrix::Vector3f _x2{}; ///< Velocity state
	matrix::Vector3f _x3{}; ///< Acceleration state
	matrix::Vector3f _T4{}; ///< Jerk/control input
};
