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
 * @file AttitudeTd.hpp
 *
 * Fourth-order tracking differentiator for attitude control.
 *
 * Domain: rotation vector (rv = 2 × qe.imag(), where qe = q⁻¹ × qd)
 *   - rv is a 3D vector in body frame, magnitude ≈ rotation angle (rad)
 *   - No Euler angle singularities, no yaw wrap discontinuities
 *   - Time derivatives of rv are body-frame angular velocity/acceleration
 *
 * Input: rotation vector error rv_d [body frame, rad]
 * Output:
 *   x1 — smoothed rotation vector (rad), for attitude P: rate_sp = Kp × x1
 *   x2 — desired body angular velocity (rad/s), direct ESO input
 *   x3 — desired body angular acceleration (rad/s²), direct ESO input
 *   x4 — desired jerk (rad/s³), not used
 *
 * Key advantage: x2/x3 are already in body frame — no W matrix needed.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

/**
 * Helper structure for smooth control gain with first derivative
 */
struct AttTdSmoothKpD1
{
	float d0; ///< Gain value
	float d1; ///< First derivative of gain
};

/**
 * @class AttitudeTd
 *
 * Fourth-order tracking differentiator for 3-axis attitude tracking.
 * Provides smooth tracking of desired Euler angles with velocity,
 * acceleration, and jerk estimation.
 *
 * Layer structure (4-order chain):
 *   x1' = x2       (angle -> angular velocity)
 *   x2' = x3       (angular velocity -> angular acceleration)
 *   x3' = x4       (angular acceleration -> jerk)
 *   x4' = T5       (jerk -> snap, the driving signal)
 */
class AttitudeTd
{
public:
	AttitudeTd() = default;
	~AttitudeTd() = default;

	/**
	 * Set tracking differentiator parameters
	 * @param P1 Gain for angle tracking layer
	 * @param P2 Gain for velocity tracking layer
	 * @param P3 Gain for acceleration tracking layer
	 * @param P4 Gain for jerk tracking layer
	 */
	void setParameters(float P1, float P2, float P3, float P4);

	/**
	 * Reset all states to zero
	 */
	void reset();

	/**
	 * Get smoothed angle state x1
	 * @return smoothed desired angle vector [rad]
	 */
	matrix::Vector3f getX1() const { return _x1; }

	/**
	 * Get angular velocity state x2
	 * @return desired angular velocity vector [rad/s]
	 */
	matrix::Vector3f getX2() const { return _x2; }

	/**
	 * Get angular acceleration state x3
	 * @return desired angular acceleration vector [rad/s²]
	 */
	matrix::Vector3f getX3() const { return _x3; }

	/**
	 * Get jerk state x4
	 * @return desired jerk vector [rad/s³]
	 */
	matrix::Vector3f getX4() const { return _x4; }

	/**
	 * Track a 3D reference signal with the fourth-order tracking differentiator
	 * @param ref_d Reference vector (rotation vector in body frame) [rad]
	 * @param dt Time step for integration (s)
	 * @param landed Landing flag, resets states when true
	 */
	void track(const matrix::Vector3f &ref_d, float dt, bool landed);

private:
	/**
	 * Smooth control gain function (zero-order)
	 * @param error Error magnitude
	 * @param P Gain parameter
	 * @param r Smoothness radius
	 * @return Smooth gain value: r * tanh(P/r * error)
	 */
	float smoothKp0(float error, float P, float r);

	/**
	 * Smooth control gain function with first derivative
	 * @param error Error magnitude
	 * @param error_dot Error rate
	 * @param P Gain parameter
	 * @param r Smoothness radius
	 * @return Smooth gain with derivative
	 */
	AttTdSmoothKpD1 smoothKp1(float error, float error_dot, float P, float r);

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
	float _P1{0.0f}; ///< Angle tracking gain
	float _P2{0.0f}; ///< Velocity tracking gain
	float _P3{0.0f}; ///< Acceleration tracking gain
	float _P4{0.0f}; ///< Jerk tracking gain

	// Smoothness radii (derived from parameters)
	float _r1{1e12f};
	float _r2{1e12f};
	float _r3{1e12f};
	float _r4{1e12f};

	// States
	bool _initialized{false}; ///< Whether x1 has been initialized with first input
	matrix::Vector3f _x1{}; ///< Smoothed rotation vector [rad] (body frame)
	matrix::Vector3f _x2{}; ///< Body angular velocity [rad/s]
	matrix::Vector3f _x3{}; ///< Body angular acceleration [rad/s²]
	matrix::Vector3f _x4{}; ///< Jerk [rad/s³]
	matrix::Vector3f _T5{}; ///< Snap (driving signal for x4)
};
