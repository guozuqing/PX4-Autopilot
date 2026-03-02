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
 * @file eso_angular_rate.hpp
 *
 * Extended State Observer (ESO) for angular rate estimation with disturbance rejection.
 * Estimates angular velocity, acceleration and external disturbances with adaptive gain
 * and delay compensation using history buffer.
 *
 * @author ACFly Development Team
 */

#pragma once

#include <stdint.h>

// History buffer length for delay compensation (8 steps)
static constexpr uint8_t ESO_ANGULAR_RATE_HIS_LENGTH = 8;

/**
 * @class EsoAngularRate
 *
 * Extended State Observer for angular rate with features:
 * - Adaptive gain based on error persistence
 * - 8-step history buffer for filter delay compensation
 * - First-order inertia modeling for actuator dynamics
 * - External disturbance estimation
 */
class EsoAngularRate
{
public:
	EsoAngularRate() = default;
	~EsoAngularRate() = default;

	/**
	 * Initialize the ESO with parameters
	 * @param T Time constant for actuator inertia (typical: 0.01-0.1s)
	 * @param b Control gain from input to angular acceleration
	 * @param beta1 Observer gain for angular velocity state
	 * @param beta2 Observer gain for disturbance state
	 * @param ceta1 Adaptive gain coefficient for beta1 (currently unused)
	 * @param ceta2 Adaptive gain coefficient for beta2
	 */
	void setEsoParameters(float beta1, float beta2, float ceta1, float ceta2);

	/**
	 * Set actuator inertia model parameters
	 * @param T Time constant for actuator inertia
	 * @param b Control gain from input to angular acceleration
	 */
	void setActParameters(float T, float b);
	/**
	 * Reset all states to zero
	 */
	void reset();

	/**
	 * Update control input and predict states (open-loop prediction)
	 * Call this after computing control output and before next run() call
	 * @param u Control input (actual motor/actuator output after saturation)
	 */
	void updateControlInput(float u);

	/**
	 * Run the ESO observer (closed-loop correction)
	 * Call this every control cycle with gyroscope measurement
	 * @param v Measured angular velocity from gyroscope (rad/s)
	 * @param dt Time step since last call (s)
	 * @return Estimated disturbance (angular acceleration disturbance)
	 */
	float run(float v, float dt, bool landed);

	/**
	 * Get estimated angular velocity
	 * @return Estimated angular velocity (rad/s)
	 */
	float getEstimatedAngularRate() const { return _z1; }

	/**
	 * Get estimated disturbance
	 * @return Estimated external disturbance (rad/s²)
	 */
	float getEstimatedDisturbance() const { return _z2; }

	/**
	 * Get estimated angular acceleration
	 * @return Total angular acceleration (rad/s²) = inertia response + disturbance
	 */
	float getEstimatedAngularAcceleration() const { return _z_inertia + _z2; }

	/**
	 * Get main power component (inertia response from control input)
	 * @return Angular acceleration from control input (rad/s²)
	 */
	float getEstimatedMainPower() const { return _z_inertia; }

	/**
	 * Get actuator time constant T
	 * @return Time constant T (s)
	 */
	float getT() const { return _T; }

	/**
	 * Get control gain b
	 * @return Control gain b (rad/s² per unit input)
	 */
	float getB() const { return _b; }

private:
	// Actuator inertia model parameters
	float _inv_T{0.0f};           ///< Inverse of time constant (1/T)
	float _T{0.0f};               ///< Time constant for first-order inertia
	float _b{0.0f};               ///< Control gain from u to angular acceleration

	// Observer states
	float _z_inertia{0.0f};       ///< Inertia state (actuator response)
	float _z1{0.0f};              ///< Angular velocity estimate (rad/s)
	float _z2{0.0f};              ///< Disturbance estimate (rad/s²)

	// Control input
	float _u{0.0f};               ///< Control input (motor/actuator command)

	// History buffer for delay compensation (stores last N angular velocity estimates)
	float _his_z1[ESO_ANGULAR_RATE_HIS_LENGTH]{};

	// Error tracking for adaptive gain
	float _last_err{0.0f};        ///< Previous error for computing error derivative
	bool _err_sign{false};        ///< Sign of error derivative (for direction detection)
	float _err_continues_time{0.0f}; ///< Time error continues in same direction

	// Time step
	float _dt{0.0f};              ///< Current time step

	// Observer gains
	float _beta1{0.0f};           ///< Observer gain for angular velocity state
	float _beta2{0.0f};           ///< Observer gain for disturbance state

	// Adaptive gain coefficients
	float _ceta1{0.0f};           ///< Adaptive coefficient for beta1 (currently disabled)
	float _ceta2{0.0f};           ///< Adaptive coefficient for beta2
};
