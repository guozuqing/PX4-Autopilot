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
 * @file eso_angular_rate.cpp
 *
 * Extended State Observer (ESO) for angular rate estimation implementation.
 *
 * @author ACFly Development Team
 */

#include "eso_angular_rate.hpp"
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

void EsoAngularRate::setEsoParameters(float beta1, float beta2, float ceta1, float ceta2)
{
	_beta1 = beta1;
	_beta2 = beta2;
	_ceta1 = ceta1;
	_ceta2 = ceta2;
}

void EsoAngularRate::setActParameters(float T, float b)
{
	_T = T;
	_inv_T = 1.0f / T;
	_b = b;
}

void EsoAngularRate::reset()
{
	_z1 = 0.0f;
	_z2 = 0.0f;
	_z_inertia = 0.0f;
	_u = 0.0f;
	_last_err = 0.0f;
	_err_sign = false;
	_err_continues_time = 0.0f;
	_dt = 0.0f;

	// Clear history buffer
	for (uint8_t i = 0; i < ESO_ANGULAR_RATE_HIS_LENGTH; ++i) {
		_his_z1[i] = 0.0f;
	}
}

void EsoAngularRate::updateControlInput(float u)
{
	_u = u;

	// Update inertia state: z_inertia' = (1/T) * (b*u - z_inertia)
	// This models a first-order actuator response
	_z_inertia += _dt * _inv_T * (_b * _u - _z_inertia);

	// Open-loop prediction of angular velocity: z1' = z_inertia + z2
	_z1 += _dt * (_z_inertia + _z2);
}

float EsoAngularRate::run(float v, float dt, bool landed)
{
	if (landed)
	{
	    reset();
	}
	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 1: Calculate observation error
	  Computes the difference between measured and estimated values
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// err: Angular velocity observation error = measured - historical estimate (8 steps ago)
	float err = v - _his_z1[0];

	// z2_err: Rate of change of disturbance error (jerk error)
	float z2_err = err - _last_err;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 2: Adaptive gain adjustment
	  Dynamically adjusts gain based on error persistence
	  Principle: Longer error persistence → potential uncompensated disturbance → increase gain
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// Detect error direction change using XOR
	if ((z2_err > 0.0f) != _err_sign) {
		_err_continues_time = 0.0f;      // Reset if error changes direction
		_err_sign = (z2_err > 0.0f);     // Update error sign
	} else {
		_err_continues_time += dt;       // Accumulate time in same direction
	}


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 3: Calculate gain scaling factors
	  Computes adaptive gain scaling and applies saturation
	  Formula: scale = 1 + ceta × t³
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	float max_beta1_scale = 0.9f / _beta1;  // Maximum beta1 scaling limit
	float err_continues_time3 = _err_continues_time * _err_continues_time * _err_continues_time;

	// beta1_scale: Angular velocity gain scaling (currently disabled with 0* multiplier)
	float beta1_scale = 1.0f + 0.0f * _ceta1 * err_continues_time3;

	// beta2_scale: Disturbance gain scaling, increases with error persistence time
	float beta2_scale = 1.0f + _ceta2 * err_continues_time3;

	// Apply saturation limits to prevent oscillation
	beta1_scale = math::constrain(beta1_scale, 1.0f, math::min(15.0f, max_beta1_scale));
	beta2_scale = math::constrain(beta2_scale, 1.0f, 5.0f);


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 4: Calculate state correction values
	  Computes correction amounts based on errors
	  z1_correction: Angular velocity correction
	  z2_correction: Disturbance correction
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	float z1_correction = beta1_scale * _beta1 * err;
	float z2_correction = beta2_scale * _beta2 * z2_err;

	// Subtract z2's accumulated effect on historical values
	// Reason: The loop below will add z2's time integral to each historical value
	// We subtract it here in advance to avoid double-counting
	z1_correction -= z2_correction * ESO_ANGULAR_RATE_HIS_LENGTH * dt;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 5: Update historical observation queue
	  Retroactively updates all historical values for delay compensation
	  Principle: History queue [t-8, t-7, ..., t-1, t]
	             Each value needs correction plus z2's integral contribution
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	float filter_dt = dt;  // Cumulative time offset, starts at 1 step

	// Iterate through history buffer (excluding the newest value his_z1[7])
	for (uint8_t k = 0; k < ESO_ANGULAR_RATE_HIS_LENGTH - 1; ++k) {
		// his_z1[k] = his_z1[k+1] (shift) + z1_correction (base) + filter_dt*z2_correction (integral)
		// filter_dt represents time difference from this point to current: [dt, 2dt, 3dt, ..., 7dt]
		_his_z1[k] = _his_z1[k + 1] + z1_correction + filter_dt * z2_correction;
		filter_dt += dt;  // Accumulate time difference
	}


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 6: Update current observation state
	  Applies corrections to observer's internal states
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	_z2 += z2_correction;  // Update disturbance estimate

	// Update angular velocity estimate (filter_dt now equals 8*dt)
	_z1 += z1_correction + filter_dt * z2_correction;

	// Save current z1 to the end of history queue
	_his_z1[ESO_ANGULAR_RATE_HIS_LENGTH - 1] = _z1;

	// Save current error (minus correction) for next z2_err calculation
	_last_err = err - z1_correction;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  Module 7: Save parameters and return result
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// Save time step for updateControlInput() to use
	_dt = dt;

	// Return estimated disturbance (angular acceleration disturbance)
	return _z2;
}
