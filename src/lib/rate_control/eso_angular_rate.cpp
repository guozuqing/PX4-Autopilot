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
	_takeoff_time = 0.0f;
	_his_head = 0;

	for (uint8_t i = 0; i < ESO_HIS_BUF_LEN; ++i) {
		_his_buf[i].timestamp_us = 0;
		_his_buf[i].z1 = 0.0f;
	}
}

void EsoAngularRate::updateControlInput(float u_actual, hrt_abstime timestamp_us)
{
	_u = u_actual;

	// Actuator inertia model: z_inertia' = (b*u_actual - z_inertia) / T
	// Uses ACTUAL applied torque, not desired — prevents model divergence on saturation
	_z_inertia += _dt * _inv_T * (_b * _u - _z_inertia);

	// Open-loop prediction: z1' = z_inertia + z2
	_z1 += _dt * (_z_inertia + _z2);

	// Store (timestamp, z1) in ring buffer for delay compensation
	_his_buf[_his_head].timestamp_us = timestamp_us;
	_his_buf[_his_head].z1 = _z1;
	_his_head = (_his_head + 1) % ESO_HIS_BUF_LEN;
}

float EsoAngularRate::lookupZ1AtTimestamp(hrt_abstime timestamp_us) const
{
	// Find the buffer entry whose timestamp is closest to (but ≤) timestamp_us.
	// Ring buffer may not be full initially, so skip entries with timestamp_us == 0.
	float best_z1 = _z1;
	hrt_abstime best_dt = UINT64_MAX;

	for (uint8_t i = 0; i < ESO_HIS_BUF_LEN; ++i) {
		if (_his_buf[i].timestamp_us == 0) {
			continue;
		}

		// Absolute time difference (handles wrap-around for practical purposes)
		hrt_abstime diff;

		if (_his_buf[i].timestamp_us <= timestamp_us) {
			diff = timestamp_us - _his_buf[i].timestamp_us;

		} else {
			diff = _his_buf[i].timestamp_us - timestamp_us;
		}

		if (diff < best_dt) {
			best_dt = diff;
			best_z1 = _his_buf[i].z1;
		}
	}

	return best_z1;
}

// ==================== ESO closed-loop correction ====================
float EsoAngularRate::run(float v, float dt, bool landed, hrt_abstime timestamp_sample)
{
	if (landed) {
		_z1 = v;
		_z2 = 0.0f;
		_z_inertia = 0.0f;
		_u = 0.0f;
		_last_err = 0.0f;
		_err_sign = false;
		_err_continues_time = 0.0f;
		_dt = dt;
		_takeoff_time = 0.0f;
		_his_head = 0;

		for (uint8_t i = 0; i < ESO_HIS_BUF_LEN; ++i) {
			_his_buf[i].timestamp_us = 0;
			_his_buf[i].z1 = v;
		}

		return _z2;
	}

	// Takeoff protection: freeze z2 for first 1 second after takeoff
	// Ground effect, gravity compensation transient, airflow — huge disturbances
	_takeoff_time += dt;

	if (_takeoff_time < 1.0f) {
		// During takeoff: only track z1, keep z2=0
		_z2 = 0.0f;
		_last_err = 0.0f;
		_err_continues_time = 0.0f;
		_dt = dt;
		return _z2;
	}

	// ---- Module 1: Observation error using timestamp-aligned z1 prediction ----
	// Look up the z1 prediction that matches the gyro's actual sample time.
	// This replaces the old fixed 8-step assumption with proper time alignment.
	float z1_at_sample = lookupZ1AtTimestamp(timestamp_sample);
	float err = v - z1_at_sample;

	// ---- Module 2: Adaptive gain adjustment ----
	// Track whether error stays same-sign for adaptive β₂ scaling
	bool any_saturated = _saturated_positive || _saturated_negative;

	if ((err > 0.0f) != _err_sign) {
		_err_continues_time = 0.0f;
		_err_sign = (err > 0.0f);

	} else if (any_saturated) {
		// Anti-windup: freeze adaptive scaling during saturation
		_err_continues_time = 0.0f;

	} else {
		_err_continues_time += dt;
		_err_continues_time = math::min(_err_continues_time, 2.0f);
	}

	// ---- Module 3: Gain scaling ----
	float max_beta1_scale = (_beta1 > 1e-6f) ? (0.9f / _beta1) : 15.0f;
	float err_continues_time3 = _err_continues_time * _err_continues_time * _err_continues_time;

	float beta1_scale = 1.0f; // β₁ adaptive disabled
	float beta2_scale = 1.0f + _ceta2 * err_continues_time3;

	beta1_scale = math::constrain(beta1_scale, 1.0f, math::min(15.0f, max_beta1_scale));
	beta2_scale = math::constrain(beta2_scale, 1.0f, 5.0f);

	// ---- Module 4: Compute corrections (standard LADRC) ----
	// Standard 2nd-order ESO:
	//   z1_dot = z_inertia + z2 + β₁·err     (done in updateControlInput + correction here)
	//   z2_dot = β₂·err                       (disturbance driven by err, not err-derivative)
	float z1_correction = beta1_scale * _beta1 * err;
	float z2_correction = beta2_scale * _beta2 * err * dt;

	// Rate-limit z2 correction per step
	float z2_correction_limit = Z2_LIMIT * dt;
	z2_correction = math::constrain(z2_correction, -z2_correction_limit, z2_correction_limit);

	// ---- PID-style directional clamping for z2 (anti-windup) ----
	// Same logic as PX4 PID updateIntegral():
	//   positive saturation → don't let z2 grow more positive
	//   negative saturation → don't let z2 grow more negative
	if (_saturated_positive) {
		z2_correction = math::min(z2_correction, 0.0f);
	}

	if (_saturated_negative) {
		z2_correction = math::max(z2_correction, 0.0f);
	}

	// ---- Module 5: Update states ----
	// With timestamp-based lookup, we no longer need to back-propagate corrections
	// through a history queue. The ring buffer entries are predictions that were
	// correct at their time of recording. Only current z1 and z2 need updating.

	_z2 += z2_correction;
	_z2 = math::constrain(_z2, -Z2_LIMIT, Z2_LIMIT);

	_z1 += z1_correction;

	// Save error sign for adaptive gain tracking
	_last_err = err;

	_dt = dt;

	return _z2;
}
