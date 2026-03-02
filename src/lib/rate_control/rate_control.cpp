/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

//------------------------------------------------------------------------------------------
void RateControl::setFeedbackGains(const Vector3f &FP1, const Vector3f &FP2)
{
	_feedback_p1 = FP1;
	_feedback_p2 = FP2;
}

void RateControl::setEsoGains(const Vector3f &beta1, const Vector3f &beta2, const Vector3f &ceta1, const Vector3f &ceta2)
{
    acfly_eso_roll.setEsoParameters(beta1(0), beta2(0), ceta1(0), ceta2(0));
    acfly_eso_pitch.setEsoParameters(beta1(1), beta2(1), ceta1(1), ceta2(1));
    acfly_eso_yaw.setEsoParameters(beta1(2), beta2(2), ceta1(2), ceta2(2));
}

void RateControl::setActGains(const Vector3f &T, const Vector3f &b)
{
    acfly_eso_roll.setActParameters(T(0), b(0));
    acfly_eso_pitch.setActParameters(T(1), b(1));
    acfly_eso_yaw.setActParameters(T(2), b(2));
}

void RateControl::setTdGains(float P1, float P2, float P3)
{
    // Set TD parameters for Roll/Pitch axes
    rate_td.setTD3Parameters(P1, P2, P3);
}
//------------------------------------------------------------------------------------------

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	//------------------------------------------------------------------------------------------
	acfly_eso_roll.run(rate(0), dt, landed);
	acfly_eso_pitch.run(rate(1), dt, landed);
	acfly_eso_yaw.run(rate(2), dt, landed);

	float angular_rate_ESO_roll = acfly_eso_roll.getEstimatedAngularRate();
	float angular_rate_ESO_pitch = acfly_eso_pitch.getEstimatedAngularRate();
	float angular_rate_ESO_yaw = acfly_eso_yaw.getEstimatedAngularRate();
	float angular_acceleration_ESO_roll = acfly_eso_roll.getEstimatedAngularAcceleration();
	float angular_acceleration_ESO_pitch = acfly_eso_pitch.getEstimatedAngularAcceleration();
	float angular_acceleration_ESO_yaw = acfly_eso_yaw.getEstimatedAngularAcceleration();
	float z_inertia_roll = acfly_eso_roll.getEstimatedMainPower();
	float z_inertia_pitch = acfly_eso_pitch.getEstimatedMainPower();
/* 	float z_inertia_yaw = acfly_eso_yaw.getEstimatedMainPower(); */

	Vector3f torque_setpoint{};

	// ===== Roll axis: TD tracking and feedforward calculation =====
	// Track Roll/Pitch rate setpoint with TD
	rate_td.track2(rate_sp, dt, landed);
	// Get TD states for Roll axis
	Vector3f td_x2 = rate_td.getX2();  // Tracked velocity
	Vector3f td_x3 = rate_td.getX3();  // Tracked acceleration
/* 	Vector3f td_T4 = rate_td.getT4();  // Tracked jerk */
	// Calculate feedforward terms (ACFly: Tv1 and Tv2)
	// Tv1 = Ps * (TD.x2 - angular_rate_ESO) + TD.x3
	float Tv1_roll = _feedback_p1(0) * (td_x2(0) - angular_rate_ESO_roll) + td_x3(0);
	float Tv1_pitch = _feedback_p1(1) * (td_x2(1) - angular_rate_ESO_pitch) + td_x3(1);
	float Tv1_yaw = _feedback_p1(2) * (td_x2(2) - angular_rate_ESO_yaw) + td_x3(2);

	// Tv2 = Ps * (TD.x3 - angular_acceleration_ESO) + TD.T4
	// Since T4 is not directly exposed, we approximate using the tracking dynamics
	// In ACFly this comes from the TD internal calculation
	float Tv2_roll = _feedback_p2(0) * (td_x3(0) - angular_acceleration_ESO_roll)/*  + td_T4(0) */;
	float Tv2_pitch = _feedback_p2(1) * (td_x3(1) - angular_acceleration_ESO_pitch)/*  + td_T4(1) */;
	float Tv2_yaw = _feedback_p2(2) * (td_x3(2) - angular_acceleration_ESO_yaw)/*  + td_T4(2) */;

	// Calculate Ta1 (total angular acceleration command)
	// Ta1 = P2 * (Tv1 - angular_acceleration_ESO) + Tv2
	float Ta1_roll = _feedback_p2(0) * (Tv1_roll - angular_acceleration_ESO_roll) + Tv2_roll;
	float Ta1_pitch = _feedback_p2(1) * (Tv1_pitch - angular_acceleration_ESO_pitch) + Tv2_pitch;
	float Ta1_yaw = _feedback_p2(2) * (Tv1_yaw - angular_acceleration_ESO_yaw) + Tv2_yaw;

	float T_roll = acfly_eso_roll.getT();
	float T_pitch = acfly_eso_pitch.getT();
/* 	float T_yaw = acfly_eso_yaw.getT(); */

	float b_roll = acfly_eso_roll.getB();
	float b_pitch = acfly_eso_pitch.getB();
	float b_yaw = acfly_eso_yaw.getB();

	if (b_roll < 0.001f) { b_roll = 1.0f; }
	if (b_pitch < 0.001f) { b_pitch = 1.0f; }
	if (b_yaw < 0.001f) { b_yaw = 1.0f; }

	// Calculate Roll axis torque using ESO control law with TD feedforward
	if (!landed) {
		// In-flight: use full ESO control law with inertia compensation
		torque_acfly_setpoint(0) = (z_inertia_roll + T_roll * Ta1_roll) / b_roll;
		torque_acfly_setpoint(1) = (z_inertia_pitch + T_pitch * Ta1_pitch) / b_pitch;
/* 		torque_acfly_setpoint(2) = (z_inertia_yaw + T_yaw * Ta1_yaw) / b_yaw; */
		torque_acfly_setpoint(2) = Ta1_yaw / b_yaw;
	} else {
		// Landed: simplified control without inertia compensation
		torque_acfly_setpoint(0) = T_roll * Ta1_roll / b_roll;
		torque_acfly_setpoint(1) = T_pitch * Ta1_pitch / b_pitch;
/* 		torque_acfly_setpoint(2) = T_yaw * Ta1_yaw / b_yaw; */
		torque_acfly_setpoint(2) = Ta1_yaw / b_yaw;
	}

	//------------------------------------------------------------------------------------------
	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	//------------------------------------------------------------------------------------------
/* 	if (_rate_ctrl_mode == 0) {
		// PID mode: all axes use PID output
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	} else {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll uses ESO
		torque_setpoint(1) = torque(1);  // Pitch uses PID
		torque_setpoint(2) = torque(2);  // Yaw uses PID
	} */
	if (_rate_ctrl_mode == 0) {
		// PID mode: all axes use PID output
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	} 
	else if (_rate_ctrl_mode == 1) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll uses ESO
		torque_setpoint(1) = torque(1);  // Pitch uses PID
		torque_setpoint(2) = torque(2);  // Yaw uses PID
	}
	else if (_rate_ctrl_mode == 2) {
		torque_setpoint(0) = torque(0);   // Roll uses ESO
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch uses ESO
		torque_setpoint(2) = torque(2);   // Yaw uses ESO
	}
	else if (_rate_ctrl_mode == 3) {
		torque_setpoint(0) = torque(0);   // Roll uses ESO
		torque_setpoint(1) = torque(1);   // Pitch uses ESO
		torque_setpoint(2) = torque_acfly_setpoint(2);   // Yaw uses ESO
	}
	else if (_rate_ctrl_mode == 4) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll uses ESO
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch uses ESO
		torque_setpoint(2) = torque(2);   // Yaw uses ESO
	}
	else if (_rate_ctrl_mode == 5) {
		torque_setpoint(0) = torque_acfly_setpoint(0);   // Roll uses ESO
		torque_setpoint(1) = torque_acfly_setpoint(1);   // Pitch uses ESO
		torque_setpoint(2) = torque_acfly_setpoint(2);   // Yaw uses ESO
	}
	else {
		// Default to PID if mode is unrecognized
		torque_setpoint = torque;
		target_darate = torque_setpoint;
	}
	//------------------------------------------------------------------------------------------
	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	//------------------------------------------------------------------------------------------
	// Update control input for Roll axis ESO observer
	// Both mode 0 (PID) and mode 1 (hybrid) update ESO for status reporting
	acfly_eso_roll.updateControlInput(torque_setpoint(0));
	acfly_eso_pitch.updateControlInput(torque_setpoint(1));
	acfly_eso_yaw.updateControlInput(torque_setpoint(2));
	//------------------------------------------------------------------------------------------

	return torque_setpoint;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);

	//------------------------------------------------------------------------------------------
	// Report detailed ESO and TD states (Roll axis only)
	rate_ctrl_status.td_trate_roll = rate_td.getX2()(0);
	rate_ctrl_status.td_trate_pitch = rate_td.getX2()(1);
	rate_ctrl_status.td_trate_yaw = rate_td.getX2()(2);
	rate_ctrl_status.td_tdrate_roll = rate_td.getX3()(0);
	rate_ctrl_status.td_tdrate_pitch = rate_td.getX3()(1);
	rate_ctrl_status.td_tdrate_yaw = rate_td.getX3()(2);

	rate_ctrl_status.eso_rate_roll = acfly_eso_roll.getEstimatedAngularRate();
	rate_ctrl_status.eso_rate_pitch = acfly_eso_pitch.getEstimatedAngularRate();
	rate_ctrl_status.eso_rate_yaw = acfly_eso_yaw.getEstimatedAngularRate();
	rate_ctrl_status.eso_drate_roll = acfly_eso_roll.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_drate_pitch = acfly_eso_pitch.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_drate_yaw = acfly_eso_yaw.getEstimatedAngularAcceleration();
	rate_ctrl_status.eso_inertia_roll = acfly_eso_roll.getEstimatedMainPower() / acfly_eso_roll.getB();
	rate_ctrl_status.eso_inertia_pitch = acfly_eso_pitch.getEstimatedMainPower() / acfly_eso_pitch.getB();
	rate_ctrl_status.eso_inertia_yaw = acfly_eso_yaw.getEstimatedMainPower() / acfly_eso_yaw.getB();

	rate_ctrl_status.eso_dis_roll = acfly_eso_roll.getEstimatedDisturbance();
	rate_ctrl_status.eso_dis_pitch = acfly_eso_pitch.getEstimatedDisturbance();
	rate_ctrl_status.eso_dis_yaw = acfly_eso_yaw.getEstimatedDisturbance();

	rate_ctrl_status.rate_cmd_roll = torque_acfly_setpoint(0);
	rate_ctrl_status.rate_cmd_pitch = torque_acfly_setpoint(1);
	rate_ctrl_status.rate_cmd_yaw = torque_acfly_setpoint(2);
	rate_ctrl_status.rate_pidcmd_roll = target_darate(0);
	rate_ctrl_status.rate_pidcmd_pitch = target_darate(1);
	rate_ctrl_status.rate_pidcmd_yaw = target_darate(2);
	//------------------------------------------------------------------------------------------
}
