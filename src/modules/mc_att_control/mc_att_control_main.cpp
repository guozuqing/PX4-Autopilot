/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();

	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	_manual_throttle_maximum.setSlewRate(0.5f);
	// Rate of change 5% per second -> 6 seconds to ramp 30% if hover thrust parameter is off
	_hover_thrust_slew_rate.setSlewRate(0.05f);
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	using math::radians;

	// ---- Attitude control params ----
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	// ---- Rate control params ----
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setFeedbackGains(
		Vector3f(_param_rate_feedr_p1.get(), _param_rate_feedp_p1.get(), _param_rate_feedy_p1.get()),
		Vector3f(_param_rate_feedr_p2.get(), _param_rate_feedp_p2.get(), _param_rate_feedy_p2.get()));

	_rate_control.setEsoGains(
		Vector3f(_param_rate_esor_beta1.get(), _param_rate_esop_beta1.get(), _param_rate_esoy_beta1.get()),
		Vector3f(_param_rate_esor_beta2.get(), _param_rate_esop_beta2.get(), _param_rate_esoy_beta2.get()),
		Vector3f(_param_rate_esor_ceta1.get(), _param_rate_esop_ceta1.get(), _param_rate_esoy_ceta1.get()),
		Vector3f(_param_rate_esor_ceta2.get(), _param_rate_esop_ceta2.get(), _param_rate_esoy_ceta2.get()));

	_rate_control.setActGains(
		Vector3f(_param_rate_act_T_r.get(), _param_rate_act_T_p.get(), _param_rate_act_T_y.get()),
		Vector3f(_param_rate_act_b_r.get(), _param_rate_act_b_p.get(), _param_rate_act_b_y.get()));

	_attitude_control.setTdParameters(
		_param_rate_td_p1.get(), _param_rate_td_p2.get(), _param_rate_td_p3.get(), _param_rate_td_p3.get());

	const int new_mode = _param_rate_eso_ctrl_mode.get();
	_rate_control.setRateCtrlMode(new_mode);

	if (new_mode != _last_rate_ctrl_mode) {
		static const char *mode_desc[] = {
			"PID (all axes)",
			"Roll=ESO, Pitch=PID, Yaw=PID",
			"Roll=PID, Pitch=ESO, Yaw=PID",
			"Roll=PID, Pitch=PID, Yaw=ESO",
			"Roll=ESO, Pitch=ESO, Yaw=PID",
			"Roll=ESO, Pitch=ESO, Yaw=ESO"
		};

		if (new_mode >= 0 && new_mode <= 5) {
			PX4_INFO("Rate ctrl mode changed: %d -> %d [%s]", _last_rate_ctrl_mode, new_mode, mode_desc[new_mode]);
		} else {
			PX4_WARN("Rate ctrl mode changed: %d -> %d [unknown, fallback PID]", _last_rate_ctrl_mode, new_mode);
		}

		_last_rate_ctrl_mode = new_mode;
	}

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	// throttle_stick_input is in range [-1, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // rescale to hover thrust param at 0 stick input
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 or other: rescale to HTE value
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Avoid accumulating absolute yaw error with arming stick gesture
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture || !_heading_good_for_control) {
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, _param_mpc_yaw_expo.get(),
				      _param_mpc_hold_dz.get());
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _yaw_setpoint_stabilized, yaw_stick_input, yaw, dt,
				       _unaided_heading);

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(roll*roll + pitch*pitch)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// we want to fly towards the direction of (roll, pitch)
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// Make sure there's a valid attitude quaternion with no yaw error when yaw is unlocked (NAN)
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		// Modify the setpoints for roll and pitch such that they reflect the user's intention even
		// if a large yaw error(yaw_sp - yaw) is present. In the presence of a yaw error constructing
		// an attitude setpoint from the yaw setpoint will lead to unexpected attitude behaviour from
		// the user's view as the tilt will not be aligned with the heading of the vehicle.

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// Align the desired tilt with the yaw setpoint
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// ==================== Triggered by vehicle_angular_velocity ====================
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// dt based on angular velocity (gyro) timestamp
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		// Update common subscriptions
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			if (_vehicle_status_sub.copy(&_vehicle_status)) {
				_vehicle_type_rotary_wing = (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = _vehicle_status.is_vtol;
				_vtol_in_transition_mode = _vehicle_status.in_transition_mode;
				_vtol_tailsitter = _vehicle_status.is_vtol_tailsitter;

				const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_rates_armed = armed;
				_spooled_up = armed && hrt_elapsed_time(&_vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		// Update hover thrust for stick scaling
		if (_hover_thrust_estimate_sub.updated()) {
			hover_thrust_estimate_s hover_thrust_estimate;

			if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
				if (hover_thrust_estimate.valid) {
					_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

				} else {
					_hover_thrust_estimate = _param_mpc_thr_hover.get();
				}
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// ==================== Attitude control (throttled to 100Hz) ====================
		// v_att is read every cycle, but _attitude_control.update() + TD only run at 100Hz.
		// Between updates, _rates_setpoint holds its last computed value for the 400Hz rate loop.
		vehicle_attitude_s v_att;

		if (_vehicle_attitude_sub.update(&v_att)) {

			const Quatf q{v_att.q};

			const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
			const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

			const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
						  && (is_hovering || is_tailsitter_transition);

			if (run_att_ctrl) {
				// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
				if (_vehicle_control_mode.flag_control_manual_enabled &&
				    !_vehicle_control_mode.flag_control_altitude_enabled &&
				    !_vehicle_control_mode.flag_control_velocity_enabled &&
				    !_vehicle_control_mode.flag_control_position_enabled) {

					generate_attitude_setpoint(q, dt);

				} else {
					_man_roll_input_filter.reset(0.f);
					_man_pitch_input_filter.reset(0.f);
					_yaw_setpoint_stabilized = NAN;
					_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
				}

				// Check for new attitude setpoint
				if (_vehicle_attitude_setpoint_sub.updated()) {
					vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

					if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
					    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

						_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
						_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
						_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
					}
				}

				// Check for a heading reset
				if (_quat_reset_counter != v_att.quat_reset_counter) {
					const Quatf delta_q_reset(v_att.delta_q_reset);
					const float delta_psi = Eulerf(delta_q_reset).psi();

					if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
						_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
					}

					_stick_yaw.ekfResetHandler(delta_psi);

					if (v_att.timestamp > _last_attitude_setpoint) {
						_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
					}

					_quat_reset_counter = v_att.quat_reset_counter;
				}

				// ---- 100Hz throttle: only run attitude controller + TD at 100Hz ----
				const hrt_abstime now_att = hrt_absolute_time();

				if (hrt_elapsed_time(&_last_att_update) >= 10000) { // 10ms = 100Hz
					_last_att_update = now_att;

					// Compute attitude dt based on actual att update interval
					const float att_dt = math::constrain(dt * 4.0f, 0.005f, 0.02f); // ~10ms at 100Hz

					Vector3f rates_sp = _attitude_control.update(q, att_dt, _maybe_landed || _landed);

					autotune_attitude_control_status_s pid_autotune;

					if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
						if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
						    && ((now_att - pid_autotune.timestamp) < 1_s)) {
							rates_sp += Vector3f(pid_autotune.rate_sp);
						}
					}

					// Update rate setpoint from attitude controller output
					_rates_setpoint = rates_sp;

					// publish rate setpoint for logging
					vehicle_rates_setpoint_s rates_setpoint_msg{};
					rates_setpoint_msg.roll = rates_sp(0);
					rates_setpoint_msg.pitch = rates_sp(1);
					rates_setpoint_msg.yaw = rates_sp(2);
					_thrust_setpoint_body.copyTo(rates_setpoint_msg.thrust_body);
					rates_setpoint_msg.timestamp = hrt_absolute_time();
					_vehicle_rates_setpoint_pub.publish(rates_setpoint_msg);
				}

			} else {
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_yaw_setpoint_stabilized = NAN;
				_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
			}
		}

		// ==================== ACRO mode: generate rate setpoint from sticks ====================
		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint_body(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint_body(0) = _thrust_setpoint_body(1) = 0.f;

				// publish rate setpoint for logging
				vehicle_rates_setpoint_s rates_setpoint_msg{};
				rates_setpoint_msg.roll = _rates_setpoint(0);
				rates_setpoint_msg.pitch = _rates_setpoint(1);
				rates_setpoint_msg.yaw = _rates_setpoint(2);
				_thrust_setpoint_body.copyTo(rates_setpoint_msg.thrust_body);
				rates_setpoint_msg.timestamp = hrt_absolute_time();
				_vehicle_rates_setpoint_pub.publish(rates_setpoint_msg);
			}
		}

		// ==================== Rate control ====================
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// Reset integrator when not armed or not rotary wing
			if (!_rates_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// Anti-windup from control allocator saturation
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// Get TD outputs from attitude controller for rate controller
			const Vector3f td_rate_sp = _attitude_control.getTdRateSp();
			const Vector3f td_rate_accel = _attitude_control.getTdRateAccel();

			// Run rate controller (pass gyro timestamp_sample for ESO delay alignment)
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed,
						     td_rate_sp, td_rate_accel, angular_velocity.timestamp_sample);

			// Yaw torque low-pass filter
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// Publish controller status for logging
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// Publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint_body.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// Battery voltage compensation
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		}

		// Throttle slew rates (for manual/stabilized mode)
		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

void MulticopterAttitudeControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
