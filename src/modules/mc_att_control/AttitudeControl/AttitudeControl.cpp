/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q, const float dt, const bool landed)
{
	// ========== Step 1: TD in REFERENCE domain (pure setpoint, no measurement q) ==========
	// ADRC principle: TD extracts reference signal derivatives and arranges transition.
	// TD input = euler_d (from _attitude_setpoint_q), NOT error signal.
	// This ensures x2/x3 are pure feedforward — independent of measurement noise/dynamics.
	const Eulerf euler_d_angles(_attitude_setpoint_q);
	const Vector3f euler_d_vec(euler_d_angles.phi(), euler_d_angles.theta(), euler_d_angles.psi());
	_attitude_td.track(euler_d_vec, dt, landed);

	const Vector3f x1 = _attitude_td.getX1(); // smoothed desired Euler angles [rad]
	const Vector3f x2 = _attitude_td.getX2(); // desired Euler angle rates     [rad/s]
	const Vector3f x3 = _attitude_td.getX3(); // desired Euler angle accels    [rad/s²]

	// ========== Step 2: Construct smoothed qd from TD x1 ==========
	// qd_s is the smoothed desired quaternion from the TD's first state.
	// When TD converges: x1 → euler_d, so qd_s ≈ _attitude_setpoint_q.
	Quatf qd = Quatf(Eulerf(x1(0), x1(1), x1(2)));

	// ========== Step 3: PX4 standard attitude error with yaw priority ==========
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		qd_red = qd;

	} else {
		qd_red *= q;
	}

	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// ========== Step 4: Attitude P control (PX4 standard) ==========
	const Quatf qe = q.inversed() * qd;
	const Vector3f eq = 2.f * qe.canonical().imag();

	Vector3f rate_p = eq.emult(_proportional_gain);

	if (std::isfinite(_yawspeed_setpoint)) {
		rate_p += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// ========== Step 5: rate_setpoint output (for PID path) ==========
	Vector3f rate_setpoint = rate_p;

	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	// ========== Step 6: W matrix — Euler rates → body rates ==========
	// W = [ 1,      0,       -sinθ     ]
	//     [ 0,    cosφ,   sinφ·cosθ    ]
	//     [ 0,   -sinφ,   cosφ·cosθ    ]
	// body_rate_ff = W × x2,   body_accel_ff = W × x3 + Ẇ × x2
	const float phi   = x1(0);
	const float theta = x1(1);
	const float sp = sinf(phi),   cp = cosf(phi);
	const float st = sinf(theta), ct = cosf(theta);
	const float ct_s = fabsf(ct) < 1e-6f ? copysignf(1e-6f, ct) : ct;

	const float pd = x2(0), qd_e = x2(1), rd = x2(2);
	Vector3f body_rate_ff;
	body_rate_ff(0) = pd                - st    * rd;
	body_rate_ff(1) =       cp * qd_e   + sp * ct_s * rd;
	body_rate_ff(2) =     - sp * qd_e   + cp * ct_s * rd;

	const float pd3 = x3(0), qd3 = x3(1), rd3 = x3(2);
	Vector3f body_accel_ff;
	body_accel_ff(0) = pd3                  - st    * rd3
			   + (       -ct_s * rd) * qd_e;
	body_accel_ff(1) =       cp * qd3       + sp * ct_s * rd3
			   + pd * (-sp * qd_e + cp * ct_s * rd)
			   + qd_e * (-sp * st * rd);
	body_accel_ff(2) =     - sp * qd3       + cp * ct_s * rd3
			   + pd * (-cp * qd_e - sp * ct_s * rd)
			   + qd_e * (-cp * st * rd);

	// ========== Step 7: Feedforward low-pass filter (20 Hz) ==========
	// TD derivatives can amplify high-frequency content.
	// Simple first-order LPF: y += alpha * (x - y), alpha = dt / (dt + 1/(2π×fc))
	const float fc = 20.0f; // cutoff frequency [Hz]
	const float lpf_alpha = dt / (dt + 1.0f / (2.0f * M_PI_F * fc));
	_rate_ff_filtered += lpf_alpha * (body_rate_ff - _rate_ff_filtered);
	_accel_ff_filtered += lpf_alpha * (body_accel_ff - _accel_ff_filtered);

	// ========== Step 8: Feedforward ramp (0→1 over 2 seconds after takeoff) ==========
	// Prevents transient excitation during takeoff when TD hasn't converged.
	if (landed) {
		_ff_ramp_time = 0.0f;
		_rate_ff_filtered.zero();
		_accel_ff_filtered.zero();
	} else {
		_ff_ramp_time += dt;
	}

	const float ramp_duration = 2.0f; // seconds
	const float ff_alpha = math::constrain(_ff_ramp_time / ramp_duration, 0.0f, 1.0f);

	// ========== Step 9: ESO feedforward outputs ==========
	// td_rate_sp    = α × LPF(W×x2) — pure reference feedforward (no measurement q!)
	// td_rate_accel = α × LPF(W×x3 + Ẇ×x2) — pure reference accel feedforward
	// P correction (rate_setpoint) is injected separately in ESO control law via rate_sp.
	_td_rate_sp_body = ff_alpha * _rate_ff_filtered;
	_td_rate_accel_body = ff_alpha * _accel_ff_filtered;

	// Clamp TD outputs to prevent extreme transients (e.g. stick step input)
	for (int i = 0; i < 3; i++) {
		_td_rate_sp_body(i) = math::constrain(_td_rate_sp_body(i), -3.0f, 3.0f);
		_td_rate_accel_body(i) = math::constrain(_td_rate_accel_body(i), -10.0f, 10.0f);
	}

	return rate_setpoint;
}
