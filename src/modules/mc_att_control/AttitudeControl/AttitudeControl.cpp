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
	// ========== 4th-order TD: smooth desired Euler angles ==========
	const Eulerf euler_d_angles(_attitude_setpoint_q);
	const Vector3f euler_d_vec(euler_d_angles.phi(), euler_d_angles.theta(), euler_d_angles.psi());
	_attitude_td.track(euler_d_vec, dt, landed);

	const Vector3f x1 = _attitude_td.getX1(); // smoothed desired angle  [rad]
	const Vector3f x2 = _attitude_td.getX2(); // desired euler rate      [rad/s]
	const Vector3f x3 = _attitude_td.getX3(); // desired euler accel     [rad/s²]

	// ========== W matrix: euler → body frame ==========
	// W = [ 1,      0,          -sinθ       ]
	//     [ 0,    cosφ,    sinφ·cosθ      ]
	//     [ 0,   -sinφ,    cosφ·cosθ      ]
	const float phi   = x1(0);
	const float theta = x1(1);
	const float sp = sinf(phi),   cp = cosf(phi);
	const float st = sinf(theta), ct = cosf(theta);
	const float ct_s = fabsf(ct) < 1e-6f ? copysignf(1e-6f, ct) : ct;

	// trajectory feedforward body rate:  ω_ff = W × x2
	const float pd = x2(0), qd_e = x2(1), rd = x2(2);
	Vector3f body_rate_ff;
	body_rate_ff(0) = pd                    - st    * rd;
	body_rate_ff(1) =       cp * qd_e       + sp * ct_s * rd;
	body_rate_ff(2) =     - sp * qd_e       + cp * ct_s * rd;

	// trajectory acceleration:  α_d = W × x3 + Ẇ × x2
	const float pd3 = x3(0), qd3 = x3(1), rd3 = x3(2);
	_td_rate_accel_body(0) = pd3                    - st    * rd3
				 + (       -ct_s * rd) * qd_e;
	_td_rate_accel_body(1) =       cp * qd3         + sp * ct_s * rd3
				 + pd * (-sp * qd_e + cp * ct_s * rd)
				 + qd_e * (-sp * st * rd);
	_td_rate_accel_body(2) =     - sp * qd3         + cp * ct_s * rd3
				 + pd * (-cp * qd_e - sp * ct_s * rd)
				 + qd_e * (-cp * st * rd);

	// ========== Attitude P control: use TD x1 (smoothed desired angle) ==========
	// x1 is the smoothed euler_d from the 4th-order TD, ensuring P control output
	// is consistent with x2/x3 fed to ESO (all from same trajectory).
	// Requires TD P1>0 (currently 8.0) for x1 to track euler_d.
	Quatf qd = Quatf(Eulerf(x1(0), x1(1), x1(2)));

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

	const Quatf qe = q.inversed() * qd;
	const Vector3f eq = 2.f * qe.canonical().imag();

	// P correction rate (attitude error → body rate)
	Vector3f rate_p = eq.emult(_proportional_gain);

	if (std::isfinite(_yawspeed_setpoint)) {
		rate_p += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// ========== rate_setpoint: P correction only (for PID path) ==========
	// PID uses _rates_setpoint = P correction, same as original PX4 — loop dynamics unchanged
	Vector3f rate_setpoint = rate_p;

	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	// ========== ESO path ==========
	// td_rate_sp = W×x2 (pure trajectory body rate from 4th-order TD)
	// td_rate_accel = W×x3 + Ẇ×x2 (trajectory body accel)
	// These two are physically consistent: td_rate_accel = d(td_rate_sp)/dt
	// P correction (rate_setpoint) is injected separately in ESO Tv1 via rate_sp param
	_td_rate_sp_body = body_rate_ff;

	return rate_setpoint;
}
