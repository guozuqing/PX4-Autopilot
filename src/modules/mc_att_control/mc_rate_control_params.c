/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/**
 * Low pass filter cutoff frequency for yaw torque setpoint
 *
 * Reduces vibrations by lowering high frequency torque caused by rotor acceleration.
 * 0 disables the filter
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_TQ_CUTOFF, 2.f);

/**
 * Rate control mode selection
 *
 * 0: PID only (all axes)
 * 1: Roll=ESO, Pitch=PID, Yaw=PID
 * 2: Roll=PID, Pitch=ESO, Yaw=PID
 * 3: Roll=PID, Pitch=PID, Yaw=ESO
 * 4: Roll=ESO, Pitch=ESO, Yaw=PID
 * 5: Roll=ESO, Pitch=ESO, Yaw=ESO
 *
 * @min 0
 * @max 5
 * @value 0 PID (all axes)
 * @value 1 Roll=ESO, Pitch=PID, Yaw=PID
 * @value 2 Roll=PID, Pitch=ESO, Yaw=PID
 * @value 3 Roll=PID, Pitch=PID, Yaw=ESO
 * @value 4 Roll=ESO, Pitch=ESO, Yaw=PID
 * @value 5 Roll=ESO, Pitch=ESO, Yaw=ESO
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(RATE_CTRL_MODE, 0);

/**
 * ESO observer gain beta1 for angular velocity state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOR_BETA1, 2.0f);

/**
 * ESO observer gain beta2 for disturbance state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 1.0
 * @max 50.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOR_BETA2, 20.0f);

/**
 * ESO adaptive coefficient ceta1 for beta1 (Roll only)
 *
 * Currently disabled (set to 0)
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOR_CETA1, 0.0f);

/**
 * ESO adaptive coefficient ceta2 for beta2 (Roll only)
 *
 * Adaptive gain coefficient for Roll axis disturbance observer
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOR_CETA2, 500.0f);

/**
 * ESO observer gain beta1 for angular velocity state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOP_BETA1, 2.0f);

/**
 * ESO observer gain beta2 for disturbance state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 1.0
 * @max 50.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOP_BETA2, 20.0f);

/**
 * ESO adaptive coefficient ceta1 for beta1 (Roll only)
 *
 * Currently disabled (set to 0)
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOP_CETA1, 0.0f);

/**
 * ESO adaptive coefficient ceta2 for beta2 (Roll only)
 *
 * Adaptive gain coefficient for Roll axis disturbance observer
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOP_CETA2, 5000.0f);

/**
 * ESO observer gain beta1 for angular velocity state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOY_BETA1, 2.0f);

/**
 * ESO observer gain beta2 for disturbance state (Roll only)
 *
 * Observer gain for Roll axis ESO
 *
 * @min 1.0
 * @max 50.0
 * @decimal 2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOY_BETA2, 20.0f);

/**
 * ESO adaptive coefficient ceta1 for beta1 (Roll only)
 *
 * Currently disabled (set to 0)
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOY_CETA1, 0.0f);

/**
 * ESO adaptive coefficient ceta2 for beta2 (Roll only)
 *
 * Adaptive gain coefficient for Roll axis disturbance observer
 *
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ESOY_CETA2, 5000.0f);

/**
 * Roll actuator time constant T
 *
 * Time constant for Roll axis actuator first-order inertia model (ESO only)
 *
 * @min 0.01
 * @max 0.2
 * @decimal 3
 * @unit s
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_T_R, 0.08f);

/**
 * Roll actuator time constant T
 *
 * Time constant for Roll axis actuator first-order inertia model (ESO only)
 *
 * @min 0.01
 * @max 0.2
 * @decimal 3
 * @unit s
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_T_P, 0.08f);

/**
 * Pitch actuator time constant T
 *
 * Time constant for Pitch axis actuator first-order inertia model (ESO only)
 *
 * @min 0.01
 * @max 0.2
 * @decimal 3
 * @unit s
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_T_Y, 0.08f);


/**
 * Roll actuator control gain b
 *
 * Control gain from input to angular acceleration for Roll axis (ESO only)
 * Typical value: 5.5 for Roll
 *
 * @min 1.0
 * @max 250.0
 * @decimal 2
 * @unit rad/s^2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_B_R, 8.0f);

/**
 * Roll actuator control gain b
 *
 * Control gain from input to angular acceleration for Roll axis (ESO only)
 * Typical value: 5.5 for Roll
 *
 * @min 1.0
 * @max 250.0
 * @decimal 2
 * @unit rad/s^2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_B_P, 8.0f);

/**
 * Roll actuator control gain b
 *
 * Control gain from input to angular acceleration for Roll axis (ESO only)
 * Typical value: 5.5 for Roll
 *
 * @min 1.0
 * @max 250.0
 * @decimal 2
 * @unit rad/s^2
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_ACT_B_Y, 8.0f);

/**
 * TD tracking differentiator parameter P1
 *
 * First parameter for TD tracking differentiator (Roll/Pitch)
 * Controls angular velocity tracking gain
 * Typical value: 15 for Roll/Pitch
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_TD_P1, 3.0f);

/**
 * TD tracking differentiator parameter P2
 *
 * Second parameter for TD tracking differentiator (Roll/Pitch)
 * Controls acceleration tracking gain
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_TD_P2, 4.0f);

/**
 * TD tracking differentiator parameter P3
 *
 * Third parameter for TD tracking differentiator (Roll/Pitch)
 * Controls jerk tracking gain
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_TD_P3, 5.0f);

/**
 * Feedback gain P1 for TD feedforward (Roll only)
 *
 * First feedback gain for TD feedforward calculation in Roll axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_R_P1, 3.0f);

/**
 * Feedback gain P1 for TD feedforward (Pitch only)
 *
 * First feedback gain for TD feedforward calculation in Pitch axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_P_P1, 3.0f);

/**
 * Feedback gain P1 for TD feedforward (Yaw only)
 *
 * First feedback gain for TD feedforward calculation in Yaw axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_Y_P1, 3.0f);

/**
 * Feedback gain P2 for TD feedforward (Roll only)
 *
 * Second feedback gain for TD feedforward calculation in Roll axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_R_P2, 3.0f);

/**
 * Feedback gain P2 for TD feedforward (Pitch only)
 *
 * Second feedback gain for TD feedforward calculation in Pitch axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_P_P2, 3.0f);

/**
 * Feedback gain P2 for TD feedforward (Yaw only)
 *
 * Second feedback gain for TD feedforward calculation in Yaw axis ESO control law
 *
 * @min 0.1
 * @max 50.0
 * @decimal 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(RATE_FEED_Y_P2, 3.0f);
