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
 * 扩张状态观测器 (ESO) —— 用于角速度估计与扰动抑制
 *
 * 功能概述:
 *   1. 估计真实角速度 z1 (补偿陀螺仪滤波延迟)
 *   2. 估计外部扰动 z2 (风/振动/质心偏移等)
 *   3. 估计执行器惯性响应 z_inertia (电机+螺旋桨的一阶惯性)
 *
 * 核心机制:
 *   - 8步历史缓冲区补偿陀螺仪滤波延迟 (~8个采样周期)
 *   - 自适应增益: 误差持续同方向时自动增大β₂加快扰动跟踪
 *   - 一阶惯性模型: 描述执行器从力矩指令到实际角加速度的延迟
 *
 * 调用时序 (每个控制周期):
 *   ① run(v, dt, landed)          —— 闭环校正: 用陀螺仪测量修正预测
 *   ② 控制律计算 torque_setpoint
 *   ③ updateControlInput(torque)   —— 开环预测: 用力矩输出预测下一周期状态
 *
 * @author ACFly Development Team
 */

#pragma once

#include <stdint.h>
#include <drivers/drv_hrt.h>

// Timestamp-based ring buffer length for delay compensation.
// Must be large enough to cover the worst-case gyro filter delay.
// At 400Hz control rate, 16 slots = 40ms max delay coverage.
static constexpr uint8_t ESO_HIS_BUF_LEN = 16;

/**
 * @class EsoAngularRate
 *
 * Single-axis Extended State Observer (one instance per Roll/Pitch/Yaw)
 *
 * Internal states:
 *   z1         — angular rate estimate (rad/s)
 *   z2         — external disturbance estimate (rad/s²)
 *   z_inertia  — actuator inertia response (rad/s²), from first-order model
 *
 * Delay compensation:
 *   Timestamp-indexed ring buffer stores (time_us, z1_prediction) pairs.
 *   On each run(), the gyro's timestamp_sample is used to look up the
 *   z1 prediction corresponding to the actual sample time, replacing
 *   the old fixed 8-step assumption.
 *
 * Anti-windup (PID-inspired):
 *   1. z_inertia model uses ACTUAL applied torque (from control allocator),
 *      not the desired torque. This prevents model divergence when saturated.
 *   2. z2 correction is directionally clamped when saturated:
 *      positive saturation → z2 can only decrease (not grow more positive)
 *      negative saturation → z2 can only increase (not grow more negative)
 *      Same logic as PX4 PID's updateIntegral saturation handling.
 *   3. Adaptive β₂ scaling is frozen during saturation.
 *
 * Observer model:
 *   ẑ₁' = z_inertia + z₂
 *   z_inertia' = (b×u_actual - z_inertia)/T
 *   z₂ driven by closed-loop correction
 */
class EsoAngularRate
{
public:
	EsoAngularRate() = default;
	~EsoAngularRate() = default;

	/**
	 * Set ESO observer gain parameters
	 * @param beta1  angular rate correction gain
	 * @param beta2  disturbance correction gain
	 * @param ceta1  β₁ adaptive coefficient (disabled, always 0)
	 * @param ceta2  β₂ adaptive coefficient
	 */
	void setEsoParameters(float beta1, float beta2, float ceta1, float ceta2);

	/**
	 * Set actuator inertia model parameters
	 * @param T  first-order time constant (s)
	 * @param b  control gain (rad/s² per normalized torque)
	 */
	void setActParameters(float T, float b);

	/**
	 * Reset all states to zero
	 */
	void reset();

	/**
	 * Open-loop prediction using ACTUAL applied torque from control allocator.
	 * Called after control allocation, before next run().
	 *
	 * Critical difference from old design: uses u_actual (what the actuators
	 * really applied) not u_desired (what the controller wanted). When saturated,
	 * u_actual < u_desired, so the inertia model tracks reality, not fantasy.
	 *
	 * Also records (timestamp, z1) into the ring buffer for delay compensation.
	 *
	 * @param u_actual  actual applied torque (normalized, post-allocator clipping)
	 * @param timestamp_us  timestamp of this control cycle (hrt_absolute_time)
	 */
	void updateControlInput(float u_actual, hrt_abstime timestamp_us);

	/**
	 * Set saturation status from control allocator
	 * @param sat_positive  true if positive torque is saturated
	 * @param sat_negative  true if negative torque is saturated
	 */
	void setSaturation(bool sat_positive, bool sat_negative) {
		_saturated_positive = sat_positive;
		_saturated_negative = sat_negative;
	}

	/**
	 * Closed-loop correction using gyro measurement.
	 * Uses timestamp_sample to find the matching z1 prediction in the ring buffer.
	 *
	 * @param v                gyro angular rate (rad/s)
	 * @param dt               time step (s)
	 * @param landed           landing flag
	 * @param timestamp_sample gyro sample timestamp (us) for delay alignment
	 * @return disturbance estimate z2 (rad/s²)
	 */
	float run(float v, float dt, bool landed, hrt_abstime timestamp_sample);

	float getEstimatedAngularRate() const { return _z1; }
	float getEstimatedDisturbance() const { return _z2; }
	float getEstimatedAngularAcceleration() const { return _z_inertia + _z2; }
	float getEstimatedMainPower() const { return _z_inertia; }
	float getT() const { return _T; }
	float getB() const { return _b; }

private:
	/**
	 * Look up the z1 prediction closest to the given timestamp.
	 * Returns the newest entry if timestamp is newer than all buffer entries,
	 * or the oldest if older than all entries.
	 */
	float lookupZ1AtTimestamp(hrt_abstime timestamp_us) const;

	// ===== Actuator model parameters =====
	float _inv_T{0.0f};
	float _T{0.0f};
	float _b{0.0f};

	// ===== Observer core states =====
	float _z_inertia{0.0f};       ///< actuator inertia response (rad/s²)
	float _z1{0.0f};              ///< angular rate estimate (rad/s)
	float _z2{0.0f};              ///< disturbance estimate (rad/s²)

	// ===== Control input =====
	float _u{0.0f};               ///< last actual applied torque (normalized)

	// ===== Timestamp-indexed ring buffer for delay compensation =====
	struct HistoryEntry {
		hrt_abstime timestamp_us{0};
		float z1{0.0f};
	};
	HistoryEntry _his_buf[ESO_HIS_BUF_LEN]{};
	uint8_t _his_head{0};         ///< next write position in ring buffer

	// ===== Adaptive gain tracking =====
	float _last_err{0.0f};
	bool _err_sign{false};
	float _err_continues_time{0.0f};

	// ===== Time step =====
	float _dt{0.0f};
	float _takeoff_time{0.0f};   ///< time since takeoff [s], z2 frozen when < 1.0

	// ===== Observer gains =====
	float _beta1{0.0f};
	float _beta2{0.0f};
	float _ceta1{0.0f};
	float _ceta2{0.0f};

	// ===== Saturation tracking (PID-style directional) =====
	bool _saturated_positive{false};  ///< positive torque saturated
	bool _saturated_negative{false};  ///< negative torque saturated

	// ===== Safety limits =====
	static constexpr float Z2_LIMIT = 20.0f;
};
