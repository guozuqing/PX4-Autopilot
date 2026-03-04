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

// ==================== ESO 闭环校正（每周期调用一次）====================
// 输入:
//   v       — 陀螺仪测量的当前角速度 (rad/s)
//   dt      — 时间步长 (s)
//   landed  — 着陆标志，着陆时重置所有ESO状态
// 返回:
//   _z2     — 扰动估计值 (rad/s²)
//
// ESO 的核心思想:
//   用「测量值 vs 历史预测值」的偏差来修正内部状态
//   8步历史队列补偿陀螺仪滤波延迟
//   自适应增益应对突变扰动
float EsoAngularRate::run(float v, float dt, bool landed)
{
	// 着陆时清零所有ESO状态，防止地面状态下扰动估计累积
	if (landed)
	{
	    reset();
	}
	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块1: 计算观测误差
	  用陀螺仪测量值与8步前的预测值做比较
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// err: 角速度观测误差 = 陀螺仪测量值 - 8步前的历史估计值
	// 为什么用8步前? 因为陀螺仪到ESO之间有滤波延迟(约8个采样周期)
	// 用延迟匹配的历史值比较，才能得到准确的误差
	float err = v - _his_z1[0];

	// z2_err: 误差的变化量(类似jerk误差)
	// 用于驱动扰动估计 z2 的校正
	float z2_err = err - _last_err;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块2: 自适应增益调整
	  原理: 误差持续同方向越久 → 可能有未补偿的扰动 → 增大增益加快跟踪
	  检测方式: 监测z2_err的符号是否翻转
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// 检测误差方向是否翻转
	if ((z2_err > 0.0f) != _err_sign) {
		_err_continues_time = 0.0f;      // 方向翻转 → 重置持续时间计数
		_err_sign = (z2_err > 0.0f);     // 更新当前符号
	} else {
		_err_continues_time += dt;       // 同方向持续 → 累加时间
		// 安全限幅: 持续时间上限2秒，防止t³项在长时间同向误差下增长过快
		_err_continues_time = math::min(_err_continues_time, 2.0f);
	}


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块3: 计算增益缩放因子
	  公式: scale = 1 + ζ × t³
	  t 越大(误差同方向持续越久) → scale 越大 → 增益越大 → 跟踪越快
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// β₁自适应上限: 防止 β₁×scale 超过0.9导致不稳定
	float max_beta1_scale = 0.9f / _beta1;
	// t³: 持续时间的立方，使增益在初期缓慢增长、后期快速增长
	float err_continues_time3 = _err_continues_time * _err_continues_time * _err_continues_time;

	// β₁缩放: 角速度校正增益的自适应（当前被0.0f禁用，始终=1.0）
	// 禁用原因: β₁自适应容易引入噪声放大，保守设计
	float beta1_scale = 1.0f + 0.0f * _ceta1 * err_continues_time3;

	// β₂缩放: 扰动校正增益的自适应（启用）
	// 误差持续同方向越久 → β₂越大 → 扰动跟踪越快
	float beta2_scale = 1.0f + _ceta2 * err_continues_time3;

	// 限幅防止增益过大导致振荡
	// β₁: 最大15倍或不超过0.9/β₁
	// β₂: 最大5倍
	beta1_scale = math::constrain(beta1_scale, 1.0f, math::min(15.0f, max_beta1_scale));
	beta2_scale = math::constrain(beta2_scale, 1.0f, 5.0f);


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块4: 计算状态校正量
	  z1_correction: 角速度估计的校正量
	  z2_correction: 扰动估计的校正量
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// 角速度校正量 = 自适应β₁ × 观测误差
	float z1_correction = beta1_scale * _beta1 * err;
	// 扰动校正量 = 自适应β₂ × 误差变化量
	float z2_correction = beta2_scale * _beta2 * z2_err;

	// 预扣除z2校正对历史队列的累积影响
	// 原因: 下面的循环会给每个历史值加上 filter_dt × z2_correction
	//       总累积 = (1+2+...+8)×dt×z2_correction = 8×dt×z2_correction (在循环中逐步加)
	//       这里预先减去 8×dt×z2_correction，保证z1_correction的净效果正确
	z1_correction -= z2_correction * ESO_ANGULAR_RATE_HIS_LENGTH * dt;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块5: 回溯更新历史观测队列
	  延迟补偿核心: 不仅校正当前估计，还回溯修正所有历史值
	  队列结构: his_z1[0]=t-8步, his_z1[1]=t-7步, ..., his_z1[7]=当前
	  每个历史值需要:
	    ① 前移一位 (his_z1[k] = his_z1[k+1])
	    ② 加上基础校正量 z1_correction
	    ③ 加上扰动的时间积分 filter_dt × z2_correction
	       (距当前越近，扰动累积影响越大)
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	float filter_dt = dt;  // 累积时间偏移量，从1步开始

	// 遍历历史队列 [0..6]，不含最新值 his_z1[7]
	for (uint8_t k = 0; k < ESO_ANGULAR_RATE_HIS_LENGTH - 1; ++k) {
		// 前移 + 基础校正 + 扰动时间积分
		// filter_dt 依次为: dt, 2dt, 3dt, ..., 7dt
		_his_z1[k] = _his_z1[k + 1] + z1_correction + filter_dt * z2_correction;
		filter_dt += dt;  // 累加时间偏移
	}


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块6: 更新当前观测状态
	  将校正量应用到ESO的核心状态变量
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// 更新扰动估计 z2 (rad/s²)
	_z2 += z2_correction;
	// 安全限幅: 防止z2在模型不匹配或噪声下发散
	_z2 = math::constrain(_z2, -Z2_LIMIT, Z2_LIMIT);

	// 更新角速度估计 z1 (rad/s)
	// 此时 filter_dt = 8×dt，即当前时刻相对于队列起点的时间偏移
	_z1 += z1_correction + filter_dt * z2_correction;

	// 将校正后的 z1 存入历史队列末尾（最新位置）
	_his_z1[ESO_ANGULAR_RATE_HIS_LENGTH - 1] = _z1;

	// 保存当前误差(减去校正量)，供下一周期计算 z2_err 使用
	// 减去校正量是因为 z1 已被校正，下次的 _his_z1[0] 也是校正后的值
	_last_err = err - z1_correction;


	/*━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
	  模块7: 保存参数并返回
	━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
	// 保存时间步长，供后续 updateControlInput()（开环预测）使用
	_dt = dt;

	// 返回扰动估计值 z2 (rad/s²)
	// 控制律中通过 getEstimatedAngularAcceleration() 获取 z_inertia + z2
	return _z2;
}
