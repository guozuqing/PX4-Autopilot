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

// 历史缓冲区长度: 8步延迟补偿
// 陀螺仪到ESO之间约有8个采样周期的滤波延迟
// ESO用8步前的预测值与当前测量值比较，才能得到准确的观测误差
static constexpr uint8_t ESO_ANGULAR_RATE_HIS_LENGTH = 8;

/**
 * @class EsoAngularRate
 *
 * 单轴扩张状态观测器 (每个轴 Roll/Pitch/Yaw 各一个实例)
 *
 * 内部状态:
 *   z1         —— 角速度估计 (rad/s)
 *   z2         —— 外部扰动估计 (rad/s²)
 *   z_inertia  —— 执行器惯性响应 (rad/s²)，由一阶惯性模型产生
 *   his_z1[8]  —— z1的8步历史队列，用于延迟补偿
 *
 * 观测器模型:
 *   ẑ₁' = z_inertia + z₂           (角速度 = 惯性响应 + 扰动)
 *   z_inertia' = (b×u - z_inertia)/T  (一阶惯性: 力矩→角加速度)
 *   z₂ 由闭环校正驱动              (无模型，纯观测)
 */
class EsoAngularRate
{
public:
	EsoAngularRate() = default;
	~EsoAngularRate() = default;

	/**
	 * 设置ESO观测器增益参数
	 * @param beta1  角速度校正增益 (误差→z1校正量的比例系数)
	 * @param beta2  扰动校正增益 (误差变化量→z2校正量的比例系数)
	 * @param ceta1  β₁自适应系数 (当前被0.0f禁用，始终不生效)
	 * @param ceta2  β₂自适应系数 (误差持续时间³→β₂缩放)
	 */
	void setEsoParameters(float beta1, float beta2, float ceta1, float ceta2);

	/**
	 * 设置执行器惯性模型参数
	 * @param T  执行器一阶惯性时间常数 (s)，典型值 0.01~0.1
	 *           描述电机+螺旋桨从力矩指令到实际角加速度的延迟
	 * @param b  控制增益 (rad/s² / 归一化力矩)
	 *           将归一化力矩转换为角加速度的比例系数
	 */
	void setActParameters(float T, float b);
	/**
	 * 重置所有状态为零
	 * 着陆时调用，防止地面状态下扰动估计累积
	 */
	void reset();

	/**
	 * 开环预测: 用本周期力矩输出预测下一周期状态
	 * 在控制律计算完成后、下一次 run() 之前调用
	 * 内部执行:
	 *   z_inertia += dt/T × (b×u - z_inertia)  // 执行器惯性模型
	 *   z1 += dt × (z_inertia + z2)             // 角速度开环预测
	 * @param u  本周期最终力矩输出 (归一化，经过控制分配后)
	 */
	void updateControlInput(float u);

	/**
	 * 闭环校正: 用陀螺仪测量值修正上一周期的开环预测
	 * 每个控制周期调用一次 (~250Hz)
	 * 内部流程:
	 *   ① 计算观测误差 = 测量值 - 8步前预测值
	 *   ② 自适应增益调整 (误差持续时间越久→β₂越大)
	 *   ③ 回溯更新8步历史队列
	 *   ④ 校正 z1, z2 状态
	 * @param v       陀螺仪测量角速度 (rad/s)
	 * @param dt      时间步长 (s)
	 * @param landed  着陆标志，着陆时重置所有状态
	 * @return 扰动估计值 z2 (rad/s²)
	 */
	float run(float v, float dt, bool landed);

	/**
	 * 获取ESO估计的角速度 z1
	 * 比陀螺仪原始值多了延迟补偿和滤波
	 * @return 角速度估计值 (rad/s)
	 */
	float getEstimatedAngularRate() const { return _z1; }

	/**
	 * 获取ESO估计的外部扰动 z2
	 * 包含风扰、振动、质心偏移等未建模力矩的等效角加速度
	 * @return 扰动估计值 (rad/s²)
	 */
	float getEstimatedDisturbance() const { return _z2; }

	/**
	 * 获取ESO估计的总角加速度 α = z_inertia + z2
	 * 控制律中用于加速度误差计算: Tv2 = P2 × (TD.x3 - α)
	 * @return 总角加速度 (rad/s²) = 执行器惯性响应 + 外部扰动
	 */
	float getEstimatedAngularAcceleration() const { return _z_inertia + _z2; }

	/**
	 * 获取执行器惯性响应 z_inertia
	 * 由一阶惯性模型产生: z_inertia' = (b×u - z_inertia) / T
	 * 控制律中用于扰动补偿: torque = (z_inertia + T×Ta1) / b
	 * @return 执行器惯性响应 (rad/s²)
	 */
	float getEstimatedMainPower() const { return _z_inertia; }

	/**
	 * 获取执行器一阶惯性时间常数 T
	 * 用于控制律的执行器反解: torque = (z_inertia + T×Ta1) / b
	 * @return 时间常数 T (s)
	 */
	float getT() const { return _T; }

	/**
	 * 获取控制增益 b
	 * 用于控制律的执行器反解: torque = (...) / b
	 * @return 控制增益 b (rad/s² / 归一化力矩)
	 */
	float getB() const { return _b; }

private:
	// ===== 执行器惯性模型参数 =====
	float _inv_T{0.0f};           ///< 时间常数的倒数 1/T，避免每次除法
	float _T{0.0f};               ///< 执行器一阶惯性时间常数 (s)
	                              //   典型值: 0.01~0.1s
	                              //   越小→执行器响应越快
	float _b{0.0f};               ///< 控制增益: 归一化力矩 → 角加速度 (rad/s²)

	// ===== 观测器核心状态 =====
	float _z_inertia{0.0f};       ///< 执行器惯性响应 (rad/s²)
	                              //   由一阶惯性模型产生: ż_inertia = (b×u - z_inertia)/T
	                              //   表示当前力矩指令产生的角加速度（含延迟）
	float _z1{0.0f};              ///< 角速度估计 (rad/s)
	                              //   ż1 = z_inertia + z2 (开环预测)
	                              //   闭环时由观测误差校正
	float _z2{0.0f};              ///< 外部扰动估计 (rad/s²)
	                              //   包含风扰/振动/质心偏移等未建模力矩
	                              //   由误差变化量驱动校正

	// ===== 控制输入 =====
	float _u{0.0f};               ///< 控制输入 (归一化力矩)，由控制律输出

	// ===== 延迟补偿历史缓冲区 =====
	//  his_z1[0] = t-8步的z1预测值 ← 与当前陀螺仪测量比较
	//  his_z1[7] = 当前z1值
	//  每次闭环校正时回溯更新所有历史值
	float _his_z1[ESO_ANGULAR_RATE_HIS_LENGTH]{};

	// ===== 自适应增益跟踪 =====
	float _last_err{0.0f};        ///< 上周期观测误差，用于计算误差变化量 z2_err
	bool _err_sign{false};        ///< 误差变化量的符号（检测方向翻转）
	float _err_continues_time{0.0f}; ///< 误差同方向持续时间 (s)
	                              //   时间越久 → β₂缩放越大 → 扰动跟踪越快

	// ===== 时间步长 =====
	float _dt{0.0f};              ///< 当前时间步长 (s)，run()中保存，updateControlInput()中使用

	// ===== 观测器增益 =====
	float _beta1{0.0f};           ///< 角速度校正增益 β₁
	                              //   越大 → z1校正越快，但噪声越大
	float _beta2{0.0f};           ///< 扰动校正增益 β₂
	                              //   越大 → z2校正越快，但更容易振荡

	// ===== 自适应系数 =====
	float _ceta1{0.0f};           ///< β₁自适应系数 (当前被0.0f禁用)
	                              //   设计意图: 误差持续时增大β₁
	                              //   禁用原因: 容易放大噪声
	float _ceta2{0.0f};           ///< β₂自适应系数 (启用)
	                              //   公式: β₂_scale = 1 + ζ₂ × t³
	                              //   t为误差同方向持续时间

	// ===== 安全限幅 =====
	static constexpr float Z2_LIMIT = 50.0f;  ///< z2 扰动估计限幅 (rad/s²)
	                              //   防止模型不匹配或噪声导致z2发散
	                              //   50 rad/s² ≈ 2865 deg/s²，对应极端扰动
};
