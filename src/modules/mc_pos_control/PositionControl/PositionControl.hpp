/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.hpp
 * @brief 位置控制（Position Control）头文件 —— 详细中文注释版
 *
 * 本文件定义了用于多旋翼（MC, MultiCopter）的级联位置控制器。控制链路通常为：
 *   位置误差(P) → 速度外环(PID) → 加速度/推力期望 → 姿态期望（俯仰/横滚）+ 油门
 *
 * 典型用法：
 *   1) setState()     : 输入当前状态（位置、速度、加速度/速度导数、偏航角）。
 *   2) setInputSetpoint() : 输入期望的轨迹/位置/速度/加速度/偏航。
 *   3) update(dt)     : 执行一周期位置-速度-PID计算，得到推力与偏航期望。
 *   4) getLocalPositionSetpoint()/getAttitudeSetpoint() : 读取控制器输出用于下游执行。
 *
 * 设计要点：
 *  - 位置环：使用比例（P）控制，输出速度期望的修正量。
 *  - 速度环：使用 PID 控制（可含积分抗偏置/扰动、微分抑制动态误差）。
 *  - 前馈：若同时给定速度期望与位置期望，则速度期望作为前馈；
 *          当 P 输出（基于误差的调节项）与前馈发生冲突时，P 输出优先。
 *  - NaN 约定：某一维 setpoint 为 NaN 表示该通道未受控（保持开放）；
 *              若同时给定（位置/速度）与推力 setpoint，则以 P/PID 计算结果为准，忽略外部推力 setpoint。
 */

 #pragma once

 #include <lib/mathlib/mathlib.h>
 #include <matrix/matrix/math.hpp>
 #include <uORB/topics/trajectory_setpoint.h>
 #include <uORB/topics/vehicle_attitude_setpoint.h>
 #include <uORB/topics/vehicle_local_position_setpoint.h>

 /**
  * @brief 位置控制状态结构体（控制输入）
  *
  * position     : NED/ENU 下的三轴位置（单位：米），具体坐标系视系统配置而定；
  * velocity     : 三轴速度（单位：m/s）。若未提供可靠加速度估计，可用速度导数代替。
  * acceleration : 三轴加速度（单位：m/s^2），可选；若不可用，控制器内部会用 _vel_dot 近似。
  * yaw          : 偏航角（单位：弧度）。
  */
 struct PositionControlStates {
	 matrix::Vector3f position;     ///< 位置向量 (x, y, z)
	 matrix::Vector3f velocity;     ///< 速度向量 (vx, vy, vz)
	 matrix::Vector3f acceleration; ///< 加速度向量 (ax, ay, az)
	 float yaw;                     ///< 偏航角 (弧度)
 };

 /**
  * @class PositionControl
  * @brief 多旋翼位置控制核心类（MC Position-Control）
  *
  * - 位置环：P 控制
  * - 速度环：PID 控制
  * - 输入：
  *    • 飞行器当前 position/velocity/yaw
  *    • 期望 setpoint（位置/速度/加速度/推力/偏航/偏航速度）
  *    • 约束（速度限制、倾斜角限制、推力上下限等）
  * - 输出：
  *    • 推力向量（机体系或世界系，具体见下游使用）
  *    • 偏航期望
  *
  * 重要行为说明：
  * 1) 同时给定位置与速度 setpoint 时，速度 setpoint 视为前馈。若 P 输出与前馈冲突，P 输出优先。
  * 2) setpoint 为 NaN 代表该通道不受控（例如仅速度控制，不关心位置）。
  * 3) 若同时给定（位置/速度）setpoint 与推力 setpoint，则忽略外部推力 setpoint，改用位置-速度 PID 回路计算推力。
  * 4) 可设置“水平-垂向加速度解耦”，以避免垂向加速对倾角分配的影响（常用于更稳定的姿态分配）。
  */
 class PositionControl
 {
 public:

	 PositionControl() = default;
	 ~PositionControl() = default;

	 /**
	  * @brief 设置位置环增益（P）
	  * @param P 三轴比例增益（x/y/z）
	  *
	  * 建议：水平与垂直轴增益可不同；z 轴常受重力与油门校准影响，需单独调参。
	  */
	 void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	 /**
	  * @brief 设置速度环 PID 增益
	  * @param P 速度环比例增益（x/y/z）
	  * @param I 速度环积分增益（x/y/z）—— 用于抵消长期偏置（如风、传感漂移、油门偏差）
	  * @param D 速度环微分增益（x/y/z）—— 抑制动态误差（对噪声敏感，需滤波/谨慎）
	  */
	 void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	 /**
	  * @brief 设置前馈与位置控制允许的最大速度（速度限幅）
	  * @param vel_horizontal 水平速度上限（m/s）
	  * @param vel_up         垂直上升速度上限（m/s）
	  * @param vel_down       垂直下降速度上限（m/s，通常正数输入，内部按方向处理）
	  *
	  * 说明：限幅同时约束来自前馈与位置环的速度请求，避免过激机动导致饱和或失稳。
	  */
	 void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	 /**
	  * @brief 设置控制器可输出的归一化总推力上下限 [0,1]
	  * @param min 最小推力（如 0 或 0.1，考虑电机空转、支撑力）
	  * @param max 最大推力（如 1 或 0.9，考虑冗余/动态裕度）
	  *
	  * 注：这里使用“归一化推力”，实际与电调/油门曲线映射有关；限制用于防止积分风up与饱和。
	  */
	 void setThrustLimits(const float min, const float max);

	 /**
	  * @brief 设置在垂直推力优先时为水平控制保留的推力裕度
	  * @param margin 保留给水平控制的归一化推力裕度（例如 0.3）
	  *
	  * 背景：当垂向推力接近上限（如紧急爬升）时，仍需留出一定配平能力用于姿态/水平速度控制，
	  *      以减少水平控制完全失能的风险。
	  */
	 void setHorizontalThrustMargin(const float margin);

	 /**
	  * @brief 设置最大倾斜角（弧度）
	  * @param tilt 最大允许偏离水平姿态的倾角（rad）
	  *
	  * 目的：限制姿态分配（由加速度/推力期望映射而来）的倾角，确保飞行器在安全包线内。
	  */
	 void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	 /**
	  * @brief 设置归一化悬停推力（Hover Thrust）
	  * @param hover_thrust 使飞机保持不升不降（水平姿态）所需的归一化推力，范围 [HOVER_THRUST_MIN, HOVER_THRUST_MAX]
	  *
	  * 注：hover_thrust 是重要的系统参数（随负载、电池电压、气压变化），
	  *     合理的范围可避免积分项补偿过多与控制抖动。
	  */
	 void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	 /**
	  * @brief 平滑更新悬停推力（不立即改变输出），通过调整积分项抵消变化
	  *
	  * 动机：直接改变 hover_thrust 会把其跃变传递到控制输出，造成瞬态冲击。
	  *      通过调整积分器，可使输出平滑过渡，提升操稳性。
	  */
	 void updateHoverThrust(const float hover_thrust_new);

	 /**
	  * @brief 设置当前飞行器状态（控制输入）
	  * @param states PositionControlStates 结构体
	  */
	 void setState(const PositionControlStates &states);

	 /**
	  * @brief 传入期望的轨迹/姿态设定（支持前馈）
	  * @param setpoint 轨迹设定（trajectory_setpoint_s），含位置/速度/加速度/偏航/偏航速率/油门等
	  *
	  * 约定：各字段为 NaN 表示该通道未设定（不受控/保持）。
	  */
	 void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	 /**
	  * @brief 执行一次 P（位置）与 PID（速度）控制，更新推力、偏航/偏航速率 setpoint
	  * @param dt 距上次调用的时间步长（s）
	  * @return true：更新成功且输出可执行；false：输入无效（如 NaN 组合不合法）
	  *
	  * 内部流程（典型）：
	  *   _positionControl()  → 基于位置误差的 P 控制，生成速度期望修正
	  *   _velocityControl()  → 基于速度误差的 PID 控制，生成加速度/推力期望
	  *   _accelerationControl() → 根据期望加速度与约束（倾角/推力）得到最终推力与姿态 setpoint
	  */
	 bool update(const float dt);

	 /**
	  * @brief 重置速度环的积分项（XYZ/XY）
	  *
	  * 用途：起飞/着陆/接管/大扰动后清空积累，防止积分风up导致超调或饱和。
	  */
	 void resetIntegral() { _vel_int.setZero(); }
	 void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); }

	 /**
	  * @brief 选择是否在姿态分配时忽略垂向加速度（水平-垂向解耦）
	  * @param val true：假定无垂向加速度，用于避免倾角被 z 加速度占用；false：正常耦合
	  */
	 void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	 /**
	  * @brief 获取控制器输出的本地位置 setpoint（含 PID 输出与前馈后的“最终”设定）
	  * @param local_position_setpoint 输出结构体（vehicle_local_position_setpoint_s）
	  *
	  * 说明：其中的加速度或推力 setpoint 可直接用于姿态控制器进行姿态分配与油门控制。
	  */
	 void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	 /**
	  * @brief 获取姿态 setpoint（由最终加速度 setpoint 反解得到）
	  * @param attitude_setpoint 输出结构体（vehicle_attitude_setpoint_s）
	  *
	  * 说明：姿态控制器跟踪此姿态 setpoint，以实现上一级的速度/位置跟踪。
	  */
	 void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	 /**
	  * @brief 提供一个空的轨迹 setpoint（所有字段为 NaN，时间戳为 0），表示“全部不受控”
	  */
	 static const trajectory_setpoint_s empty_trajectory_setpoint;

 private:
	 // ========================= 参数/常量 =========================
	 // 悬停推力的合法范围（避免错误配置导致控制不可用）
	 static constexpr float HOVER_THRUST_MIN = 0.05f;
	 static constexpr float HOVER_THRUST_MAX = 0.9f;

	 // ========================= 内部校验/子过程 =========================
	 bool _inputValid();                 ///< 校验输入 setpoint/状态是否有效（NaN/范围/组合等）
	 void _positionControl();            ///< 位置 P 控制（生成速度期望修正）
	 void _velocityControl(const float dt); ///< 速度 PID 控制（生成加速度/推力期望）
	 void _accelerationControl();        ///< 期望加速度到推力/姿态的映射与限幅

	 // ========================= 控制器增益 =========================
	 matrix::Vector3f _gain_pos_p; ///< 位置环 P 增益（x/y/z）
	 matrix::Vector3f _gain_vel_p; ///< 速度环 P 增益（x/y/z）
	 matrix::Vector3f _gain_vel_i; ///< 速度环 I 增益（x/y/z）
	 matrix::Vector3f _gain_vel_d; ///< 速度环 D 增益（x/y/z）

	 // ========================= 限制/约束 =========================
	 float _lim_vel_horizontal{};   ///< 水平速度限幅（用于前馈与位置控制）
	 float _lim_vel_up{};           ///< 垂直上升速度限幅
	 float _lim_vel_down{};         ///< 垂直下降速度限幅
	 float _lim_thr_min{};          ///< 允许输出的最小总推力（归一化，区间 [0,1] —— 注：原注释 [-1,0] 为旧表述）
	 float _lim_thr_max{};          ///< 允许输出的最大总推力（归一化，区间 [0,1]）
	 float _lim_thr_xy_margin{};    ///< 垂直优先时为水平控制保留的推力裕度
	 float _lim_tilt{};             ///< 输出姿态允许的最大倾角（弧度）

	 // ========================= 关键参数 =========================
	 float _hover_thrust{}; ///< 悬停推力（归一化，位于 [HOVER_THRUST_MIN, HOVER_THRUST_MAX]）
	 bool _decouple_horizontal_and_vertical_acceleration{true}; ///< 姿态分配时是否忽略 z 加速度的影响（解耦）

	 // ========================= 当前状态 =========================
	 matrix::Vector3f _pos;     /**< 当前本地位置（m） */
	 matrix::Vector3f _vel;     /**< 当前本地速度（m/s） */
	 matrix::Vector3f _vel_dot; /**< 速度导数（m/s^2），当无可靠加速度估计时作为替代 */
	 matrix::Vector3f _vel_int; /**< 速度环积分项（用于抵消稳态误差/外界扰动） */
	 float _yaw{};              /**< 当前偏航角（rad） */

	 // ========================= 期望量（Setpoints） =========================
	 matrix::Vector3f _pos_sp;   /**< 期望位置（m），NaN 表示该轴不受控 */
	 matrix::Vector3f _vel_sp;   /**< 期望速度（m/s），可作前馈，NaN 表示该轴不受控 */
	 matrix::Vector3f _acc_sp;   /**< 期望加速度（m/s^2），用于姿态分配/推力计算 */
	 matrix::Vector3f _thr_sp;   /**< 期望推力（归一化），若提供但同时有位置/速度 setpoint，则会被 PID 结果覆盖 */
	 float _yaw_sp{};            /**< 期望偏航角（rad） */
	 float _yawspeed_sp{};       /**< 期望偏航角速度（rad/s） */
 };

 /* ========================= 附：调参与集成建议 =========================
  *
  * 1) 坐标系一致性
  *    - 请确保传入的 position/velocity 与 setpoint 使用相同坐标系（如 NED），避免符号方向错误。
  *
  * 2) 限幅与饱和
  *    - setVelocityLimits()/setThrustLimits()/setTiltLimit() 应结合机体性能、任务需求设置。
  *    - 推力与倾角限幅会相互影响（姿态分配时的分量投影受限），必要时增大 XY 裕度或降低 z 轴需求。
  *
  * 3) 悬停推力
  *    - 悬停推力不准确会给 z 轴带来稳态误差，建议飞行中在线估计/校正，或使用 updateHoverThrust() 平滑更新。
  *
  * 4) 积分管理
  *    - 起飞、落地、切换控制模式、或大幅变化时，可调用 resetIntegral()/resetIntegralXY() 以避免积分风up。
  *
  * 5) 微分项（D）
  *    - D 项容易放大噪声，实际工程中常加入低通滤波或限制微分增益。
  *
  * 6) 前馈（Feed-Forward）
  *    - 若上层规划器提供平滑的速度/加速度参考，可显著改善跟踪相位滞后与过冲。
  *
  * 7) 水平/垂向解耦
  *    - 对于强 z 机动（快速爬升/下降），启用解耦可避免倾角被 z 加速度“占满”，提升横向可控性。
  *
  * 8) 故障/边界情况
  *    - 若传感异常导致 NaN/Inf，_inputValid() 应返回 false，调用方应做降级/保护（如切返手动/高度保持）。
  *
  * 9) 与姿态控制器接口
  *    - getAttitudeSetpoint() 输出需由姿态控制器执行；实际通道为 roll/pitch/yaw/thrust，
  *      其中 roll/pitch 由加速度期望反解得到，thrust 由 z 分量与 hover_thrust 等合成。
  *
  * 10) 单位与范围
  *    - 所有角度以弧度；速度 m/s；加速度 m/s^2；推力在 [0,1] 归一化范围（具体映射见电调/推进系统）。
  */
