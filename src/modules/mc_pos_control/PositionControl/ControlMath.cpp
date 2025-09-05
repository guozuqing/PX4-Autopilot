/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	att_sp.thrust_body[2] = -thr_sp.length();
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

// 根据期望的机体 z 轴方向（body_z，世界系表达）与期望偏航角 yaw_sp，
// 生成完整的姿态设定（四元数）到 att_sp。
// 约定：x 前、y 右、z 下（NED）；旋转矩阵 R_sp 的列向量依次为机体系 {x_b, y_b, z_b} 在世界系中的表达。
void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
    // --- 1) 零向量保护 ---
    // 若传入的 body_z 几乎为零向量，则没有方向意义；设为安全的朝上/朝下基准。
    // 这里将 z 分量设为 1（NED 下 +z 为向下，这里仅用于避免零向量；紧接着会归一化）。
    if (body_z.norm_squared() < FLT_EPSILON) {
        body_z(2) = 1.f;
    }

    // 统一成单位向量，便于后续正交基构造
    body_z.normalize();

    // --- 2) 由偏航角 yaw_sp 构造 XY 平面上的参考方向 y_C ---
    // y_C 是期望偏航的“切向方向”，为 ( -sin(yaw), cos(yaw), 0 )。
    // 直觉：世界系里，以 yaw_sp 指向的“朝前”方向为 x_yaw = [cos, sin, 0]，
    // 与其正交的 y_C = Rz(90°)*x_yaw = [-sin, cos, 0]。
    const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

    // --- 3) 用叉积构造机体 x 轴：x_b = y_C × z_b ---
    // 这样保证 x_b ⟂ z_b 且 x_b 在 yaw 参考平面（受 y_C 引导）。
    // 注意：此处的“×”是右手法则；PX4 中 Vector3f 的 '%' 运算符即为叉积。
    Vector3f body_x = y_C % body_z;

    // --- 4) 倒飞一致性处理（保持机头朝前）---
    // 若 z_b 的 z 分量 < 0，说明机体 z 轴朝“上”（NED 负向），即倒飞状态。
    // 为避免倒飞时机头朝向反转（保持 nose-to-front），将 x_b 取反。
    if (body_z(2) < 0.f) {
        body_x = -body_x;
    }

    // --- 5) 特殊情况：推力完全在水平面（z_b 与世界 z 正交）---
    // 若 |z_b.z| 很小（接近 0），意味着期望推力方向恰在 XY 面上，
    // 这时仅凭 y_C 与 z_b 可能导致 x_b 退化（长度≈0）。为构造一个有效的正交基，
    // 临时将 x_b 设为 (0,0,1)（朝下），后续 yaw 分量实际不会被使用（因为 thrust 在水平面）。
    if (fabsf(body_z(2)) < 0.000001f) {
        body_x.zero();
        body_x(2) = 1.f;
    }

    // 归一化 x_b，确保正交基数值稳定
    body_x.normalize();

    // --- 6) 用叉积得到机体 y 轴：y_b = z_b × x_b ---
    // 这样可保证 {x_b, y_b, z_b} 构成右手正交基（x×y=z）。
    const Vector3f body_y = body_z % body_x;

    // --- 7) 用列向量方式填充期望旋转矩阵 R_sp = [x_b | y_b | z_b] ---
    Dcmf R_sp;
    for (int i = 0; i < 3; i++) {
        R_sp(i, 0) = body_x(i);
        R_sp(i, 1) = body_y(i);
        R_sp(i, 2) = body_z(i);
    }

    // --- 8) 将旋转矩阵转换成四元数，并写入姿态设定 ---
    const Quatf q_sp{R_sp};
    q_sp.copyTo(att_sp.q_d);
}


// 在二维平面（XY）上约束向量和 v0 + v1 的幅值不超过 max。
// 设计目标：优先保留 v0（位置误差产生的“优先量”），只在必要时缩放 v1（叠加/前馈量）。
Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
    // 情况 1：直接相加不超限，返回原和向量
    // 几何含义：||v0 + v1|| <= max，无需约束
	if (Vector2f(v0 + v1).norm() <= max) {
		// 向量未超过最大模长
		return v0 + v1;

	// 情况 2：v0 本身就已达到/超过上限
	// 策略：按 v0 的方向限幅到 max（体现“v0 优先”策略）
	} else if (v0.length() >= max) {
		// v0 的模长已超过 max，直接把 v0 限幅
		return v0.normalized() * max;

	// 情况 3：v0 与 v1 近似相等（大小与方向都非常接近）
	// 在“已超限”的前提下，只能沿 v0 方向把结果限到 max
	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// 两个向量几乎相同（阈值 0.001）
		return v0.normalized() * max;

	// 情况 4：v0 近似为零向量
	// 策略：此时无“优先量”，仅沿 v1 的方向限幅到 max
	} else if (v0.length() < 0.001f) {
		// v0 ≈ 0，则仅用 v1 的方向与模长上限
		return v1.normalized() * max;

	// 情况 5：一般情况（v0 未超限、v0 非零、v0 与 v1 不相等，且 v0+v1 会超限）
	// 思路：保持 v0 不变，只沿 v1 的方向加一个可调标量 s，使得 ||v0 + s*u1|| = max
	//      其中 u1 = v1/||v1|| 为 v1 的单位方向
	} else {
		// 变量含义：
		// vf = 最终输出向量，要求 ||vf|| <= max
		// s  = 沿 u1（v1 的单位方向）叠加的缩放系数，期望 s >= 0
		// u1 = v1 的单位向量
		// 关系：vf = v0 + v1 = v0 + s * u1
		// 约束：||vf|| <= max；在此分支实际通过解方程令 ||vf|| = max

		// 解法：令 u = u1, v = v0，目标：||v + s*u|| = max
		// 展开平方：||v + s*u||^2 = (v + s*u)·(v + s*u)
		//                             = ||v||^2 + 2s(u·v) + s^2||u||^2
		// 因为 u 是单位向量：||u||=1
		// 得：s^2 + 2(u·v)s + (||v||^2 - max^2) = 0
		// 记：m = u·v，c = ||v||^2 - max^2
		// 二次方程：s^2 + 2m s + c = 0
		// 判别式：Δ = (2m)^2 - 4*1*c = 4(m^2 - c)
		// 取“非负根”（只加不减）：s = -m + sqrt(m^2 - c)

		Vector2f u1 = v1.normalized(); // v1 的单位方向
		float m = u1.dot(v0);          // m = u·v = u1·v0，v0 在 u1 方向的投影
		float c = v0.dot(v0) - max * max; // c = ||v0||^2 - max^2

		// 解析解：选择 s = -m + sqrt(m^2 - c)（非负且使 ||v0 + s*u1|| = max）
		float s = -m + sqrtf(m * m - c);

		// 返回叠加后的结果：在 u1 方向加 s，使得最终落在半径为 max 的圆周上
		return v0 + u1 * s;
	}
}


bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.f) {
			// target waypoint is already behind us
			res = line_b;

		} else {
			// target is in front of us
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		res = d; // go directly to line

		// previous waypoint is still in front of us
		if ((sphere_c - line_a) * ab_norm < 0.f) {
			res = line_a;
		}

		// target waypoint is already behind us
		if ((sphere_c - line_b) * ab_norm > 0.f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

} // ControlMath
