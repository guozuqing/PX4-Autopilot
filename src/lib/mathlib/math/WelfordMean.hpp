/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * @file WelfordMean.hpp
 *
 * 使用 Welford 的在线算法来计算均值和方差。
 */

#pragma once

#include <lib/mathlib/mathlib.h>

namespace math
{

template <typename Type = float>
class WelfordMean
{
public:
    // 对于新的数据点，计算新的计数、均值和 M2（方差的一部分）
    bool update(const Type &new_value)
    {
        // 第一个数据点的情况
        if (_count == 0) {
            reset();  // 重置状态
            _count = 1;
            _mean = new_value;  // 初始化均值
            return false;  // 第一个数据点，无法计算方差

        } else if (_count == UINT16_MAX) {  // 防止计数溢出
            // 计数溢出，重置计数器，但保持均值和方差
            _M2 = _M2 / _count;  // 归一化 M2
            _M2_accum = 0;

            _count = 1;  // 重置计数器
        } else {
            _count++;  // 增加数据计数
        }

        // 计算新的均值
        const Type delta{new_value - _mean};
        const Type mean_change = delta / _count;
        _mean = kahanSummation(_mean, mean_change, _mean_accum);  // 使用 Kahan 加法算法避免精度丢失

        // 计算 M2（方差的一部分）
        const Type M2_change = delta * (new_value - _mean);
        _M2 = kahanSummation(_M2, M2_change, _M2_accum);  // 更新 M2

        // 防止浮动精度导致负方差
        _M2 = math::max(_M2, 0.f);

        // 如果均值或方差无效，重置并返回 false
        if (!PX4_ISFINITE(_mean) || !PX4_ISFINITE(_M2)) {
            reset();  // 重置状态
            return false;  // 数据无效
        }

        return valid();  // 如果数据点数大于 2，则返回有效
    }

    // 检查当前是否有足够的数据点来计算方差
    bool valid() const { return _count > 2; }

    // 获取当前数据点数
    auto count() const { return _count; }

    // 重置算法的内部状态
    void reset()
    {
        _count = 0;
        _mean = 0;
        _M2 = 0;

        _mean_accum = 0;  // Kahan 加法的累积误差
        _M2_accum = 0;    // Kahan 加法的累积误差
    }

    // 返回当前均值
    Type mean() const { return _mean; }

    // 返回当前方差，注意：使用无偏估计
    Type variance() const { return _M2 / (_count - 1); }

    // 返回当前标准差（方差的平方根）
    Type standard_deviation() const { return std::sqrt(variance()); }

private:
    // 使用 Kahan 加法算法计算 sum_previous 和 input 的和
    inline Type kahanSummation(Type sum_previous, Type input, Type &accumulator)
    {
        const Type y = input - accumulator;  // 计算补偿值
        const Type t = sum_previous + y;  // 求和
        accumulator = (t - sum_previous) - y;  // 更新累积误差
        return t;  // 返回新的和
    }

    Type _mean{};   // 当前均值
    Type _M2{};     // 方差的 M2 部分（平方和）

    Type _mean_accum{};  // Kahan 加法的均值累积误差
    Type _M2_accum{};    // Kahan 加法的 M2 累积误差

    uint16_t _count{0};  // 数据点的计数
};

} // namespace math
