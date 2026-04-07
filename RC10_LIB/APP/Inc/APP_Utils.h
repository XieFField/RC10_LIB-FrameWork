#ifndef APP_UTILS_H_
#define APP_UTILS_H_

#include <stdint.h>

#include <algorithm>

#include "arm_math.h"

namespace jia
{
    using u8 = uint8_t;
    using u16 = uint16_t;
    using u32 = uint32_t;
    using u64 = uint64_t;

    using i8 = int8_t;
    using i16 = int16_t;
    using i32 = int32_t;
    using i64 = int64_t;

    using f32 = float;
    using f64 = double;

    constexpr f32 kPi = 3.14159265358979323846f;

    /**
     * @brief 数值范围限制
     * @param val 输入值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return T 限制后的值
     */
    template <typename T>
    constexpr inline T clampValue(const T &val, const T &min_val, const T &max_val)
    {
        if (val < min_val)
            return min_val;
        if (val > max_val)
            return max_val;
        return val;
    }

    /**
     * @brief 弧度转换为度
     * @param rad 弧度（单位：弧度）
     * @return f32 度（单位：度）
     */
    constexpr inline f32 radToDegF32(f32 rad)
    {
        return rad * 360.0f / (2.0f * kPi);
    }

    /**
     * @brief 度转换为弧度
     * @param deg 度（单位：度）
     * @return f32 弧度（单位：弧度）
     */
    constexpr inline f32 degToRadF32(f32 deg)
    {
        return deg * (2.0f * kPi) / 360.0f;
    }

    /**
     * @brief 计算正弦值（角度制）
     * @param deg 角度（度）
     * @return f32 正弦值（-1.0f ~ 1.0f）
     */
    inline f32 sinDegF32(f32 deg)
    {
        f32 sinf_result = sinf(deg * (kPi / 180.0f));

        // if (sinf_result > 1.0f)
        // {
        //     sinf_result = 1.0f;
        // }
        // else if (sinf_result < -1.0f)
        // {
        //     sinf_result = -1.0f;
        // }

        return sinf_result;
    }

    /**
     * @brief 计算余弦值（角度制）
     * @param deg 角度（度）
     * @return f32 余弦值（-1.0f ~ 1.0f）
     */
    inline f32 cosDegF32(f32 deg)
    {
        f32 cosf_result = cosf(deg * (kPi / 180.0f));

        // if (cosf_result > 1.0f)
        // {
        //     cosf_result = 1.0f;
        // }
        // else if (cosf_result < -1.0f)
        // {
        //     cosf_result = -1.0f;
        // }

        return cosf_result;
    }

    /**
     * @brief 基于时间的一维信号速率限幅函数
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param maxRate      最大速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    inline f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 max_rate)
    {
        f32 diff = target - current;
        f32 max_step = max_rate * dt;
        if (diff > max_step)
            return current + max_step;
        else if (diff < -max_step)
            return current - max_step;
        else
            return target;
    }

    /**
     * @brief 基于时间的一维信号速率限幅函数（分离方向）
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param max_inc_rate 最大正速率（单位/秒）
     * @param max_dec_rate 最大负速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    inline f32 limit1DSignalRateByTimeSeparateIncAndDecF32(f32 target, f32 current, f32 dt, f32 max_inc_rate, f32 max_dec_rate)
    {
        f32 diff = target - current;
        f32 max_inc_step = max_inc_rate * dt;
        f32 max_dec_step = max_dec_rate * dt;
        if (diff > max_inc_step)
            return current + max_inc_step;
        else if (diff < -max_dec_step)
            return current - max_dec_step;
        else
            return target;
    }

    /**
     * @brief 限制角度变化率（考虑角度环绕）
     * @param target 目标角度[-180°, 180°)
     * @param current_angle 当前角度[-180°, 180°)
     * @param period 时间周期（秒）
     * @param max_rate 最大角度变化率（度/秒）
     * @return 限制后的角度值[-180°, 180°)
     */
    inline f32 limit1D180AngleRateByTimeF32(f32 target, f32 current, f32 period, f32 max_rate)
    {
        // 计算角度差，考虑环绕情况
        f32 angle_diff = target - current;

        // 调整角度差到 [-180°, 180°] 范围内，找到最短路径
        if (angle_diff > 180.0f)
        {
            angle_diff -= 360.0f;
        }
        else if (angle_diff < -180.0f)
        {
            angle_diff += 360.0f;
        }

        // 计算最大允许的角度变化
        float max_angle_change = max_rate * period;

        // 限制角度变化
        clampValue(angle_diff, -max_angle_change, max_angle_change);

        // 计算新的角度
        f32 new_angle = current + angle_diff;

        // 确保新角度在 [-180°, 180°] 范围内
        if (new_angle > 180.0f)
        {
            new_angle -= 360.0f;
        }
        else if (new_angle < -180.0f)
        {
            new_angle += 360.0f;
        }

        return new_angle;
    }

    inline f32 limit1DPiAngleRateByTimeF32(f32 target, f32 current, f32 period, f32 max_rate)
    {
        return degToRadF32(limit1D180AngleRateByTimeF32(radToDegF32(target), radToDegF32(current), period, radToDegF32(max_rate)));
    }

    /**
     * @brief 三值取小
     * @param a 第一个值
     * @param b 第二个值
     * @param c 第三个值
     * @return T 三值中的最小值
     */
    template <typename T>
    constexpr inline T minOfThree(const T &a, const T &b, const T &c)
    {
        return std::min(std::min(a, b), c);
    }

    /**
     * @brief 转速转换为角速度（单位：弧度/秒）
     * @param rpm 转速（单位：转/分）
     * @return f32 角速度（单位：弧度/秒）
     */
    constexpr inline f32 rpmToRadsF32(f32 rpm)
    {
        return rpm * (2.0f * kPi) / 60.0f;
    }

    /**
     * @brief 角速度转换为转速（单位：转/分）
     * @param omega 角速度（单位：弧度/秒）
     * @return f32 转速（单位：转/分）
     */
    constexpr inline f32 radsToRpmF32(f32 omega)
    {
        return omega * 60.0f / (2.0f * kPi);
    }

    /**
     * @brief 角速度转换为线速度
     * @param omega 角速度（单位：弧度/秒）
     * @param radius 轮子半径（单位：米）
     * @return f32 线速度（单位：米/秒）
     */
    constexpr inline f32 omegaToVelF32(f32 omega, f32 radius)
    {
        return omega * radius;
    }

    /**
     * @brief 线速度转换为角速度
     * @param vel 线速度（单位：米/秒）
     * @param radius 轮子半径（单位：米）
     * @return f32 角速度（单位：弧度/秒）
     */
    constexpr inline f32 velToOmegaF32(f32 vel, f32 radius)
    {
        return vel / radius;
    }

    /**
     * @brief 生成正弦波信号（float类型输出）
     * @param t 输入时间（单位：秒，float类型）
     * @param amplitude 振幅（默认1.0f，输出范围[-amplitude, amplitude]）
     * @param frequency 频率（默认1.0Hz，每秒振荡次数）
     * @param phase 相位偏移（默认0.0f，单位：弧度）
     * @return float 正弦波当前时刻的幅值
     */
    inline f32 sineWaveGeneratorF32(f32 time, f32 amplitude = 1.0f, f32 frequency = 1.0f, f32 phase = 0.0f, f32 offset = 0.0f)
    {
        return amplitude * sinf(2.0f * kPi * frequency * time + phase) + offset;
    }

    /**
     * @brief 基于时间的一维信号速率限幅函数（分离方向）
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param max_inc_rate 最大增加速率（单位/秒）
     * @param max_dec_rate 最大减少速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    inline f32 limit1DSignalRateByTimeSeparateAbsIncAndDecF32(f32 target, f32 current, f32 dt, f32 max_inc_rate, f32 max_dec_rate)
    {
        f32 diff = target - current;
        if (diff > 0.0f && current > 0.0f || diff < 0.0f && current < 0.0f || current == 0.0f)
        {
            f32 max_inc_step = max_inc_rate * dt;
            if (diff > max_inc_step)
                return current + max_inc_step;
            else if (diff < -max_inc_step)
                return current - max_inc_step;
            else
                return target;
        }
        else if (diff < 0.0f && current > 0.0f || diff > 0.0f && current < 0.0f)
        {
            f32 max_dec_step = max_dec_rate * dt;
            if (diff < -max_dec_step)
                return current - max_dec_step;
            else if (diff > max_dec_step)
                return current + max_dec_step;
            else
                return target;
        }
        else
        {
            return target;
        }
    }

    /**
     * @brief 按比例缩放三个数值，确保其绝对值不超过各自的最大值限制
     * @param val1 第一个输入值（可正可负）
     * @param val2 第二个输入值（可正可负）
     * @param val3 第三个输入值（可正可负）
     * @param max1 第一个值的最大绝对值限制（必须为正数）
     * @param max2 第二个值的最大绝对值限制（必须为正数）
     * @param max3 第三个值的最大绝对值限制（必须为正数）
     * @param out1 输出：处理后的第一个值
     * @param out2 输出：处理后的第二个值
     * @param out3 输出：处理后的第三个值
     * @return f32 缩放比例：
     *         - 若所有值都符合限制，返回1.0f
     *         - 若最大值为负数，返回-1.0f
     *         - 若有一个或多个值超限制，返回缩放比例（确保所有值绝对值均≤各自的最大值）
     */
    inline f32 scaleThreeValuesToMaxF32(f32 val1, f32 val2, f32 val3,
                                        f32 max1, f32 max2, f32 max3,
                                        f32 &out1, f32 &out2, f32 &out3)
    {
        // 安全校验：最大值必须为非负数
        if (max1 < 0.0f || max2 < 0.0f || max3 < 0.0f)
        {
            return -1.0f;
        }

        // 特殊情况：若存在最大值为0，直接全部返回0（避免除以0）
        if (max1 == 0.0f && max2 == 0.0f && max3 == 0.0f)
        {
            out1 = 0.0f;
            out2 = 0.0f;
            out3 = 0.0f;
            return 0.0f;
        }

        // 计算每个值的"超限倍数"（当前值绝对值 / 对应最大值）
        // 倍数>1表示超限，倍数≤1表示合规
        f32 ratio1 = fabsf(val1) / max1;
        f32 ratio2 = fabsf(val2) / max2;
        f32 ratio3 = fabsf(val3) / max3;

        // 找到最大的超限倍数
        f32 maxRatio = ratio1;
        if (ratio2 > maxRatio)
            maxRatio = ratio2;
        if (ratio3 > maxRatio)
            maxRatio = ratio3;

        // 确定缩放比例：若最大倍数≤1，缩放比例为1（不缩放）；否则为1/maxRatio
        f32 scaleFactor = (maxRatio > 1.0f) ? (1.0f / maxRatio) : 1.0f;

        // 按比例缩放并保留符号
        out1 = val1 * scaleFactor;
        out2 = val2 * scaleFactor;
        out3 = val3 * scaleFactor;

        return scaleFactor;
    }

    /**
     * @brief 右手坐标系 · 绕 Z 轴旋转坐标
     * @param x,y 输入坐标
     * @param theta 旋转弧度（逆时针为正）
     * @param x_out,y_out 输出旋转后坐标
     */
    constexpr inline void rotateAroundZAxisF32(f32 x, f32 y, f32 theta,
                                               f32 &x_out, f32 &y_out)
    {
        f32 cos_theta = cosf(theta);
        f32 sin_theta = sinf(theta);

        x_out = x * cos_theta + y * sin_theta;
        y_out = -x * sin_theta + y * cos_theta;
    }

    /**
     * @brief 死区处理，将数值在死区范围内映射为0
     * @param value 输入值
     * @param dead_band 死区范围（必须为正数）
     * @return T 处理后的值
     */
    template <typename T>
    constexpr inline T deadZoneToZero(const T &value, const T &dead_band)
    {
        return (value >= dead_band || value <= -dead_band) ? value : 0;
    }

    /**
     * @brief 死区处理，将数值在死区范围内映射为死区中心
     * @param value 输入值
     * @param deadband_center 死区中心（可正可负）
     * @param deadband_radius 死区半径（必须为正数）
     * @return T 处理后的值
     */
    template <typename T>
    constexpr inline T deadZoneToCenter(const T &value, const T &dead_band_center, const T &dead_band_radius)
    {
        if (value > (dead_band_center + dead_band_radius) || value < (dead_band_center - dead_band_radius))
        {
            return value;
        }
        else
        {
            return dead_band_center; // 死区内返回中心点（核心逻辑）
        }
    }

    /**
     * @brief 角度归一化，将角度转换为[0,360)度范围内
     * @param angle 输入角度（单位：度）
     * @return T 归一化后的角度（单位：度）
     */
    template <typename T>
    constexpr inline typename std::enable_if<std::is_same<T, f32>::value, T>::type
    normalizeAngleTo360(T angle)
    {
        constexpr T full_circle = 360.0f;
        T normalized = fmodf(angle, full_circle);
        if (normalized < 0.0f)
        {
            normalized += full_circle;
        }
        return normalized;
    }
    template <typename T>
    constexpr inline typename std::enable_if<std::is_integral<T>::value, T>::type
    normalizeAngleTo360(T angle)
    {
        constexpr T full_circle = static_cast<T>(360);
        T remainder = angle % full_circle;
        return (remainder < 0) ? (remainder + full_circle) : remainder;
    }

    /**
     * @brief 角度归一化，将角度转换为[-180,180)度范围内
     * @param angle 输入角度（单位：度）
     * @return T 归一化后的角度（单位：度）
     */
    template <typename T>
    constexpr inline T normalizeAngleTo180(T angle)
    {
        constexpr T full_circle = static_cast<T>(360);
        constexpr T half_circle = static_cast<T>(180);
        // 先归一化到 [0, 360)
        T normalized = normalizeAngleTo360(angle);
        // 等于和大于180°的部分转换为负数
        if (normalized >= half_circle)
        {
            normalized -= full_circle;
        }
        return normalized;
    }

    /**
     * @brief 角度归一化，将角度转换为[-π,π)弧度范围内
     * @param angle 输入角度（单位：弧度）
     * @return T 归一化后的角度（单位：弧度）
     */
    template <typename T>
    constexpr inline typename std::enable_if<std::is_same<T, f32>::value, T>::type
    normalizeAngleToPi(T angle)
    {
        constexpr T full_circle = 2.0f * kPi;
        T normalized = fmodf(angle, full_circle);
        if (normalized < 0.0f)
        {
            normalized += full_circle;
        }
        return normalized;
    }

    /**
     * @brief 基于角度的正弦函数（单位：度），快速版本
     * @param angle 输入角度（单位：度）
     * @return 对应角度的正弦值
     */
    constexpr inline f32 sinDegF32F(f32 angle)
    {
        f32 normalized_deg = normalizeAngleTo360(angle);
        f32 rad = degToRadF32(normalized_deg);

        f32 sin_result = arm_sin_f32(rad);

        return sin_result;
    }

    /**
     * @brief 基于角度的余弦函数（单位：度），快速版本
     * @param angle 输入角度（单位：度）
     * @return 对应角度的余弦值
     */
    constexpr inline f32 cosDegF32F(f32 angle)
    {
        f32 normalized_deg = normalizeAngleTo360(angle);
        f32 rad = degToRadF32(normalized_deg);

        f32 cos_result = arm_cos_f32(rad);

        return cos_result;
    }
} // namespace jia

#endif
