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
     * @brief 数值范围钳位：将 val 限制在 [min_val, max_val] 范围内
     * @param val 输入值
     * @param min_val 最小允许值
     * @param max_val 最大允许值
     * @return T 钳位后的值
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
     * @return f32 角度（单位：度）
     */
    constexpr inline f32 radToDegF32(f32 rad)
    {
        return rad * 360.0f / (2.0f * kPi);
    }

    /**
     * @brief 度转换为弧度
     * @param deg 角度（单位：度）
     * @return f32 弧度（单位：弧度）
     */
    constexpr inline f32 degToRadF32(f32 deg)
    {
        return deg * (2.0f * kPi) / 360.0f;
    }

    /**
     * @brief 计算角度的正弦值（角度制输入）
     * @param deg 角度（单位：度）
     * @return f32 正弦值（-1.0f ~ 1.0f）
     */
    inline f32 sinDegF32(f32 deg)
    {
        f32 sinf_result = sinf(deg * (kPi / 180.0f));

        return sinf_result;
    }

    /**
     * @brief 计算角度的余弦值（角度制输入）
     * @param deg 角度（单位：度）
     * @return f32 余弦值（-1.0f ~ 1.0f）
     */
    inline f32 cosDegF32(f32 deg)
    {
        f32 cosf_result = cosf(deg * (kPi / 180.0f));

        return cosf_result;
    }

    /**
     * @brief 基于时间的一维信号速率限制（无过零处理）
     * @param target  目标值
     * @param current 当前值
     * @param dt      时间步长（单位：秒）
     * @param max_rate 最大变化速率（单位：值/秒）
     * @return        速率限制后的下一时刻值
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
     * @brief 基于时间的一维信号速率限制（分别设置增减速率限制）
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（单位：秒）
     * @param max_inc_rate 最大增加速率（单位：值/秒）
     * @param max_dec_rate 最大减少速率（单位：值/秒）
     * @return             速率限制后的下一时刻值
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
     * @brief 限制角度变化率（角度制，考虑 ±180° 环绕）
     * @param target        目标角度 [-180°, 180°)
     * @param current_angle 当前角度 [-180°, 180°)
     * @param period        时间周期（单位：秒）
     * @param max_rate      最大角度变化率（单位：度/秒）
     * @return              限制后的角度值 [-180°, 180°)
     */
    inline f32 limit1D180AngleRateByTimeF32(f32 target, f32 current, f32 period, f32 max_rate)
    {
        // 计算角度差，考虑 ±180° 环绕
        f32 angle_diff = target - current;

        // 将角度差限制在 [-180°, 180°] 范围内，找到最短路径
        if (angle_diff > 180.0f)
        {
            angle_diff -= 360.0f;
        }
        else if (angle_diff < -180.0f)
        {
            angle_diff += 360.0f;
        }

        // 计算此周期允许的最大角度变化量
        float max_angle_change = max_rate * period;

        // 限制角度变化量不超过最大允许值
        angle_diff = clampValue(angle_diff, -max_angle_change, max_angle_change);

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

    /**
     * @brief 限制角度变化率（弧度制，考虑 ±π 环绕）
     */
    inline f32 limit1DPiAngleRateByTimeF32(f32 target, f32 current, f32 period, f32 max_rate)
    {
        return degToRadF32(limit1D180AngleRateByTimeF32(radToDegF32(target), radToDegF32(current), period, radToDegF32(max_rate)));
    }

    /**
     * @brief 三值取最小值
     * @param a 第一个值
     * @param b 第二个值
     * @param c 第三个值
     * @return T 三个值中的最小值
     */
    template <typename T>
    constexpr inline T minOfThree(const T &a, const T &b, const T &c)
    {
        return std::min(std::min(a, b), c);
    }

    /**
     * @brief 转速（RPM）转换为角速度（弧度/秒）
     * @param rpm 转速（单位：转/分）
     * @return f32 角速度（单位：弧度/秒）
     */
    constexpr inline f32 rpmToRadsF32(f32 rpm)
    {
        return rpm * (2.0f * kPi) / 60.0f;
    }

    /**
     * @brief 角速度（弧度/秒）转换为转速（RPM）
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
     * @brief 正弦波信号生成器（float 参数版）
     * @param time      当前时间（单位：秒）
     * @param amplitude 振幅，默认 1.0f，输出范围 [-amplitude, amplitude]
     * @param frequency 频率，默认 1.0Hz
     * @param phase     相位偏移，默认 0.0f（单位：弧度）
     * @param offset    直流偏置，默认 0.0f
     * @return float    正弦波在当前时刻的值
     */
    inline f32 sineWaveGeneratorF32(f32 time, f32 amplitude = 1.0f, f32 frequency = 1.0f, f32 phase = 0.0f, f32 offset = 0.0f)
    {
        return amplitude * sinf(2.0f * kPi * frequency * time + phase) + offset;
    }

    /**
     * @brief 基于时间的一维信号速率限制（按绝对值方向分离加减速限制）
     *
     * 根据当前值的符号判断加速/减速方向，分别使用对应的速率限制。
     * 当 target 与 current 同号时使用加速限制（max_inc_rate），
     * 当 target 与 current 异号时使用减速限制（max_dec_rate）。
     *
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（单位：秒）
     * @param max_inc_rate 最大增加速率（单位：值/秒）
     * @param max_dec_rate 最大减少速率（单位：值/秒）
     * @return             速率限制后的下一时刻值
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
     * @brief 三值按最大限幅等比缩放
     *
     * 当任一值的绝对值超过其对应的最大限制时，将所有三个值等比缩放，
     * 使得超标的值恰好满足限制，其余值按相同比例缩放。
     *
     * @param val1, val2, val3 输入值（可正可负）
     * @param max1, max2, max3 各值对应的最大绝对值限制（必须非负）
     * @param out1, out2, out3 输出：缩放后的值
     * @return f32 缩放比例：
     *         - 所有值均未超标时返回 1.0f
     *         - 最大值为 0 时返回 -1.0f（错误标志）
     *         - 任一值超标时返回缩放比例（<1.0f）
     */
    inline f32 scaleThreeValuesToMaxF32(f32 val1, f32 val2, f32 val3,
                                        f32 max1, f32 max2, f32 max3,
                                        f32 &out1, f32 &out2, f32 &out3)
    {
        // 安全检查：最大值限制不能为负数
        if (max1 < 0.0f || max2 < 0.0f || max3 < 0.0f)
        {
            return -1.0f;
        }

        // 特例：所有最大限制均为 0 时，直接全输出 0
        if (max1 == 0.0f && max2 == 0.0f && max3 == 0.0f)
        {
            out1 = 0.0f;
            out2 = 0.0f;
            out3 = 0.0f;
            return 0.0f;
        }

        // 计算每个值的"超限比例" = 当前值绝对值 / 对应最大值
        // 比例 > 1 表示超限，比例 ≤ 1 表示合规
        f32 ratio1 = fabsf(val1) / max1;
        f32 ratio2 = fabsf(val2) / max2;
        f32 ratio3 = fabsf(val3) / max3;

        // 找到最大的超限比例
        f32 maxRatio = ratio1;
        if (ratio2 > maxRatio)
            maxRatio = ratio2;
        if (ratio3 > maxRatio)
            maxRatio = ratio3;

        // 确定缩放比例：如果最大比例 > 1，缩放比例为 1 / maxRatio；否则不缩放
        f32 scaleFactor = (maxRatio > 1.0f) ? (1.0f / maxRatio) : 1.0f;

        // 应用缩放比例到所有输出值
        out1 = val1 * scaleFactor;
        out2 = val2 * scaleFactor;
        out3 = val3 * scaleFactor;

        return scaleFactor;
    }

    /**
     * @brief 二维坐标系绕 Z 轴旋转
     * @param x, y    输入坐标
     * @param theta   旋转角度（单位：弧度，逆时针为正）
     * @param x_out, y_out 输出：旋转后的坐标
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
     * @brief 死区映射为零：输入值在死区范围内时输出 0
     * @param value     输入值
     * @param dead_band 死区范围（必须为正数）
     * @return T        处理后的值
     */
    template <typename T>
    constexpr inline T deadZoneToZero(const T &value, const T &dead_band)
    {
        return (value >= dead_band || value <= -dead_band) ? value : 0;
    }

    /**
     * @brief 死区映射到中心：输入值在死区范围内时输出中心值
     * @param value            输入值
     * @param deadband_center  死区中心值
     * @param deadband_radius  死区半径（必须为正数）
     * @return T               处理后的值
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
            return dead_band_center; // 映射到死区中心（摇杆回中逻辑）
        }
    }

    /**
     * @brief 角度归一化到 [0, 360) 度范围
     * @param angle 输入角度（单位：度）
     * @return T    归一化后的角度（单位：度）
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
     * @brief 角度归一化到 [-180, 180) 度范围
     * @param angle 输入角度（单位：度）
     * @return T    归一化后的角度（单位：度）
     */
    template <typename T>
    constexpr inline T normalizeAngleTo180(T angle)
    {
        constexpr T full_circle = static_cast<T>(360);
        constexpr T half_circle = static_cast<T>(180);
        // 先归一化到 [0, 360)
        T normalized = normalizeAngleTo360(angle);
        // 大于等于 180° 的部分转换为负角度
        if (normalized >= half_circle)
        {
            normalized -= full_circle;
        }
        return normalized;
    }

    /**
     * @brief 角度归一化到 [-π, π) 弧度范围
     * @param angle 输入角度（单位：弧度）
     * @return T    归一化后的角度（单位：弧度）
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
     * @brief 快速正弦函数（角度制，CMSIS-DSP 加速版）
     * @param angle 输入角度（单位：度）
     * @return      对应角度的正弦值
     */
    constexpr inline f32 sinDegF32Fast(f32 angle)
    {
        f32 normalized_deg = normalizeAngleTo360(angle);
        f32 rad = degToRadF32(normalized_deg);

        f32 sin_result = arm_sin_f32(rad);

        return sin_result;
    }

    /**
     * @brief 快速余弦函数（角度制，CMSIS-DSP 加速版）
     * @param angle 输入角度（单位：度）
     * @return      对应角度的余弦值
     */
    constexpr inline f32 cosDegF32Fast(f32 angle)
    {
        f32 normalized_deg = normalizeAngleTo360(angle);
        f32 rad = degToRadF32(normalized_deg);

        f32 cos_result = arm_cos_f32(rad);

        return cos_result;
    }

    // ============================================================
    // 底盘运动学工具函数
    // ============================================================

    /**
     * @brief 自身坐标系速度 → 世界坐标系速度变换
     * @param vx      自身坐标系 x 轴速度（单位：米/秒）
     * @param vy      自身坐标系 y 轴速度（单位：米/秒）
     * @param yaw_rad 底盘当前 yaw 角（单位：弧度，逆时针为正）
     * @param out_vx  输出：世界坐标系 x 轴速度（单位：米/秒）
     * @param out_vy  输出：世界坐标系 y 轴速度（单位：米/秒）
     */
    inline void transSpeedBodyToWorld(f32 vx, f32 vy, f32 yaw_rad, f32 &out_vx, f32 &out_vy)
    {
        f32 cos_theta = cosf(yaw_rad);
        f32 sin_theta = sinf(yaw_rad);
        out_vx = vx * cos_theta - vy * sin_theta;
        out_vy = vx * sin_theta + vy * cos_theta;
    }

    /**
     * @brief 世界坐标系速度 → 自身坐标系速度变换
     * @param vx      世界坐标系 x 轴速度（单位：米/秒）
     * @param vy      世界坐标系 y 轴速度（单位：米/秒）
     * @param yaw_rad 底盘当前 yaw 角（单位：弧度，逆时针为正）
     * @param out_vx  输出：自身坐标系 x 轴速度（单位：米/秒）
     * @param out_vy  输出：自身坐标系 y 轴速度（单位：米/秒）
     */
    inline void transSpeedWorldToBody(f32 vx, f32 vy, f32 yaw_rad, f32 &out_vx, f32 &out_vy)
    {
        f32 cos_theta = cosf(yaw_rad);
        f32 sin_theta = sinf(yaw_rad);
        out_vx = vx * cos_theta + vy * sin_theta;
        out_vy = -vx * sin_theta + vy * cos_theta;
    }

    /**
     * @brief 车体目标速度限幅：将各轴速度限制在最大允许范围内
     * @param vel_x, vel_y, omega_z  目标速度（x/y 轴：米/秒，z 轴：弧度/秒）
     * @param max_vel_x, max_vel_y, max_omega_z  各轴最大允许值
     * @param out_vel_x, out_vel_y, out_omega_z  输出：限幅后的速度
     */
    inline void clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z,
                                          f32 max_vel_x, f32 max_vel_y, f32 max_omega_z,
                                          f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
    {
        out_vel_x = clampValue(vel_x, -max_vel_x, max_vel_x);
        out_vel_y = clampValue(vel_y, -max_vel_y, max_vel_y);
        out_omega_z = clampValue(omega_z, -max_omega_z, max_omega_z);
    }

    /**
     * @brief 车体加速度限制：基于时间对各轴速度进行加速度限制
     * @param is_limit      是否启用限制
     * @param tar_vel_x, tar_vel_y, tar_omega_z  目标速度
     * @param cur_vel_x, cur_vel_y, cur_omega_z  当前速度
     * @param period        控制周期（单位：秒）
     * @param max_acc_xy    xy 线加速度上限（单位：米/秒^2）
     * @param max_dec_xy    xy 线减速度上限（单位：米/秒^2）
     * @param max_acc_z     z 角加速度上限（单位：弧度/秒^2）
     * @param max_dec_z     z 角减速度上限（单位：弧度/秒^2）
     * @param out_vel_x, out_vel_y, out_omega_z  输出：限制后的速度
     */
    inline void limitChassisAcceleration(bool is_limit,
                                         f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z,
                                         f32 cur_vel_x, f32 cur_vel_y, f32 cur_omega_z,
                                         f32 period,
                                         f32 max_acc_xy, f32 max_dec_xy,
                                         f32 max_acc_z, f32 max_dec_z,
                                         f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
    {
        if (is_limit)
        {
            out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, cur_vel_x, period, max_acc_xy, max_dec_xy);
            out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, cur_vel_y, period, max_acc_xy, max_dec_xy);
            out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, cur_omega_z, period, max_acc_z, max_dec_z);
        }
        else
        {
            out_vel_x = tar_vel_x;
            out_vel_y = tar_vel_y;
            out_omega_z = tar_omega_z;
        }
    }
} // namespace jia

#endif
