/**
 * @file   Module_Encoder.h
 * @author XieFField
 * @brief  编码器换算转子角度、总路程
 * @version 1.0
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#pragma once
#ifdef __cplusplus
extern "C"{}
#endif

#ifdef __cplusplus

#include <cstdint>
#include <cmath>
#include <cstddef>
#include "APP_tool.h"

/*此类只做机械转子角度计算，非电机真实角度*/
class Encoder{
public:
    Encoder(uint16_t range = 8192): range_(range){}

    /**
     * @brief 更新编码器原始值，计算当前角度和总路程
     * @param raw_value 编码器原始值
     */
    void update(uint16_t raw_value);

    float getAngle() const { return angle_; }

    float getTotalAngle() const { return total_angle_; }

    float getAngle_redian() const { return angle_ * (PI / 180.0f); }

    float getTotalAngle_redian() const { return total_angle_ * (PI / 180.0f); }

    /**
     * 将当前时刻的总路程重新定位到指定值，重定定义偏移量
     */
    void relocate_totalAngle(float now_totalAngle);

private:
    float angle_ = 0.0f;        // 当前单圈角度(0..360)
    float total_angle_ = 0.0f;  // 总连续角度
    bool  is_init_ = false;
    uint16_t offset_ = 0;       // 初始Raw值（用于扣除初始相位）
    
    // 绝对圈数法核心变量
    int32_t round_cnt_ = 0;     // 旋转圈数计数(整数，无精度损失)
    float last_angle_ = 0.0f;   // 上一帧的单圈角度(0..360)
    float start_angle_ = 0.0f;  // 初始时刻的单圈角度(用于计算相对总程)

    // 大数精度保护
    float precision_offset_ = 0.0f; // 因重置圈数而产生的累积偏置
    uint16_t range_;

    // 若在首帧反馈前调用 relocate，则延迟到首帧初始化时再应用
    bool has_pending_relocate_ = false;
    float pending_relocate_total_angle_ = 0.0f;
};

#endif

#endif