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

    float angle_ = 0.0f; //当前角度
    float total_angle_ = 0.0f; //总角度

    bool is_init_ = false;
    uint16_t offset_ = 0;
    uint16_t last_ = 0;
    int32_t round_cnt_ = 0;
    uint16_t range_;

    float bias_deg_ = 0.0f; //角度偏移(单位:度)
};

#endif

#endif