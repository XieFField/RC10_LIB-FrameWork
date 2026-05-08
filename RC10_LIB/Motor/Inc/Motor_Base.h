/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 电机控制基类
 * @version 1.0
 * @date 2025-09-16
 */
#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#pragma once

#include "BSP_CanFrame.h"
#include <cstdint>
#include <cstddef>

class fdCANbus; // 前向声明

// 电机控制基类，提供通用的电机控制接口。
// 具体电机类型（DJI、VESC等）通过继承实现各自的通信协议。
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    }
    virtual ~Motor_Base(){};

    // 目标值设定接口
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};
    // 刹车设定接口，brake_current 为刹车电流（单位：mA）
    virtual void setBrake(float brake_current)
    {
        (void)brake_current;
    };

    // 电机状态更新（由派生类实现，在CAN收发循环中调用）
    virtual void update(){};

    // 反馈值读取接口（返回内部缓存值）
    virtual float getRPM() const { return rpm_; }
    virtual float getCurrent() const { return current_; }
    virtual float getAngle() const { return angle_; }
    virtual float getTotalAngle() const { return totalAngle_; }

    /**
     * @brief 将电机目标值打包为CAN帧
     * @param outFrames 输出的CAN帧数组
     * @param maxFrames 最大帧数
     * @return 实际打包的帧数
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    /**
     * @brief 从CAN帧解析电机反馈数据
     * @param cf 输入的CAN帧
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief 检查CAN帧是否匹配此电机（用于CAN接收分发）
     * @param cf 待检查的CAN帧
     * @return true 匹配此电机，false 不匹配
     */
    virtual bool matchesFrame(const CanFrame& cf) const
    {
        (void)cf;
        return false;
    }

    // 获取电机减速比
    virtual float get_GearRatio() const { return GEAR_RATIO; }

    // 目标值读取接口（非virtual，直接返回缓存值）
    float getTargetRPM() const { return target_rpm_; }
    float getTargetCurrent() const { return target_current_; }
    float getTargetAngle() const { return target_angle_; }
    float getTargetTotalAngle() const { return target_totalAngle_; }

    // CAN总线访问接口
    fdCANbus* bus() const { return bus_; }
    uint32_t getID() const { return motor_id_; }

protected:
    // 电机标识信息
    uint32_t motor_id_;
    bool isExtended_;
    fdCANbus* bus_;

    // 目标值 / 状态缓存
    float target_rpm_ = 0.0f;         // 目标转速（单位：RPM）
    float target_current_ = 0.0f;     // 目标电流（单位：mA）
    float target_angle_ = 0.0f;       // 目标单圈角度（单位：度）
    float target_totalAngle_ = 0.0f;  // 目标多圈总角度（单位：度）

    float GEAR_RATIO = 1.0f;  // 减速比，默认1
    float rpm_ = 0.0f;        // 当前实际转速（单位：RPM）
    float current_ = 0.0f;    // 当前实际电流（单位：mA）
    float angle_ = 0.0f;      // 当前实际单圈角度（单位：度）
    float totalAngle_ = 0.0f; // 当前实际多圈总角度（单位：度）
    float temperature_ = 0.0f; // 当前温度（单位：摄氏度）
};

#endif // MOTOR_BASE_H
