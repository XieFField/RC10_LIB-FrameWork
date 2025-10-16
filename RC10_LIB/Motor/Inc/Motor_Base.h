/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 电机基类定义
 * @version 1.0
 * @date 2025-09-16
 */
#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#pragma once
#ifdef __cplusplus

#endif // __cplusplus
#include "BSP_CanFrame.h"
#include <cstdint>
#include <cstddef>
class fdCANbus; // 前置声明

//电机基类包含通用接口
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // 目标设定
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};

    // 周期性更新函数，用于执行控制逻辑，在调度任务被唤醒，并周期性执行。
    virtual void update(){};

    // 反馈读取
    virtual float getRPM() const{};
    virtual float getCurrent() const{};
    virtual float getAngle() const{};
    virtual float getTotalAngle() const{};

    // 抽象：打包命令 -> 返回需要发送的帧数
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    // 抽象：解析反馈帧
    virtual void updateFeedback(const CanFrame& cf) = 0;

    // 帧匹配（默认简单 ID/Ext 匹配）
    virtual bool matchesFrame(const CanFrame& cf) const{};

    virtual float get_GearRatio() const { return GEAR_RATIO; }

    float getTargetRPM() const { return target_rpm_; }
    float getTargetCurrent() const { return target_current_; }
    float getTargetAngle() const { return target_angle_; }
    float getTargetTotalAngle() const { return target_totalAngle_; }
    

    fdCANbus* bus() const { return bus_; }
    uint32_t getID() const { return motor_id_; }
protected:
    uint32_t motor_id_;
    bool isExtended_;
    fdCANbus* bus_;

    // 目标/状态量
    float target_rpm_ = 0.0f; //转速
    float target_current_= 0.0f; //电流
    float target_angle_ = 0.0f; //角度
    float target_totalAngle_ = 0.0f; //总角度
    
    float GEAR_RATIO = 1.0f; // 减速比，默认为1
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; //温度

};




#endif // MOTOR_BASE_H
