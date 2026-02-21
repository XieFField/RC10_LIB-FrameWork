/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 电机基类，定义了电机的基本接口和属性
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


class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // 控制接口
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};

    // 更新电机状态
    virtual void update(){};
    
    // 获取反馈数据
    virtual float getRPM() const { return 0.0f; }   
    virtual float getCurrent() const { return 0.0f; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief 打包控制命令为CAN帧，默认不打包，由具体电机类型负责实现
     * @param outFrames 输出的CAN帧数组，调用者负责分配内存
     * @param maxFrames     输入的最大帧数，调用者提供的数组大小
     * @return 实际打包的帧数，0表示未打包
     * @attention 由具体电机类型负责实现，默认不打包
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief 更新反馈数据，默认不处理，由具体电机类型负责实现反馈帧解析
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief 判断CAN帧是否匹配当前电机，默认不匹配，由具体电机类型负责实现
     * @return true表示匹配，false表示不匹配
     */
    virtual bool matchesFrame(const CanFrame& cf) const
    {
        (void)cf;
        return false;
    }

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

    // 控制目标
    float target_rpm_ = 0.0f; 
    float target_current_= 0.0f; 
    float target_angle_ = 0.0f; 
    float target_totalAngle_ = 0.0f; 
    
    float GEAR_RATIO = 1.0f; // 减速比，默认为1.0f
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; // 温度，单位摄氏度

};




#endif // MOTOR_BASE_H
