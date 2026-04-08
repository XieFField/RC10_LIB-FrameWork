/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 电机基类声明
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

//电机基类
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // =
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};

    // 更新接口，子类必须实现以更新电机状态
    virtual void update(){};
    
    // 反馈接口，子类可选择性实现以提供特定的反馈数据访问
    virtual float getRPM() const { return 0.0f; }   
    virtual float getCurrent() const { return 0.0f; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief 打包发送的CAN帧接口，子类必须实现以提供特定的控制命令帧
     * @param outFrames 用于存放打包后CAN帧的数组，调用者提供内存，子类负责填充内容。数组大小由 maxFrames 参数指定。
     * @param maxFrames 用于指定 outFrames 数组的大小
     * @return 实际填充的CAN帧数量
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief CAN报文解析接口，子类必须实现以处理特定的反馈帧
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief CAN帧匹配函数，默认实现为严格ID和扩展帧标志匹配，子类可 override 以实现更复杂的匹配逻辑（如模糊ID匹配）
     * @param cf 要匹配的CAN帧
     * @return 如果帧匹配当前电机实例，则返回true；否则返回false。默认实现为严格ID和扩展帧标志匹配。
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

    //目标值（由控制器设置，电机类或子类可选择性使用）
    float target_rpm_ = 0.0f; //目标转速 rpm
    float target_current_= 0.0f; //目标电流 ma
    float target_angle_ = 0.0f; //目标角度 deg
    float target_totalAngle_ = 0.0f; //目标总角度 deg
    
    float GEAR_RATIO = 1.0f; // 减速比
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; //电机温度

};




#endif // MOTOR_BASE_H
