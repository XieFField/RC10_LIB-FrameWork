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

class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus, 
        bool calcTotalAngle = true, bool calcAngle = true)
        : is_calcangle(calcAngle),
          is_calcTotalAngle(calcTotalAngle)
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
    virtual void setBrake(float brake_current)
    {
        (void)brake_current;
    };

    // 更新函数，负责根据最新的反馈数据计算控制输出
    virtual void update(){};
    
    // 获取输出轴状态
    virtual float getRPM() const { return rpm_; }   
    virtual float getCurrent() const { return current_; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief 打包要发送的CAN帧，子类必须实现以此提供特定的控制命令帧
     * @param outFrames 用于存储打包的CAN帧的数组，调用者提供内存，子类负责填充内容。数组大小由 maxFrames 参数指定。
     * @param maxFrames 用于指定 outFrames 数组的大小
     * @return 实际填充的CAN帧数量，如果超过 maxFrames 则只填充 maxFrames 个
     * @attention 该函数由 fdCANbus 的调度器任务周期性调用，以实现定时发送控制命令
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief CAN帧解析接口，子类必须实现以此处理特定的反馈数据
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief CAN帧匹配函数，默认实现为ID和帧类型匹配，子类可 override 以实现更复杂的匹配逻辑（如协议ID匹配）
     * @param cf 需要匹配的CAN帧
     * @return 如果帧匹配当前电机实例，则返回true，否则返回false。默认实现为简单的ID和帧类型匹配
     */
    virtual bool matchesFrame(const CanFrame& cf) const
    {
        (void)cf;
        return false;
    }

    float get_GearRatio() const { return GEAR_RATIO; }
    float get_inv_GearRatio() const { return inv_GEAR_RATIO_; }
    float getTargetRPM() const { return target_rpm_; }
    float getTargetCurrent() const { return target_current_; }
    float getTargetAngle() const { return target_angle_; }
    float getTargetTotalAngle() const { return target_totalAngle_; }
    

    fdCANbus* bus() const { return bus_; }
    uint32_t getID() const { return motor_id_; }


    void reset_controlFrequency(uint16_t newFreq) 
    { 
        if(newFreq > 0 && newFreq % 100 == 0 && newFreq <= 1000) // 控制频率必须是100的整数倍
            control_Frequency_ = newFreq; 
        else
            control_Frequency_ = 1000; // 恢复默认值
    }


    uint16_t get_controlFrequency() const { return control_Frequency_; }
    uint16_t get_controlCnt() const { return control_cnt; }
    void reset_controlCnt() { control_cnt = 0; }
    void increment_controlCnt() { control_cnt++; }



protected:
    bool is_calcangle = true; //仅仅在is_calcTotalAngle为true时，is_calcangle才生效
    bool is_calcTotalAngle = true;
    uint32_t motor_id_;
    bool isExtended_;
    fdCANbus* bus_;

    // 目标值
    float target_rpm_ = 0.0f; // 目标转速 rpm
    float target_current_= 0.0f; // 目标电流 ma
    float target_angle_ = 0.0f; // 目标角度 deg
    float target_totalAngle_ = 0.0f; // 目标总角度 deg

    float GEAR_RATIO = 1.0f; // 减速比
    float inv_GEAR_RATIO_ = 1.0f; // 反减速比，预计算以提高效率
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; // 电机温度

    uint16_t control_cnt = 0; // 控制周期计数器，用于实现不同频率的控制逻辑
private:
    uint16_t control_Frequency_ = 1000; // 默认控制频率 Hz，重设的控制频率必须是100的整数倍

};




#endif // MOTOR_BASE_H