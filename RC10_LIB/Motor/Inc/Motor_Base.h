/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰柦顭掓�?閿燂�?
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
class fdCANbus; // 鍓嶉敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ�?

//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽€氶敓�?��帴鍖℃�?
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // 鐩敓鏂ゆ�?閿熷�?���?
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};
    virtual void setBrake(float brake_current)
    {
        (void)brake_current;
    };

    // 閿熸枻鎷烽敓鏂ゆ嫹閿熺殕闈╂嫹閿熼摪鐚存嫹閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ嫹鎵ч敓鍙尅鎷烽敓鏂ゆ�?閿熺�?��锋�?閿熸枻鎷烽敓鑺傜�?��烽敓鏂ゆ嫹閿熸枻鎷烽敓浠婅浼欐嫹閿熺獤锝忔嫹閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ嫹鎵ч敓鍙�?鎷�
    virtual void update(){};
    
    // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰�?
    virtual float getRPM() const { return 0.0f; }   
    virtual float getCurrent() const { return 0.0f; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�?洰閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓绱篈N�?��
     * @param outFrames 閿熸枻鎷烽敓鑺傝�?��风朝閿熸枻鎷烽敓鏂ゆ嫹CAN�?�敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ�?
     * @param maxFrames 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ�?
     * @return 瀹為敓缁炶揪鎷烽敓鏂ゆ�?閿熺�?N�?�敓鏂ゆ嫹閿熸枻鎷�
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ�?閿熸枻鎷烽敓鏂ゆ嫹閿熺殕纰�?嫹閿熸枻鎷峰閿熸枻鎷烽敓绱篈N�?��
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ�?CAN�?�敓瑙掑嚖鎷烽敓鏂ゆ嫹閿熻妭姝ょ�?��烽敓锟�
     * @return 閿熸枻鎷烽敓鐙°儻鎷烽敓鏂ゆ�?铓嶇シ閿熺但rue閿熸枻鎷烽敓鏂ゆ嫹閿熸触杩斾�?��穎alse
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
public:
    uint32_t motor_id_;
    bool isExtended_;
    fdCANbus* bus_;

    // 鐩敓鏂ゆ�?/鐘舵�?��敓鏂ゆ�?
    float target_rpm_ = 0.0f; //杞敓鏂ゆ�?
    float target_current_= 0.0f; //閿熸枻鎷烽敓鏂ゆ�?
    float target_angle_ = 0.0f; //閿熻璁规�?
    float target_totalAngle_ = 0.0f; //閿熸澃瑙掕鎷�
    
    float GEAR_RATIO = 1.0f; // 閿熸枻鎷烽敓鍔�?��忔�?榛橀敓鏂ゆ�?涓�1
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; //閿熼�?��规�?

};




#endif // MOTOR_BASE_H
