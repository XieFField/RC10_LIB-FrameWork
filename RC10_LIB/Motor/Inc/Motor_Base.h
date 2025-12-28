/**
 * @file Motor_Base.h
 * @author XieFField
 * @brief 锟斤拷锟斤拷锟斤拷喽拷锟�
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
class fdCANbus; // 前锟斤拷锟斤拷锟斤拷

//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷通锟矫接匡拷
class Motor_Base {
public:
    Motor_Base(uint32_t id, bool isExt, fdCANbus* bus)
    {
        motor_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    };
    virtual ~Motor_Base(){};

    // 目锟斤拷锟借定
    virtual void setTargetRPM(float rpm_set){};
    virtual void setTargetCurrent(float current_set){};
    virtual void setTargetAngle(float angle_set){};
    virtual void setTargetTotalAngle(float totalAngle_set){};

    // 锟斤拷锟斤拷锟皆革拷锟铰猴拷锟斤拷锟斤拷锟斤拷锟斤拷执锟叫匡拷锟斤拷锟竭硷拷锟斤拷锟节碉拷锟斤拷锟斤拷锟今被伙拷锟窖ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷执锟叫★拷
    virtual void update(){};
    
    // 锟斤拷锟斤拷锟斤拷取
    virtual float getRPM() const { return 0.0f; }   
    virtual float getCurrent() const { return 0.0f; }
    virtual float getAngle() const { return 0.0f; }
    virtual float getTotalAngle() const { return 0.0f; }

    
    /**
     * @brief 锟斤拷锟斤拷锟斤拷目锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟紺AN帧
     * @param outFrames 锟斤拷锟节达拷糯锟斤拷锟斤拷CAN帧锟斤拷锟斤拷锟斤拷
     * @param maxFrames 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
     * @return 实锟绞达拷锟斤拷锟紺AN帧锟斤拷锟斤拷
     */
    virtual std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) = 0;

    
    /**
     * @brief 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆碉拷锟斤拷姆锟斤拷锟紺AN帧
     */
    virtual void updateFeedback(const CanFrame& cf) = 0;

    /**
     * @brief 锟斤拷锟斤拷锟斤拷锟斤拷CAN帧锟角凤拷锟斤拷锟节此碉拷锟�
     * @return 锟斤拷锟狡ワ拷锟斤拷蚍祷锟絫rue锟斤拷锟斤拷锟津返伙拷false
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

    // 目锟斤拷/状态锟斤拷
    float target_rpm_ = 0.0f; //转锟斤拷
    float target_current_= 0.0f; //锟斤拷锟斤拷
    float target_angle_ = 0.0f; //锟角讹拷
    float target_totalAngle_ = 0.0f; //锟杰角讹拷
    
    float GEAR_RATIO = 1.0f; // 锟斤拷锟劫比ｏ拷默锟斤拷为1
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float totalAngle_ = 0.0f;
    float temperature_ = 0.0f; //锟铰讹拷

};




#endif // MOTOR_BASE_H
