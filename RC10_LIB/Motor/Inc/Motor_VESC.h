/**
 * @file Motor_VESC.h
 * @author XieFField
 * @brief VESC电机类
 * @version 1.0
 */

#ifndef __MOTOR_VESC_H
#define __MOTOR_VESC_H

#pragma once

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#ifdef __cplusplus
}
#endif



#ifdef __cplusplus

#include "Motor_Base.h"
#include "BSP_CanFrame.h"
#include "APP_tool.h"
#include "BSP_fdCAN_Driver.h"
#include "BSP_RTOS.h"

// VESC CAN命令类型
typedef enum {
    CAN_CMD_SET_DUTY = 0,
    CAN_CMD_SET_CURRENT = 1,
    CAN_CMD_SET_CURRENT_BRAKE = 2,
    CAN_CMD_SET_ERPM = 3,
    CAN_CMD_SET_POS = 4,
    CAN_CMD_SET_POS_SPD_LIM = 5,
    CAN_CMD_SET_CURRENT_REL = 6,
    CAN_CMD_SET_CURRENT_BRAKE_REL = 7,
    CAN_CMD_SET_CURRENT_HANDBRAKE = 8,
    CAN_CMD_SET_CURRENT_HANDBRAKE_REL = 9
} VESC_CAN_CMD;

// VESC CAN反馈报文类型
typedef enum {
    CAN_PACKET_STATUS_1 = 9,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
    CAN_PACKET_STATUS_6 = 28
} VESC_CAN_PACKET_ID;

typedef enum{
    SET_NULL,   //无
    SET_eRPM,   //电气转速闭环模式
    SET_CURRENT,//电流闭环模式
    SET_DUTY,   //占空比模式，不推荐使用
    SET_POS,    //位置模式，不推荐使用
    SET_BRAKE,  //刹车
}VESC_MODE;

class VESC_Motor : public Motor_Base {
public:
    /**
     * @param id 电机CAN ID
     * @param bus 指向fdCANbus实例的指针
     * @param poles 电机极对数，默认21(u8)
     */
    VESC_Motor(uint32_t id, fdCANbus* bus, float poles);
    ~VESC_Motor(){};

    void update() override
    {
        //do nothing 
    }

    void setTargetCurrent(float current_set) override;

    /**
     * @brief 设置目标转速，单位RPM (注意不是eRPM，是RPM)
     */
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override{};
    void setTargetTotalAngle(float totalAngle_set) override;

    void setBrake(float brake_current);

    void setDuty(float duty);


    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    void updateFeedback(const CanFrame& cf) override;

    bool matchesFrame(const CanFrame& cf) const override
    {
        if (!cf.isextended || (cf.ID & 0xFF) != motor_id_)
            return false;

        uint8_t packet_type = (cf.ID >> 8) & 0xFF;
        return packet_type == CAN_PACKET_STATUS_1; // 当前只解析STATUS_1
    }

    float get_GearRatio() const override { return GEAR_RATIO; }

    void reset_GearRatio(float reset_value){GEAR_RATIO = reset_value;}

    int32_t RPM_to_eRPM(float rpm) const
    {
        return static_cast<int32_t>(rpm * poles_);
    }

    float eRPM_to_RPM(int32_t eRPM) const
    {
        return static_cast<float>(eRPM) / poles_;
    }

    /**
     * @brief 根据当前控制模式，重置其他控制参数，避免冲突
     */
    void reset_otherParam()
    {
        switch(mode_)
        {
            case SET_NULL:
                target_duty_ = 0.0f;
                target_totalAngle_ = 0.0f;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_brake_current_ = 0;
                break;
            case SET_eRPM:
                target_duty_ = 0.0f;
                target_brake_current_ = 0;
                target_current_ = 0.0f;
                target_totalAngle_ = 0.0f;

                break;
            case SET_CURRENT:
                target_duty_ = 0.0f;
                target_brake_current_ = 0;
                target_rpm_ = 0.0f;
                target_totalAngle_ = 0.0f;
                break;
            case SET_POS:
                target_duty_ = 0.0f;
                target_brake_current_ = 0;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                break;
            case SET_BRAKE:
                target_duty_ = 0.0f;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_totalAngle_ = 0.0f;
                break;
            default:
                break;
        }
    }
private:
    int32_t target_eRPM_ = 0; //电气转速
    float GEAR_RATIO = 1.0f; // VESC一般无减速器
    VESC_MODE mode_ = SET_NULL;
    float poles_; // 极对数，默认21
    float target_duty_ = 0.0f; //占空比  -1.0~1.0
    float duty_ = 0.0f; //当前占空比
    int32_t eRPM_ = 0; 
    int16_t target_brake_current_ = 0; //刹车电流
    uint8_t id_check_; //回传id，用于给用户分辨motor_id_和电调id是否一致
};


#endif // __cplusplus


#endif// __MOTOR_VESC_H