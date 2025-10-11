/**
 * @file Motor_DJI.h
 * @author XieFField
 * @brief 大疆电机类
 * @version 1.0
 * 
 * 此文件包含大疆 M3508 M2006 GM6020封装
 */

/*
封装思路参考了洁宇哥的框架，不过现在设计成了其他电机和DJI电机可以混搭的做法
*/

#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#pragma once

#ifdef __cplusplus
extern "C" {
    #include <stdint.h>
    #include "BSP_CanFrame.h"
}
#endif

/*
    ____      ______     __  _______  __________  ____ 
   / __ \    / /  _/    /  |/  / __ \/_  __/ __ \/ __ \
  / / / /_  / // /     / /|_/ / / / / / / / / / / /_/ /
 / /_/ / /_/ // /     / /  / / /_/ / / / / /_/ / _, _/ 
/_____/\____/___/    /_/  /_/\____/ /_/  \____/_/ |_|  
*/

#ifdef __cplusplus

#include "Motor_Base.h"
#include "APP_tool.h"
#include "APP_PID.h"
#include <cstring>
#include "Module_Encoder.h"

typedef enum {
    M3508_Type,
    M2006_Type,
    GM6020_Type
} DJI_MotorType;

//
/**
 * @brief     负责打包4电机合帧
 * @attention 如果你想要在同一路CAN上搭载GM6020和M3508/M2006，请确保GM6020在下片上
 *            而M3508/M2006在上片上。因为M3508/M2006的ID范围是0x201到0x208, 
 *            而GM6020的ID范围是0x205到0x211
 */
class DJI_Motor : public Motor_Base {
public:
    /**
     * @brief     负责打包4电机合帧
     * @attention 如果你想要在同一路CAN上搭载GM6020和M3508/M2006，请确保GM6020在下片上
     *            而M3508/M2006在上片上。因为M3508/M2006的ID范围是0x201到0x208, 
     *            而GM6020的ID范围是0x205到0x211
     */
    DJI_Motor(DJI_MotorType type, uint32_t id, fdCANbus *bus);
    ~DJI_Motor(){};

    bool matchesFrame(const CanFrame& cf) const override
    {
        if(cf.isextended) 
            return false; // DJI电机只用标准帧

        if(type_ == GM6020_Type)//：0x204+驱动器ID
        {
            if(cf.ID < (0x204 + 1) || cf.ID > (0x204 + 7))
                return false; // 非法ID

            else if(cf.ID >= (0x204 + 1) && cf.ID <= (0x204 + 7))
                return (cf.ID == (0x204 + motor_id_));
            
            else
                return false; // 非法ID 
        }
        if(type_ == M3508_Type || type_ == M2006_Type)//：0x200+驱动器ID 1~8
        {
            if(cf.ID < (0x200 + 1) || cf.ID > (0x200 + 8))
                return false; // 非法ID

            else if(cf.ID >= (0x200 + 1) && cf.ID <= (0x200 + 8))
                return (cf.ID == (0x200 + motor_id_));

            else
                return false; // 非法ID
        }
        else
            return false; // 未知类型
    }

    /**
     * @brief 将虚拟电流值转换为实际电流值
     */
    float virtualCurrent_to_realCurrent(int16_t virtualCurrent);

    /**
     * @brief 将实际电流值转换为虚拟电流值
     */
    int16_t realCurrent_to_virtualCurrent(float realCurrent);


    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override
    {
        return 0; // 由 DJI_Group 负责打包
    }

    void updateFeedback(const CanFrame& cf) override;

    void setTargetCurrent(float current_set) override
    {
        target_current_ = current_set;
    }

    DJI_MotorType getType() const { return type_; }

    void reset_AnglePidTimePSC(int reset_value = 0)
    {
        anglePid_timePSC = reset_value;
    }

protected:
    int anglePid_timePSC = 10; //角度时间分频 默认为 10 即控制频率为100Hz
    int anglePid_timeCnt = 0; //角度时间计数

private:
     //1~8
    DJI_MotorType type_;
    int16_t encoder_raw_ = 0; //原始编码器值

    

//common
    virtual uint32_t recv_id() const{return 0x200;}

    Encoder encoder_; //编码器实例
};

uint32_t send_idLow();
uint32_t send_idHigh();
uint32_t send_idLow6020();
uint32_t send_idHigh6020();

//------- DJI_Group ------- 打包命令 -----------------------------
// 
/**
 * @brief     负责打包4电机合帧
 * @attention 如果你想要在同一路CAN上搭载GM6020和M3508/M2006，请确保GM6020在下片上
 *            而M3508/M2006在上片上。因为M3508/M2006的ID范围是0x201到0x208, 
 *            而GM6020的ID范围是0x205到0x211
 */
class DJI_Group : public Motor_Base {
public:
    static constexpr std::size_t MAX_GROUP_SIZE = 4;
    
    DJI_Group(uint32_t baseTxId, fdCANbus* bus);
    ~DJI_Group() = default;

    /**
     * @brief 注册电机
     */
    bool addMotor(DJI_Motor* motor);

    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    void updateFeedback(const CanFrame& cf) override
    {
        (void)cf;
    }

    void update() override
    {
        //do nothing
    }

private:
    uint32_t baseTxID_;
    
    DJI_Motor* motors_p[MAX_GROUP_SIZE] = {nullptr, nullptr, nullptr, nullptr};
    uint8_t motor_count_ = 0;

    bool containsGM6020 = false; //是否包含GM6020, M3508/M2006不和GM6020混用

    int calcSlot(uint32_t motorID, DJI_MotorType type) const; // 计算槽位
};

#define M3508_DECRATION 19.2032f //减速比
#define M3508_KT 0.01562f // 3508的内圈转矩常数 单位：N.M/A

// 控制模式枚举
typedef enum {
    CURRENT_CONTROL, // 开环电流控制
    SPEED_CONTROL,   // 速度闭环控制
    ANGLE_CONTROL,    // 角度闭环控制
    TOTAL_ANGLE_CONTROL // 总角度闭环控制
} ControlMode;

class M3508 : public DJI_Motor {
public:
    M3508(uint32_t motor_id, fdCANbus* bus);
    ~M3508() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa);

    // 控制接口
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override; //周期性更新

    // 获取输出轴状态
    float getRPM() const override;
    float getAngle() const override;
    float getTotalAngle() const override;

    void reset_GearRatio(float reset_value){GEAR_RATIO = reset_value;}

    float get_GearRatio() const override { return GEAR_RATIO; }
private:
    
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 19.2032f; // 减速比

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};

#define M2006_DECRATION 36.0f //减速比
class M2006 : public DJI_Motor {
public:
    M2006(uint32_t motor_id, fdCANbus* bus);
    ~M2006() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_I_Separa, const PID_Param_Config& angle_params, float angle_I_Separa);

    // 控制接口
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override;

    // 获取输出轴状态
    float getRPM() const override;
    float getAngle() const override;
    float getTotalAngle() const override;
    float get_GearRatio() const override { return GEAR_RATIO; }
    void reset_GearRatio(float reset_value){GEAR_RATIO = reset_value;}
private:
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 36.0f;

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};

class GM6020 : public DJI_Motor {
public:
    GM6020(uint32_t motor_id, fdCANbus* bus);
    ~GM6020() {};

    void pid_init(const PID_Param_Config& speed_params, float speed_I_Separa, const PID_Param_Config& angle_params, float angle_I_Separa);

    // 控制接口
    void setTargetCurrent(float current_set) override;
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override;
    void setTargetTotalAngle(float totalAngle_set) override;

    void update() override;

    // 获取输出轴状态 (GM6020无减速器)
    float getRPM() const override { return rpm_; }
    float getAngle() const override { return angle_; }
    float getTotalAngle() const override { return totalAngle_; }

private:
    ControlMode mode_ = CURRENT_CONTROL;
    float GEAR_RATIO = 1.0f;

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};


#endif



#endif // __DJI_MOTOR_H