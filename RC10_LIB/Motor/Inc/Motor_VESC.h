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
#include <cstring>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Motor_Base.h"
#include "BSP_CanFrame.h"
#include "APP_tool.h"
#include "APP_PID.h"
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
    SET_NULL,               // 无
    SET_eRPM,               // 电气转速闭环模式，命令直接交给 VESC 内部闭环
    SET_CURRENT,            // 电流闭环模式，直接下发电流
    SET_PID_SPEED_CURRENT,  // 本地 PID 速度环，库内算电流后再下发
    SET_DUTY,               // 占空比模式，不推荐使用
    SET_POS,                // 位置模式，不推荐使用
    SET_BRAKE,              // 刹车
}VESC_MODE;

typedef enum {
    VESC_RPM_CONTROL_NATIVE_ERPM = 0, // setTargetRPM 走 VESC 原生 eRPM 闭环
    VESC_RPM_CONTROL_PID_CURRENT = 1  // setTargetRPM 走本地 PID 速度环 + CURRENT 下发
} VESC_RPM_CONTROL_MODE;

class VESC_Motor : public Motor_Base {
public:
    /**
     * @param id 电机CAN ID
     * @param bus 指向fdCANbus实例的指针
     * @param poles 电机极对数，默认21(u8)
     */
    VESC_Motor(uint32_t id, fdCANbus* bus, float poles);
    ~VESC_Motor(){};

    /**
     * @brief 周期更新函数
     * @note  仅当处于本地 PID 速度环模式时，才会在这里计算目标电流
     */
    void update() override;

    /**
     * @brief 初始化本地速度环 PID 参数
     * @param speed_params 速度环 PID 参数
     * @param speed_tdRatio 增量式 PID 的 td_ratio
     */
    void pid_init(const PID_Param_Config& speed_params, float speed_tdRatio);

    /**
     * @brief 设置速度环输出附加电流偏置
     * @note  该偏置只在 SET_PID_SPEED_CURRENT 模式下参与最终电流输出，
     *        用于在 chassis 层把虚拟惯量/阻尼/库仑摩擦等效到电流指令端。
     */
    void setSpeedPidCurrentBias(float current_bias_mA) { speed_pid_current_bias_mA_ = current_bias_mA; }

    /**
     * @brief 读取当前配置的速度环电流偏置
     */
    float getSpeedPidCurrentBias() const { return speed_pid_current_bias_mA_; }

    /**
     * @brief 读取速度 PID 的原始输出电流（未叠加 bias）
     */
    float getSpeedPidRawOutputCurrent() const { return speed_pid_raw_output_current_mA_; }

    /**
     * @brief 读取速度 PID 的总输出电流（raw + bias）
     */
    float getSpeedPidTotalOutputCurrent() const { return speed_pid_total_output_current_mA_; }

    /**
     * @brief 设置 RPM 控制策略
     * @note  默认仍为 VESC 原生 eRPM 闭环；仅显式切换后才使用本地 PID 速度环
     */
    void setRpmControlMode(VESC_RPM_CONTROL_MODE mode) { rpm_control_mode_ = mode; }
    VESC_RPM_CONTROL_MODE getRpmControlMode() const { return rpm_control_mode_; }

    void setTargetCurrent(float current_set) override;

    /**
     * @brief 设置目标转速，单位RPM (注意不是eRPM，是RPM)
     * @note  行为由 rpm_control_mode_ 决定：
     *        1. 原生模式：转成 eRPM 发给 VESC
     *        2. PID 模式：只记录 RPM 目标，后续在 update() 中算出目标电流
     */
    void setTargetRPM(float rpm_set) override;
    void setTargetAngle(float angle_set) override{};
    void setTargetTotalAngle(float totalAngle_set) override;
    void setBrake(float brake_current) override;
    void setDuty(float duty);

    PID_Param_Config get_speed_pid_params() const { return speed_pid_.get_params(); }
    float get_speed_pid_td_ratio() const { return speed_pid_.get_td_ratio(); }

    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;
    void updateFeedback(const CanFrame& cf) override;

    bool matchesFrame(const CanFrame& cf) const override
    {
        if (!cf.isextended || (cf.ID & 0xFFU) != motor_id_)
            return false;

        uint8_t packet_type = static_cast<uint8_t>((cf.ID >> 8) & 0xFFU);
        return packet_type == CAN_PACKET_STATUS_1; // 当前只解析 STATUS_1
    }

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
     * @note  新增的 SET_PID_SPEED_CURRENT 模式保留 target_rpm_ 和 bias，
     *        因为 update() 仍需要继续参与速度环计算与虚拟负载叠加。
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
                target_brake_current_ = 0.0f;
                target_eRPM_ = 0;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            case SET_eRPM:
                target_duty_ = 0.0f;
                target_brake_current_ = 0.0f;
                target_current_ = 0.0f;
                target_totalAngle_ = 0.0f;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            case SET_CURRENT:
                target_duty_ = 0.0f;
                target_brake_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_totalAngle_ = 0.0f;
                target_eRPM_ = 0;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            case SET_PID_SPEED_CURRENT:
                target_duty_ = 0.0f;
                target_brake_current_ = 0.0f;
                target_totalAngle_ = 0.0f;
                target_eRPM_ = 0;
                break;
            case SET_DUTY:
                target_brake_current_ = 0.0f;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_totalAngle_ = 0.0f;
                target_eRPM_ = 0;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            case SET_POS:
                target_duty_ = 0.0f;
                target_brake_current_ = 0.0f;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_eRPM_ = 0;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            case SET_BRAKE:
                target_duty_ = 0.0f;
                target_current_ = 0.0f;
                target_rpm_ = 0.0f;
                target_totalAngle_ = 0.0f;
                target_eRPM_ = 0;
                speed_pid_raw_output_current_mA_ = 0.0f;
                speed_pid_total_output_current_mA_ = 0.0f;
                break;
            default:
                break;
        }
    }
private:
    int32_t target_eRPM_ = 0; // 电气转速
    float GEAR_RATIO = 1.0f; // VESC一般无减速器
    VESC_MODE mode_ = SET_NULL;
    VESC_RPM_CONTROL_MODE rpm_control_mode_ = VESC_RPM_CONTROL_NATIVE_ERPM; // RPM 控制策略
    float poles_; // 极对数，默认21
    float target_duty_ = 0.0f; // 占空比 -1.0~1.0
    float duty_ = 0.0f; // 当前占空比
    int32_t eRPM_ = 0;
    float target_brake_current_ = 0.0f; // brake current in mA
    float speed_pid_current_bias_mA_ = 0.0f; // 速度环附加电流偏置，通常由上层虚拟负载模型写入
    float speed_pid_raw_output_current_mA_ = 0.0f; // 本地速度 PID 原始输出，不含 bias
    float speed_pid_total_output_current_mA_ = 0.0f; // 实际下发前的总电流输出，等于 raw + bias
    uint8_t id_check_ = 0; // 回传id，用于给用户分辨 motor_id_ 和电调 id 是否一致
    PID_Incremental speed_pid_; // 本地 PID 速度环，仅在显式启用时使用
};

#endif // __cplusplus

#endif// __MOTOR_VESC_H
