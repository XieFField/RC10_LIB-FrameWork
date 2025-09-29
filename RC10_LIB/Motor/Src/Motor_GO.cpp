/**
 * @file		Motor_Go.c
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-09-28 (创建日期)
 * @date        2025-09- (最后修改日期)
 * @platform	
 * @version     0.1.0
 * @details     
 * @note		
 * @warning		
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 0.1.0
 *      - 修订日期: 2025-09-
 *      - 主要变更:
 *			- 
 *		- 不足之处:
 *			- 
 *      - 作者: ZhangJiaJia
 */


#include "Motor_Go.h"


/**
 * @brief 检查CAN帧是否符合电机的报文格式
 * @param cf CAN帧
 * @return true 匹配成功
 * @return false 匹配失败
 */
bool GO_Motor::matchesFrame(const CanFrame& cf) const
{
    if(!cf.isextended) 
        return false; // GO电机使用拓展帧
    if(!(cf.ID >> 27 < 14 && cf.ID >> 27 >= 0))
        return false; // GO电机ID范围为0-14
    return true;
}

/**
 * @brief 打包命令
 * @param outFrames 输出CAN帧数组
 * @param maxFrames 输出CAN帧数组最大长度
 * @return std::size_t 实际打包的CAN帧数量
 */
std::size_t GO_Motor::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
    if(maxFrames < 1)
        return 0; // 无法打包

    outFrames[0].isextended = true;
    outFrames[0].DLC = 8;
    memset(outFrames[0].data, 0, 8);

    CAN_extended_id_t extended_id = {
        .module_id = 3,
        .upload_or_download = 0,
        .control_or_response = 0,
        .low_3 = (uint8_t)motor_control_mode_,
        .low_2 = motor_id_ << 4 | (uint8_t)motor_mode_ << 1 | 0,
        .low_1 = 0,
        .reserved = 0,
    };

    
    CAN_extended_id_t& id = extended_id;
    outFrames[0].ID =   (id.module_id << 27) | 
                        (id.upload_or_download << 26) |
                        (id.control_or_response << 24) |
                        (id.low_3 << 16) |
                        (id.low_2 << 8) |
                        id.low_1;

    CAN_data_t data = {
         .byte_0 = 0, // 后面待修改
         .byte_1 = 0, // 后面待修改
         .byte_2 = 0, // 后面待修改
         .byte_3 = 0, // 后面待修改
         .byte_4 = (uint8_t)(((int16_t)(target_rpm_)) >> 8),
         .byte_5 = (uint8_t)((int16_t)(target_rpm_)),
         .byte_6 = (uint8_t)(((int16_t)(target_torque_)) >> 8),
         .byte_7 = (uint8_t)((int16_t)(target_torque_)),
    };         

    switch (motor_control_mode_)
    {
        case Motor_Control_Mode::MODE_11:
        {
            int16_t kpos_int = (int16_t)(kpos_);
            int16_t kspd_int = (int16_t)(kspd_);
            data.byte_0 = (uint8_t)(kpos_int >> 8);
            data.byte_1 = (uint8_t)(kpos_int);
            data.byte_2 = (uint8_t)(kspd_int >> 8);
            data.byte_3 = (uint8_t)(kspd_int);
            break;
        }
        default:
        {
            float theta_set = target_angle_ / 360 * 32768;
            int32_t theta_int = (int32_t)(theta_set);
            data.byte_0 = (uint8_t)(theta_int >> 24);
            data.byte_1 = (uint8_t)(theta_int >> 16);
            data.byte_2 = (uint8_t)(theta_int >> 8);
            data.byte_3 = (uint8_t)(theta_int);

            float omega_set = target_rpm_ / 60 * 256;
            int16_t omega_int = (int16_t)(omega_set);
            data.byte_4 = (uint8_t)(omega_int >> 8);
            data.byte_5 = (uint8_t)(omega_int);

            float torque_set = target_torque_ * 256;
            int16_t torque_int = (int16_t)(torque_set);
            data.byte_6 = (uint8_t)(torque_int >> 8);
            data.byte_7 = (uint8_t)(torque_int);

            break;
        }
    }

    outFrames[0].data[0] = data.byte_0;
    outFrames[0].data[1] = data.byte_1;
    outFrames[0].data[2] = data.byte_2;
    outFrames[0].data[3] = data.byte_3;
    outFrames[0].data[4] = data.byte_4;
    outFrames[0].data[5] = data.byte_5;
    outFrames[0].data[6] = data.byte_6;
    outFrames[0].data[7] = data.byte_7;

    return 1;
}

/**
 * @brief 设置目标输出轴转速，单位RPM
 * @param rpm_set 目标输出轴转速
 */
void GO_Motor::setTargetRPM(float rpm_set)
{
    target_rpm_ = rpm_set;
}

/**
 * @brief 设置目标输出轴角度，单位度
 * @param angle_set 目标输出轴角度
 */
void GO_Motor::setTargetAngle(float angle_set)
{
    target_angle_ = angle_set;
}

/**
 * @brief 解析电机返回的CAN报文
 * @param cf 电机返回的CAN报文
 */
void GO_Motor::updateFeedback(const CanFrame& cf)
{
    CAN_extended_id_t extended_id = {
        .module_id = cf.ID >> 27,
        .upload_or_download = (cf.ID >> 26) & 0x1,
        .control_or_response = (cf.ID >> 24) & 0x1,
        .low_3 = (cf.ID >> 16) & 0xFF,
        .low_2 = (cf.ID >> 8) & 0xFF,
        .low_1 = cf.ID & 0xFF,
        .reserved = 0,
    };

    CAN_data_t data = {
        .byte_0 = cf.data[0],
        .byte_1 = cf.data[1],
        .byte_2 = cf.data[2],
        .byte_3 = cf.data[3],
        .byte_4 = cf.data[4],
        .byte_5 = cf.data[5],
        .byte_6 = cf.data[6],
        .byte_7 = cf.data[7],
    };

    if ( extended_id.low_3 == (uint8_t)Motor_Control_Mode::MODE_2)
    {
        int16_t kpos_int = (data.byte_0 << 8) | data.byte_1;
        int16_t kspd_int = (data.byte_2 << 8) | data.byte_3;
        kpos_ = static_cast<float>(kpos_int) / 32768 * 360;
        kspd_ = static_cast<float>(kspd_int) / 32768 * 360;
    }
    else if (extended_id.low_1 == -128)
    {
        // 电机报错，待处理
    }
    else if (extended_id.low_1 >= -127 && extended_id.low_1 <= 127)
    {
        current_atm_ = (float)extended_id.low_3;
        current_motor_temperature_ = extended_id.low_1;

        int32_t angle_int = (data.byte_0 << 24) | (data.byte_1 << 16) | (data.byte_2 << 8) | data.byte_3;
        current_angle_ = (float)angle_int / 32768 * 360;

        int16_t omega_int = (data.byte_4 << 8) | data.byte_5;
        target_rpm_ = (float)omega_int / 256 * 60;

        int16_t torque_int = (data.byte_6 << 8) | data.byte_7;
        current_torque_ = (float)torque_int / 256;
    }
    else 
    {
        // 不应该发生的情况，待处理
    }
}

/**
 * @brief 读取电机当前模式
 */
void GO_Motor::readKposAndKspd()
{
    motor_control_mode_ = Motor_Control_Mode::MODE_12;
}






