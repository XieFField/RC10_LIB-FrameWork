/**
 * @file		Motor_Go.c
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-09-28 (创建日期)
 * @date        2025-09-29 (最后修改日期)
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
 *      - 修订日期: 2025-09-29
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
bool GO_Motor::matchesFrame(const CanFrame& cf) const override
{
    if(!cf.isextended) 
        return false; // GO电机使用拓展帧
    if(cf.id > 0 || cf.id < 3)
        return false; // GO电机ID范围为0-3
    return true;
}

/**
 * @brief 打包命令
 * @param outFrames 输出CAN帧数组
 * @param maxFrames 输出CAN帧数组最大长度
 * @return std::size_t 实际打包的CAN帧数量
 */
std::size_t GO_Motor::packCommand(CanFrame outFrames[], std::size_t maxFrames) override
{
    if(maxFrames < 1)
        return 0; // 无法打包

    outFrames[0].isextended = true;
    outFrames[0].dlc = 8;
    memset(outFrames[0].data, 0, 8);


    CAN_extended_id_t extended_id = {
        .module_id = motor_id_,
        .upload_or_download = 0,
        .control_or_response = 0,
        .low_3 = 10,
        .low_2 = 0,
        .low_1 = 0,
        .reserved = 0,
    };



    switch (mode_)
    {
        case DEFAULT:
        {
            Motor_Mode motor_mode_ = Motor_Mode::DEFAULT;



            break;
        }
        case FOC:
        {
            motor_mode_ = Motor_Mode::FOC;
            break;
        }
        case CALIBRATION:
        {
            motor_mode_ = Motor_Mode::CALIBRATION;
            break;
        }
        default:
        {
            motor_mode_ = Motor_Mode::DEFAULT;
            break;
        }
    }
    return 1;
}

/**
 * @brief 设置目标输出轴转速，单位RPM
 * @param rpm_set 目标输出轴转速
 */
void GO_Motor::setTargetRPM(float rpm_set) override
{
    target_rpm_ = rpm_set;
}

/**
 * @brief 设置目标输出轴角度，单位度
 * @param angle_set 目标输出轴角度
 */
void GO_Motor::setTargetAngle(float angle_set) override
{
    target_angle_ = angle_set;
}
