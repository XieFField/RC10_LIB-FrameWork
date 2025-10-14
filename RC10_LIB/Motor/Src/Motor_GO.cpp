/**
 * @file		Motor_Go.c
 * @brief       宇树GO-M8010-6电机驱动，支持力矩、速度和角度三种独立控制模式
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-09-28 (创建日期)
 * @date        2025-10-14 (最后修改日期)
 * @platform	学院STM32H723ZGT6核心板
 * @version     0.1.0
 * @details     暂无
 * @todo        1. 把KposKspd的设置和读取从程序里删掉
 *              2. 待解决GO电机编码器初始化指令要确认存在有效接收对象的问题
 *              3. 代码表述待优化
 * @note        暂无
 * @warning		暂无
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 0.1.0
 *      - 修订日期: 2025-10-14
 *      - 主要变更:
 *			- 完成基本功能，可以并入主代码
 *          - 支持力矩、速度和角度三种独立控制模式
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
    if(!(cf.ID >> 27 < 15 && cf.ID >> 27 >= 0))
        return false; // GO电机ID范围为0-15
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






    int16_t inputTorque = (int16_t)(target_torque_ * 256.0f); // 没有四舍五入，直接截断


    if(!isInit_)
    {
        motor_control_mode_ = Motor_Control_Mode::MODE_10; // MODE_10 和 MODE_13 均可以

        this->setKposAndKspd(0, 0);
    }
    else if(isSetKposKspd_)
    {
        motor_control_mode_ = Motor_Control_Mode::MODE_11; // 设置kpos和kspd
        isReadKposKspd_ = true;
    }
    else if(isReadKposKspd_)
    {
        motor_control_mode_ = Motor_Control_Mode::MODE_12; // 读取kpos和kspd
    }
    else if(isReturnData_)
    {
        motor_control_mode_ = Motor_Control_Mode::MODE_10; // 每控制一次电机就返回一次数据
    }
    else if(!isReturnData_)
    {
        motor_control_mode_ = Motor_Control_Mode::MODE_13; 
        // 每控制一次电机CAN不返回电机数据
        // 除非电机报错，报错时会返回电机数据
        // 用户需要电机数据时需要发送问答命令，电机将返回最后一次通讯时保留的数据
    }
    else
    {
        // 意料之外的情况，待处理
    }

    if(isSetKposKspd_ || isReadKposKspd_)
    {
        mode_ = Mode::SET_DEFAULT;
    }

    switch(mode_)
    {
        case Mode::SET_DEFAULT:
        {
            motor_mode_ = Motor_Mode::DEFAULT;
            break;
        }
        case Mode::SET_TORQUE:
        {
            motor_mode_ = Motor_Mode::FOC;
            break;
        }
        case Mode::SET_RPM:
        {
            motor_mode_ = Motor_Mode::FOC;
            break;
        }
        case Mode::SET_POS:
        {
            motor_mode_ = Motor_Mode::FOC;
            break;
        }
    }


    if(!isInit_)
    {
        motor_mode_ = Motor_Mode::CALIBRATION;
        isInit_ = true;
    }






    CAN_extended_id_t extended_id = {
        .module_id = 3,
        .upload_or_download = 0,
        .control_or_response = 0,
        .low_3 = (uint8_t)motor_control_mode_,
        .low_2 =   (uint8_t)motor_mode_ << 4 | motor_id_ << 0,
        .low_1 = 0,
        .reserved = 0,
    };

    CAN_data_t data = {
         .byte_0 = 0,
         .byte_1 = 0,
         .byte_2 = 0,
         .byte_3 = 0,
         .byte_4 = 0,
         .byte_5 = 0,
         .byte_6 = (uint8_t)(inputTorque),
         .byte_7 = (uint8_t)(inputTorque >> 8),
    };      


    if(isSetKposKspd_)
    {
        uint16_t input_kpos = (uint16_t)(target_kpos_ * 1280.0f); // 没有四舍五入，直接截断
        uint16_t input_kspd = (uint16_t)(target_kspd_ * 1280.0f); // 没有四舍五入，直接截断

        data.byte_0 = (uint8_t)(input_kpos);
        data.byte_1 = (uint8_t)(input_kpos >> 8);
        data.byte_2 = (uint8_t)(input_kspd);
        data.byte_3 = (uint8_t)(input_kspd >> 8);
		
		isSetKposKspd_ = false;
    }



    CAN_extended_id_t& id = extended_id;
    outFrames[0].ID =   (id.module_id << 27) | 
                        (id.upload_or_download << 26) |
                        (id.control_or_response << 24) |
                        (id.low_3 << 16) |
                        (id.low_2 << 8) |
                        id.low_1;

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
 * @brief 设置目标输出轴转矩，单位N.m
 * @param torque_set 目标输出轴转矩
 */
void GO_Motor::setTargetTorque(float torque_set)
{
    mode_ = Mode::SET_TORQUE;
    resetParam();
    target_torque_ = torque_set; // 没有输入检查
}

/**
 * @brief 设置目标输出轴转速，单位RPM
 * @param rpm_set 目标输出轴转速
 */
void GO_Motor::setTargetRPM(float rpm_set)
{
    mode_ = Mode::SET_RPM;
    resetParam();
    target_rpm_ = rpm_set;
}

/**
 * @brief 设置目标输出轴角度，单位度
 * @param angle_set 目标输出轴角度
 */
void GO_Motor::setTargetAngle(float angle_set)
{
    mode_ = Mode::SET_POS;
    resetParam();
    target_angle_ = angle_set;

    float angle_offset = target_angle_ - current_angle_;
    if(angle_offset > 180.0f)
    {
        angle_offset -= 360.0f;
    }
    else if(angle_offset < -180.0f)
    {
        angle_offset += 360.0f;
    }

    target_totalAngle_ = current_totalAngle_ + angle_offset;
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
        int16_t kpos_int = (data.byte_1 << 8) | data.byte_0;
        int16_t kspd_int = (data.byte_3 << 8) | data.byte_2;
        current_kpos_ = static_cast<float>(kpos_int) / 1280.0f;
        current_kspd_ = static_cast<float>(kspd_int) / 1280.0f;

        isReadKposKspd_ = false;

        // 暂时注释
        // if(target_kpos_ != current_kpos_ || target_kspd_ != current_kspd_)
        // {
        //     isSetKposKspd_ = true;
        // }
    }
    else if (extended_id.low_1 == -128)
    {
        // 电机报错，待处理
    }
    else if (extended_id.low_1 >= -127 && extended_id.low_1 <= 127)
    {
        current_atm_ = (float)extended_id.low_3;
        current_motor_temperature_ = extended_id.low_1;

        
        int32_t angle_int = (data.byte_3 << 24) | (data.byte_2 << 16) | (data.byte_1 << 8) | data.byte_0;
        float current_eangle = (float)angle_int / 32768 * 360;
        current_totalAngle_original_ = current_eangle / GEAR_RATIO_;
        current_totalAngle_ = current_totalAngle_original_ - current_totalAngle_offset_;
        current_angle_ = fmod(current_totalAngle_, 360.0f);
        if(current_angle_ < 0) 
            current_angle_ += 360.0f;
		
		if (isResetTotalAngle_)
        {
            current_totalAngle_offset_ = current_totalAngle_original_;
            isResetTotalAngle_ = false;
        }

        int16_t omega_int = (data.byte_5 << 8) | data.byte_4;
        float current_erpm = (float)omega_int / 256 * 60;
        current_rpm_ = current_erpm / GEAR_RATIO_;
        if(fabsf(current_rpm_) < 1.5f)
        {
            current_rpm_ = 0.0f;
        }

        int16_t torque_int = (data.byte_7 << 8) | data.byte_6;
        current_torque_ = (float)torque_int / 256;
    }
    else 
    {
        // 不应该发生的情况，待处理
    }
}

/**
 * @brief 设置电机Kpos和Kspd
 * @param kpos 电机刚度系数/位置误差比例系数
 * @param kspd 电机阻尼系数/速度误差比例系数
 */
void GO_Motor::setKposAndKspd(float kpos, float kspd)
{
    target_kpos_ = kpos;
    target_kspd_ = kspd;

    isSetKposKspd_ = true;
}

/**
 * @brief 重置电机控制参数，防止控制参数冲突
 */
void GO_Motor::resetParam()
{
    switch (mode_)
    {
    case Mode::SET_DEFAULT:
        target_torque_ = 0;
        target_rpm_ = 0;
        target_angle_ = 0;
        target_totalAngle_ = 0;
        break;
    case Mode::SET_TORQUE:
        target_rpm_ = 0;
        target_angle_ = 0;
        target_totalAngle_ = 0;
        break;
    case Mode::SET_RPM:
        target_angle_ = 0;
        target_totalAngle_ = 0;
        break;
    case Mode::SET_POS:
        // NONE
        break;
    default:
        // 不应该出现这种情况，待处理
        break;
    }
}

/**
 * @brief 设置目标输出轴总角度，单位度
 * @param totalAngle_set 目标输出轴总角度
 */
void GO_Motor::setTargetTotalAngle(float totalAngle_set)
{
    mode_ = Mode::SET_POS;
    resetParam();
    target_totalAngle_ = totalAngle_set;
}

/**
 * @brief 周期性被唤醒函数，可用于更新电机状态
 */
void GO_Motor::update()
{
    switch (mode_)
    {
    case Mode::SET_DEFAULT:
        // NONE
        break;
    case Mode::SET_TORQUE:
        // NONE
        break;
    case Mode::SET_RPM:
        target_torque_ = speed_pid_.pid_calc(target_rpm_, current_rpm_);
        break;
    case Mode::SET_POS:
        anglePid_timeCnt_++;
            if(anglePid_timeCnt_ >= anglePid_timePSC_)
            {
                float expected_rpm = angle_pid_.pid_calc(target_totalAngle_, current_totalAngle_);
                target_rpm_ = expected_rpm;
                anglePid_timeCnt_ = 0;
            }
            target_torque_ = speed_pid_.pid_calc(target_rpm_, current_rpm_);
        break;
    default:
        // 不应该出现这种情况，待处理
        break;
    }
}

/**
 * @brief 重置输出轴总角度为0度
 */
void GO_Motor::resetTotalAngle()
{
    isResetTotalAngle_ = true;
}

/**
 * @brief 初始化PID参数
 * @param speed_params 速度PID参数
 * @param speed_tdRatio 速度PID微分时间比例
 * @param angle_params 角度PID参数
 * @param angle_I_Separa 角度PID积分分离阈值
 */
void GO_Motor::pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa)
{
    speed_pid_.set_params(speed_params, speed_tdRatio);
    angle_pid_.set_params(angle_params, angle_I_Separa);
}

/**
 * @brief 获取当前输出轴转速
 * @return float 当前输出轴转速
 */
float GO_Motor::getRPM() const
{
    return current_rpm_;
}

/**
 * @brief 获取当前输出轴角度
 * @return float 当前输出轴角度
 */
float GO_Motor::getAngle() const
{
    return current_angle_;
}

/**
 * @brief 获取当前输出轴总角度
 * @return float 当前输出轴总角度
 */
float GO_Motor::getTotalAngle() const
{
    return current_totalAngle_;
}

/**
 * @brief 获取当前目标输出轴转速
 * @return float 当前目标输出轴转速
 */
float GO_Motor::getTargetRPM() const
{
    return target_rpm_;
}

/**
 * @brief 获取当前目标输出轴总角度
 * @return float 当前目标输出轴总角度
 */
float GO_Motor::getTargetTotalAngle() const
{
    return target_totalAngle_;
}

/**
 * @brief 获取当前输出轴转矩
 * @return float 当前输出轴转矩
 */
float GO_Motor::getTorque() const
{
    return current_torque_;
}
