/**
 * @file		Motor_GO.h
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


#ifndef __MOTOR_GO_H__
#define __MOTOR_GO_H__

#pragma once    // 再次冗余保证不重复包含

#if defined(__cplusplus) && __cplusplus < 201103L
#error "此文件需要支持C++11及以上编译环境,请确保编译器支持C++11或更高版本。"
#elif !defined(__cplusplus)
#error "此文件需要支持C++编译环境,请确保编译器支持__cplusplus宏。"
#endif

#include <cstring>
#include <cstdint>
#include <cstddef>

#include "Module_Encoder.h"
#include "Motor_Base.h"

/**
 * @brief 
 * @details 
 * @note 
 */
class GO_Motor : public Motor_Base
{
public:
    /**
     * @brief 构造函数
     * @param id 电机ID
     * @param bus CAN总线指针
     */
    GO_Motor(uint32_t id, fdCANbus *bus) : Motor_Base(id, true, bus){};

    ~GO_Motor(){};

    /**
     * @brief 检查CAN帧是否符合电机的报文格式
     * @param cf CAN帧
     * @return true 匹配成功
     * @return false 匹配失败
     */
    bool matchesFrame(const CanFrame& cf) const override;

    /**
     * @brief 打包命令
     * @param outFrames 输出CAN帧数组
     * @param maxFrames 输出CAN帧数组最大长度
     * @return std::size_t 实际打包的CAN帧数量
     */
    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    /**
     * @brief 设置目标输出轴转速，单位RPM
     * @param rpm_set 目标输出轴转速
     */
    void setTargetRPM(float rpm_set) override;

    /**
     * @brief 设置目标输出轴角度，单位度
     * @param angle_set 目标输出轴角度
     */
    void setTargetAngle(float angle_set) override;

    /**
     * @brief 解析电机返回的CAN报文
     * @param cf 电机返回的CAN报文
     */
    void updateFeedback(const CanFrame& cf) override;

    /**
     * @brief 读取电机当前模式
     */
    void readKposAndKspd();

private:
    enum class Motor_Mode : uint8_t
    {
        DEFAULT = 0, // 锁定
        FOC = 1, // FOC闭环
        CALIBRATION = 2, // 编码器校准
    };

    enum class Motor_Control_Mode : uint8_t
    {
        MODE_10 = 10, // 每控制一次电机CAN就返回一次电机数据，高频率下会导致can总线占满
        MODE_11 = 11, // 设置kpos和kspd
        MODE_12 = 12, // 读取kpos和kspd
        MODE_13 = 13, // 每控制一次电机CAN不返回电机数据除非电机报错，报错时会返回电机数据，用户需要电机数据时需要发送问答命令，电机将返回最后一次通讯时保留的数据
        MODE_2 = 2, // 发送读取命令（控制模式12）可回读对应ID电机设置的KposKspd(返回内容:2)
    };

    typedef struct CAN_extended_id_s
    {
        unsigned int module_id : 2;   // 2位：模块ID（0到3）
        unsigned int upload_or_download : 1;   // 1位：下发为0，上传为1
        unsigned int control_or_response : 2;   // 2位：控制为0，响应为1
        unsigned int low_3 : 8;   // 8位：低位3
        unsigned int low_2 : 8;   // 8位：低位2
        unsigned int low_1 : 8;   // 8位：低位1
        unsigned int reserved : 3;   // 3位：填充补全位，将29位的CAN ID段补全为32位
    } CAN_extended_id_t;

    typedef struct CAN_data_s
    {
        uint8_t byte_0; // 字节0
        uint8_t byte_1; // 字节1
        uint8_t byte_2; // 字节2
        uint8_t byte_3; // 字节3
        uint8_t byte_4; // 字节4
        uint8_t byte_5; // 字节5
        uint8_t byte_6; // 字节6
        uint8_t byte_7; // 字节7
    } CAN_data_t;

    Motor_Mode motor_mode_ = Motor_Mode::DEFAULT;
    Motor_Control_Mode motor_control_mode_ = Motor_Control_Mode::MODE_13; // 默认模式13


    float kp_ = 0.f; // 电机刚度系数/位置误差比例系数（输入）
    float kw_ = 0.f; // 电机阻尼系数/速度误差比例系数（输入）

    float target_rpm_ = 0.f; // 目标输出轴转速
    float target_angle_ = 0.f; // 目标输出轴角度
    float target_torque_ = 0.f; // 目标输出轴转矩


    float kpos_ = 0.f; // 电机刚度系数/位置误差比例系数
    float kspd_ = 0.f; // 电机阻尼系数/速度误差比例系数

    float current_angle_ = 0.f; // 当前输出轴角度
    float current_rpm_ = 0.f; // 当前输出轴转速
    float current_torque_ = 0.f; // 当前输出轴转矩

    float current_atm_ = 0.f; // 当前气压
    int8_t current_motor_temperature_ = 0; // 当前电机温度
    
};










#endif // __MOTOR_GO_H__
