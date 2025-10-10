/**
 * @file		Motor_GO.h
 * @brief
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-09-28 (创建日期)
 * @date        2025-10- (最后修改日期)
 * @platform	
 * @version     0.1.0
 * @details     
 * @note        1. 把KposKspd的设置和读取从程序里删掉
 *              2. 待解决GO电机编码器初始化指令要确认存在有效接收对象的问题
 * @warning		
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 0.1.0
 *      - 修订日期: 2025-10-
 *      - 主要变更:
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
#include <cmath>    


#include "Motor_Base.h"
#include "APP_PID.h"



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

    void pid_init(const PID_Param_Config& speed_params, float speed_tdRatio, const PID_Param_Config& angle_params, float angle_I_Separa);

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
     * @brief 设置目标输出轴转矩，单位N.m
     * @param torque_set 目标输出轴转矩
     */
    void setTargetTorque(float torque_set);

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
     * @brief 设置目标输出轴总角度，单位度
     * @param totalAngle_set 目标输出轴总角度
     */
    void setTargetTotalAngle(float totalAngle_set) override;

    /**
     * @brief 解析电机返回的CAN报文
     * @param cf 电机返回的CAN报文
     */
    void updateFeedback(const CanFrame& cf) override;

    /**
     * @brief 周期性被唤醒函数，可用于更新电机状态
     */
    void update() override;

    /**
     * @brief 重置输出轴总角度为0度
     */
    void resetTotalAngle();

    /**
     * @brief 获取当前输出轴转速
     * @return float 当前输出轴转速
     */
    float getRPM() const override;

    /**
     * @brief 获取当前输出轴角度
     * @return float 当前输出轴角度
     */
    float getAngle() const override;

    /**
     * @brief 获取当前输出轴总角度
     * @return float 当前输出轴总角度
     */
    float getTotalAngle() const override;

private:
    /**
     * @brief 设置电机Kpos和Kspd
     * @param kpos 电机刚度系数/位置误差比例系数
     * @param kspd 电机阻尼系数/速度误差比例系数
     */
    void setKposAndKspd(float kpos, float kspd);

    /**
     * @brief 重置电机控制参数，防止控制参数冲突
     */
    void resetParam();


    enum class Mode : uint8_t
    {
        SET_DEFAULT = 0, // 锁定模式
        SET_TORQUE, // 转矩模式
        SET_RPM, // 速度模式
        SET_POS, // 位置模式
    };

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
        MODE_2 = 2, // 接收到读取命令（控制模式12）可回读对应ID电机设置的KposKspd(返回内容:2)
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


    // 这两个都是先验量
    bool isInit_ = false; // 标记是否初始化，默认为否
    bool isReturnData_ = true; // 标记是否每控制一次电机就返回一次数据，默认为是
    
    // 这三个都是先验量
    Mode mode_ = Mode::SET_DEFAULT;
    Motor_Mode motor_mode_ = Motor_Mode::DEFAULT;
    Motor_Control_Mode motor_control_mode_ = Motor_Control_Mode::MODE_13; // 默认模式13


    // 这三个既是先验量，也是后验量
    bool isSetKposKspd_ = false; // 标记是否需要重新设置电机刚度系数和阻尼系数，默认为否
    bool isReadKposKspd_ = false; // 标记是否需要重新读取电机刚度系数和阻尼系数，默认为否
    bool isResetTotalAngle_ = true; // 标记是否需要重置输出轴总角度为0度，默认为是


    const float GEAR_RATIO_ = 6.33f; // GO电机减速比为6.33

    float target_kpos_ = 0.f; // 电机刚度系数/位置误差比例系数（输入）
    float target_kspd_ = 0.f; // 电机阻尼系数/速度误差比例系数（输入）

    float target_rpm_ = 0.f; // 目标输出轴转速
    float target_angle_ = 0.f; // 目标输出轴角度
    float target_totalAngle_ = 0.f; // 目标输出轴总角度
    float target_torque_ = 0.f; // 目标输出轴转矩


    float current_kpos_ = 0.f; // 电机刚度系数/位置误差比例系数
    float current_kspd_ = 0.f; // 电机阻尼系数/速度误差比例系数

    float current_angle_ = 0.f; // 当前输出轴角度
    float current_totalAngle_original_ = 0.f; // 当前输出轴总角度（原始值）
    float current_totalAngle_offset_ = 0.f; // 当前输出轴总角度（偏移值）
    float current_totalAngle_ = 0.f; // 当前输出轴总角度
    float current_rpm_ = 0.f; // 当前输出轴转速
    float current_torque_ = 0.f; // 当前输出轴转矩

    float current_atm_ = 0.f; // 当前气压，GO电机好像并不回传此项
    int8_t current_motor_temperature_ = 0; // 当前电机温度

    PID_Incremental speed_pid_;
    PID_Position angle_pid_;
};










#endif // __MOTOR_GO_H__
