/**
 * @file FSMstauts_enum.h
 * @author XieFField
 * @brief 状态机相关的数据库
 */

#ifndef __FSM_STATUS_ENUM_H
#define __FSM_STATUS_ENUM_H

#ifdef __cplusplus
extern "C" {

}
#endif

#ifdef __cplusplus

#include <iostream>
#include <cmath>  
#include "APP_tool.h"

typedef enum{
    ALL_STOP, //全部机构停止工作

    MANUAL_CONTROL, //手动控制模式

    AUTO_CONTROL, //半自动控制模式

    DEBUG_MODE, //调试模式
}FSM_Status_E;


typedef enum{
    ARM_MANUAL_CONTROL, //串联臂手动控制模式



    ARM_AUTO_CONTROL, //串联臂自动控制模式



    ARM_IDLE, //串联臂空闲状态，维持当前状态

    ARM_STOP,

    ARM_DEBUG,

    ARM_CALIBRATE, //串联臂校准模式


}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL_A, //底盘手动控制模式（角速度可控）
    CHASSIS_MANUAL_CONTROL_B, //底盘手动控制模式（角速度为0，锁定角度）
    CHASSIS_LOCK_FORWEAPON,    //底盘锁定yaw，为武器大师提供稳定平台
    CHASSIS_MANUAL_CONTROL_C, //底盘手动控制模式C

    CHASSIS_TESTFOR_ARM, //底盘测试模式，为串联臂提供稳定平台


    CHASSIS_CAMERA_DEBUG, // 视觉调试模式
    CHASSIS_CAMERA, // 相机闭环模式
    CHASSIS_AUTO_CONTROL_CB, //底盘自动控制模式
    CHASSIS_AUTO_CONTROL_KFS, //底盘自动控制模式梅林内

    CHASSIS_STOP,
}CHASSIS_Status_E;

typedef enum{
    WEAPONSAGE_MANUAL_CONTROL, //手操
    WEAPONSAGE_AUTO_CONTROL, //自动控制模式
    WEAPONSAGE_STOP,        //停止
    WEAPONSAGE_DEBUG,       //调试模式  
    WEAPONSAGE_CAMERA,      //相机协同模式
    WEAPONSAGE_IDLE,    //空闲状态，维持当前状态
    WEAPONSAGE_CALIBRATE, //校准模式
}WeaponSage_Status_E;


//遥控临时调试使用
typedef struct{
    uint16_t SWA;
    uint16_t SWB;
    uint16_t SWC;
    uint16_t SWD;
    uint16_t LEFT_X;
    uint16_t LEFT_Y;
    uint16_t RIGHT_X;
    uint16_t RIGHT_Y;
    
}airjoy_S;




#endif // __cplusplus


#endif // __FSM_STATUS_ENUM_H