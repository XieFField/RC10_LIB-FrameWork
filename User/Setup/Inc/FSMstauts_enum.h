/**
 * @file FSMstauts_enum.h
 * @author XieFField
 * @brief 状态机枚举
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
    ALL_STOP, //STOP状态

    MANUAL_CONTROL, //手动控制模式

    AUTO_CONTROL, //自动控制模式

    DEBUG_MODE, //调试模式
}FSM_Status_E;


typedef enum{
    ARM_MANUAL_CONTROL, //手操



    ARM_AUTO_CONTROL, //自动



    ARM_IDLE, //待机

    ARM_STOP,

    ARM_DEBUG,

    ARM_CALIBRATE, //校准


}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL_A, //手操A 无锁角
    CHASSIS_MANUAL_CONTROL_B, //手操B 有锁角
    CHASSIS_LOCK_FORWEAPON,    //无用
    CHASSIS_MANUAL_CONTROL_C, //手动控制模式C

    CHASSIS_TESTFOR_ARM, //测试模式Ϊṩȶƽ̨


    CHASSIS_CAMERA_DEBUG, // 无用
    CHASSIS_CAMERA, // 无用
    CHASSIS_AUTO_CONTROL_CB, //夹杆自动
    CHASSIS_AUTO_CONTROL_KFS, //KFS自动

    CHASSIS_STOP,
}CHASSIS_Status_E;

typedef enum{
    WEAPONSAGE_MANUAL_CONTROL, //手动控制
    WEAPONSAGE_AUTO_CONTROL_CATCH, //自动控制模式,抓取
    WEAPONSAGE_AUTO_CONTROL_DOCK, //自动控制模式, docking
    WEAPONSAGE_STOP,        //停止
    WEAPONSAGE_DEBUG,       //调试模式  
    WEAPONSAGE_CAMERA,      //摄像头模式
    WEAPONSAGE_IDLE,    // 待机
    WEAPONSAGE_CALIBRATE, //校准模式
}WeaponSage_Status_E;





#endif // __cplusplus


#endif // __FSM_STATUS_ENUM_H