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

#define USE_RC10_AIRJOY 0 //启用自制遥控器

typedef enum{
    ALL_STOP, //STOP状态

    MANUAL_CONTROL, //手动控制模式

    AUTO_CONTROL, //自动控制模式

    DEBUG_MODE, //调试模式
}FSM_Status_E;


typedef enum{
    SEND_ALL_STOP = 0x01, //STOP状态
    SEND_RELOCATE_LIDAR = 0x02, //重新定位雷达
    SEND_CHASSIS_MANUAL_CTRL = 0x03, //底盘手动控制
    SEND_ARM_MANUAL_CTRL = 0x04, //机械臂手动控制
    SEND_WEAPON_MANUAL_CTRL = 0x05, //武器系统手动控制
    SEND_CHASSIS_WAIT_AUTO = 0x06, //底盘等待自动控制
    SEND_ARM_SEMI_AUTO_1 = 0x07, //机械臂半自动控制
    SEND_ARM_SEMI_AUTO_2 = 0x08, //机械臂半自动控制
    SEND_ARM_AUTO = 0x09, //机械臂自动控制
    SEND_WEAPONSAGE_SEMI_AUTO_1 = 0x0A, //武器系统半自动控制
    SEND_WEAPONSAGE_SEMI_AUTO_2 = 0x0B, //武器系统半自动控制
    SEND_WEAPONSAGE_AUTO = 0x0C //武器系统自动控制
}SEND_MODE_TO_AIRJOY_E;

typedef enum{
    SEND_COMBINE_CMD = 0x01, //发送合体指令
    SEND_DOCK_SUCCESS = 0x02, //对接成功
    SEND_DOCK_FAIL = 0x03, //对接失败

    SEND_PUT_DOWN_LOW = 0x04, //放置低位
    SEND_PUT_DOWN_HIGH = 0x05, //放置高位
}SEND_CMD_TO_R2;

typedef enum{
    ARM_MANUAL_CONTROL, //手操

    ARM_AUTO_CONTROL, //自动


    ARM_SEMI_AUTO_CONTROL_1, //半自动
    ARM_SEMI_AUTO_CONTROL_2, //半自动
    ARM_IDLE, //待机

    ARM_STOP,

    ARM_DEBUG,

    ARM_CALIBRATE, //校准
}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL_A, //手操A 无锁角
    CHASSIS_MANUAL_CONTROL_B, //手操B 有锁角
    CHASSIS_MANUAL_CONTROL_C,
    CHASSIS_LOCK_FORWEAPON,    //对接

    CHASSIS_AUTO_CONTROL_CB, //夹杆自动
    CHASSIS_AUTO_CONTROL_KFS, //KFS自动

    CHASSIS_STOP,
}CHASSIS_Status_E;

typedef enum{
    WEAPONSAGE_MANUAL_CONTROL, //手动控制


    WEAPONSAGE_AUTO_CONTROL, //自动控制
    WEAPONSAGE_SEMI_AUTO_CONTROL_1, //半自动控制
    WEAPONSAGE_SEMI_AUTO_CONTROL_2, //半自动控制

    WEAPONSAGE_AUTO_CONTROL_CATCH, //自动控制模式,抓取
    WEAPONSAGE_AUTO_CONTROL_DOCK, //自动控制模式, docking
	WEAPONSAGE_AUTOCONTROL,

    WEAPONSAGE_STOP,        //停止
    WEAPONSAGE_DEBUG,       //调试模式  
    WEAPONSAGE_IDLE,    // 待机
    WEAPONSAGE_CALIBRATE, //校准模式
}WeaponSage_Status_E;



#endif // __cplusplus


#endif // __FSM_STATUS_ENUM_H