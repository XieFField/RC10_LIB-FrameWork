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

#define USE_RC10_AIRJOY 1 //启用自制遥控器
#define CHALLENGE_3ZONE 0 //挑战赛3区专用代码

typedef enum{
    ALL_STOP, //STOP状态

    MANUAL_CONTROL, //手动控制模式

    AUTO_CONTROL, //自动控制模式

    DEBUG_MODE, //调试模式
}FSM_Status_E;


typedef enum{
    SEND_ALL_STOP = 0x01, //STOP状态
    SEND_RELOCATE_LIDAR = 0x02, //重新定位雷达
    SEND_SET_SPEAR = 0x03, //设置矛杆
    SEND_CHASSIS_MANUAL_CTRL = 0x04, //底盘手动控制
    SEND_ARM_MANUAL_CTRL = 0x05, //机械臂手动控制
    SEND_WEAPON_MANUAL_CTRL = 0x06, //武器系统手动控制
    
    SEND_COMBINE_MODE = 0x07, //发送合体模式
    SEND_COMP_ARM = 0x08, // 竞技场机械臂模式
    SEND_COMP_WEAPON = 0x09, // 竞技场武器模式
    SEND_ARM_AUTO = 0x0A, //机械臂自动模式
    SEND_ARM_SEMI = 0x0B, //机械臂半自动模式
    SEND_WEAPON_AUTO = 0x0C, //武器系统自动模式
    SEND_WEAPON_SEMI = 0x0D, //武器系统半自动模式
    SEND_ARM_LOW_MANUAL_LEVEL = 0x0E, //机械臂减配手操
    SEND_WEAPON_LOW_MANUAL_LEVEL = 0x0F, //武器系统减配手操
}SEND_MODE_TO_AIRJOY_E;

typedef enum{
    SEND_NONE = 0x00, //无指令
    SEND_DOCK_SUCCESS = 0x01, //对接成功
    SEND_COMBINE_CMD = 0x03, //发送合体指令
    SEND_WAIT_COMBINE = 0x02, //等待合体指令
    SEND_PUT_DOWN_HIGH = 0x04, //放置高位
    SEND_PUT_DOWN_LOW_Left = 0x05, //放置低位左
    SEND_PUT_DOWN_LOW_Right = 0x06, //放置低位右
    SEND_PUT_DOWN_LOW_Mid = 0x07, //放置低位中
    
}SEND_CMD_TO_R2;

typedef enum{
    ARM_MANUAL_CONTROL, //手操 (十字键不包含放置模式)
    ARM_AUTO_CONTROL, //自动
    ARM_SEMI_AUTO_CONTROL, //半自动 (十字键模式，没有放置功能)
    ARM_COMP_SEMI_CONTROL, //竞技场半自动 (十字键模式，包含放置功能，不包含拾取功能)
    ARM_MANUAL_LOW_LEVEL, // 减配版手操   (十字键不包含放置模式)
    ARM_IDLE, //待机
    ARM_STOP,
    ARM_DEBUG,
    ARM_CALIBRATE, //校准

    ARM_CHALLENGE_3ZONE, //挑战赛3区专用代码

    ARM_COMBINE_IDLE, //合体待机
}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL_A, //手操A 无锁角
    CHASSIS_MANUAL_CONTROL_B, //手操B 有锁角
    CHASSIS_MANUAL_CONTROL_C,
    CHASSIS_MANUAL_CONTROL_D,
    CHASSIS_MANUAL_CONTROL_CZ,//xy换

    CHASSIS_AUTO_CONTROL_CB, //夹杆自动
    CHASSIS_AUTO_CONTROL_KFS, //KFS自动
    
    SEMI_AUIO_CZ_FIT,//三区合体模式
    SEMI_AUIO_CZ_ARM,//三区机械臂模式
    SEMI_AUIO_CZ_WEAPON,//三区武器模式
    
    SEMI_AUIO_CZ_ARM_Challenge,
    
    CHASSIS_AUTO_CONTROL_CZ_R1, //对抗区R1自身自动
    CHASSIS_AUTO_CONTROL_CZ_R2, //对抗区R2合体自动
    
    CHASSIS_STOP,
}CHASSIS_Status_E;

typedef enum{
    WEAPONSAGE_MANUAL_CONTROL, //手动控制
    WEAPONSAGE_SEMI_AUTO_CONTROL, //半自动控制  只保留十字键逻辑
    WEAPONSAGE_MANUAL_LOW_LEVEL, //减配版手操
    WEAPONSAGE_COMP_MANUAL_CONTROL, //竞技场手操

	WEAPONSAGE_AUTOCONTROL,

    WEAPONSAGE_STOP,        //停止
    WEAPONSAGE_DEBUG,       //调试模式  
    WEAPONSAGE_IDLE,    // 待机
    WEAPONSAGE_CALIBRATE, //校准模式
    WEAPONSAGE_KFS_IDLE, // KFS自动模式的待机状态
    WEAPONSAGE_TEST_POINT,
}WeaponSage_Status_E;



#endif // __cplusplus


#endif // __FSM_STATUS_ENUM_H