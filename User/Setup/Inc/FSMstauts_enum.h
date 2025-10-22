#ifndef __FSM_STATUS_ENUM_H
#define __FSM_STATUS_ENUM_H

#ifdef __cplusplus
extern "C" {

}
#endif

#ifdef __cplusplus

typedef enum{
    ALL_STOP, //全部机构停止工作

    MANUAL_CONTROL, //手动控制模式

    AUTO_CONTROL, //半自动控制模式
}FSM_Status_E;


typedef enum{
    ARM_MANUAL_CONTROL, //串联臂手动控制模式

    ARM_AUTO_CONTROL, //串联臂自动控制模式

    ARM_IDLE, //串联臂空闲状态

    ARM_STOP,
}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL, //底盘手动控制模式

    CHASSIS_AUTO_CONTROL, //底盘自动控制模式

    CHASSIS_IDLE, //底盘空闲状态

    CHASSIS_STOP,
}CHASSIS_Status_E;


#endif // __cplusplus


#endif // __FSM_STATUS_ENUM_H