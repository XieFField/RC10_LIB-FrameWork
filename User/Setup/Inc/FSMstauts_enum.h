/**
 * @file FSMstauts_enum.h
 * @author XieFField
 * @brief ״̬����ص����ݿ�
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
    ALL_STOP, //ȫ������ֹͣ����

    MANUAL_CONTROL, //�ֶ�����ģʽ

    AUTO_CONTROL, //���Զ�����ģʽ

    DEBUG_MODE, //����ģʽ
}FSM_Status_E;


typedef enum{
    ARM_MANUAL_CONTROL, //�������ֶ�����ģʽ

    ARM_AUTO_CONTROL, //�������Զ�����ģʽ

    ARM_IDLE, //�����ۿ���״̬��ά�ֵ�ǰ״̬

    ARM_STOP,

    ARM_DEBUG,

    ARM_CALIBRATE, //������У׼ģʽ
}ARM_Status_E;


typedef enum{
    CHASSIS_MANUAL_CONTROL_A, //�����ֶ�����ģʽ�����ٶȿɿأ�
    CHASSIS_MANUAL_CONTROL_B, //�����ֶ�����ģʽ�����ٶ�Ϊ0�������Ƕȣ�

    CHASSIS_AUTO_CONTROL, //�����Զ�����ģʽ

    CHASSIS_STOP,
}CHASSIS_Status_E;

typedef enum{
    WEAPONSAGE_MANUAL_CONTROL, //�ֲ�
    WEAPONSAGE_AUTO_CONTROL, //�Զ�����ģʽ
    WEAPONSAGE_STOP,        //ֹͣ
    WEAPONSAGE_DEBUG,       //����ģʽ  
    WEAPONSAGE_IDLE,    //����״̬��ά�ֵ�ǰ״̬
    WEAPONSAGE_CALIBRATE,   //У׼ģʽ
}WeaponSage_Status_E;


//ң����ʱ����ʹ��
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