/**
 * @file Motor_DM.h
 * @author 70er66
 * @brief ��������
 * @version 1.0
 * 
 * ���ļ���������J4310��װ
 */
 
 
 
#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H
#pragma once
#ifdef __cplusplus
extern "C" {
    #include <stdint.h>
    #include "BSP_CanFrame.h"
	
}
#endif

#ifdef __cplusplus

#include "Motor_Base.h"
#include "APP_tool.h"
#include "APP_PID.h"
#include <cstring>
#include "Module_Encoder.h"
#include "arm_math.h"

/*
* ���ڵ�����ͺ�ĿǰѧԺֻ��DM_J4310
  ������������?���Լ�����װ���������ϵ�е��
* Ŀǰ�����װ���ٶ�ģʽ��λ��ģʽ��MITģʽ�����е�ģʽ������dm_mode_�л���ͬʱ�豣������λ��һ�£�
* ������ʹ����֪��
  ʹ��ǰ��Ҫ����λ����ȷ�����ģʽ��У׼����?(����ע�����ĵ�P_MAX,V_MAX,TFF_MAX�Ȳ�����Ҫ��֤����λ���趨һ�£���Ϊ�⼸�����������������Ƶ�������У�����Ӱ�쵽�������֡�Ĵ��?)��
  ȷ�����ID��(��������ID����Ҫ����λ�����趨������������ID��һ����Slave_ID,�����ID����֡��ID���ڳ�ʼ����ʱ�����id����һ����Master_ID,���ID�Ƿ���֡��ID������ƥ�䷴��֡���ڳ�ʼ����ʱ����m_id);
  ���ʹ��ǰ���?����ʹ��֡���ڷ��Ϳ���֡��ʹ��������ҲҪ��һ֡ʧ��֡��
* attention���Ȳ�Ҫʹ���ٶ�ģʽ���ٶ�ģʽ��֡��ʽ������can�ķ����г�ͻ�������Ľ�
* �������£����������һЩMIT�㷨�ϵĿ����Ѿ���չ��������ͣ�������ִ������Ҷ��ָ����?
*/

typedef enum {
	J4310_Type
} DM_MotorType;


typedef enum{
	MOTOR_MIT_MODE,        //MITģʽ
	MOTOR_POSVEL_MODE,		//λ���ٶ�ģʽ
	MOTOR_VEL_MODE,        //�ٶ�ģʽ
	MOTOR_DISABLE_MODE,		//���ʧ��?		
	MOTOR_ENABLE_MODE,     //���ʹ��?
	MOTOR_SETZERO_MODE,   //�������?
	MOTOR_CLEARERR_MODE,	//�������?
}DM_MOTOR_MODE;



class DM_Motor : public Motor_Base
{
public:
	 DM_Motor(DM_MotorType type, uint32_t m_id,uint32_t id, fdCANbus *bus);
    ~DM_Motor(){};
	bool matchesFrame(const CanFrame& cf) const override
    {
		if(cf.ID!=Master_Id||cf.isextended)
			return false;
		else
			return (cf.ID==Master_Id);
    }
//	
	void updateFeedback(const CanFrame& cf) override;
	
	void update() override;
	
	std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames);
//���º�������ģʽ�л�
/*------------------------------------------------------------------------------------*/	
	void setTargetTotalAngle(float v_target ,float totalAngle_set); 
    void setTargetRPM(float rpm_set);
	void setMIT(float pos,float vel,float kp, float kd ,float t_ff);
	void motorEnable(void){dm_mode_=MOTOR_ENABLE_MODE;}
	void motorDisable(void){dm_mode_=MOTOR_DISABLE_MODE;}
	void motorSetZero(void){dm_mode_=MOTOR_SETZERO_MODE;}
	void motorClearErr(void){dm_mode_=MOTOR_CLEARERR_MODE;}
/*------------------------------------------------------------------------------------*/

	float getCurrentVel(){return v_int;}
	float getCurrentPos(){return p_int;}
	float getCurrentID(){return DM_Id; }
    
    float getTotalAngle() const override { return angle * 180.0f / 3.1415926f; }
	
	float getAngle() const override;
	
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
	int float_to_uint(float x,float x_min, float x_max, int bits);



private:
//���±������ڴ�ŷ���֡�������Ĳ���?
/*------------------------------------------------------------------------------------*/
	uint8_t Master_Id;                          //����֡ID
	uint8_t DM_Id;                              //���ID
	uint8_t Error_num;                          //������
	
	int p_int,v_int,i_int;
	float angle,speed,tarque;      				//�Ƕ� �ٶ� ����
/*------------------------------------------------------------------------------------*/


//���²�����֤����λ��һ��
/*------------------------------------------------------------------------------------*/
	const float P_MIN = -12.5f, P_MAX = 12.5f;   //����
    const float V_MIN = -30.0f, V_MAX = 30.0f;   //����
    const float I_MAX = 18.0f, I_MIN = -18.0f;		
	const float KP_MAX =500.0f,KP_MIN=0.0f;
	const float KD_MAX =5.0f,KD_MIN=0.0f;
	const float TFF_MAX=10.0f,TFF_MIN=-10.0f;
	DM_MOTOR_MODE dm_mode_;
/*------------------------------------------------------------------------------------*/
	float P_des=0.0f;                           //Ŀ��λ��
	float V_des=0.0f;							//Ŀ���ٶ�
	float Kp=0.0f,Kd=0.0f;						//PD����������
	float T_ff=0.0f;							//	���Ť��?
	const float pi=3.1415926;
	PID_Incremental dm_speed_pid_;
    PID_Position dm_angle_pid_;
	
	DM_MotorType type;
};


//std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;
#endif



#endif