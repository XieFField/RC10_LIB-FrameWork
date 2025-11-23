/**
 * @file Motor_DM.h
 * @author 70er66
 * @brief 达妙电机类
 * @version 1.0
 * 
 * 此文件包含达妙J4310封装
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

/*
* 关于电机的型号目前学院只有DM_J4310
  后续如果有需要可以继续封装达妙的其他系列电机
* 目前达妙封装了速度模式，位置模式和MIT模式；其中的模式是依靠dm_mode_切换，同时需保持与上位机一致；
* 达妙电机使用须知：
  使用前需要在上位机上确定电机模式，校准参数(尤其注意下文的P_MAX,V_MAX,TFF_MAX等参数需要保证与上位机设定一致，因为这几个参数不仅仅是限制电机的运行，还会影响到后面控制帧的打包)；
  确定电机ID号(达妙电机的ID号需要再上位机中设定，其中有两个ID：一个是Slave_ID,这个是ID控制帧的ID，在初始化的时候就是id，另一个是Master_ID,这个ID是反馈帧的ID，用于匹配反馈帧，在初始化的时候是m_id);
  电机使用前需要发送使能帧，在发送控制帧，使用完后最好也要发一帧失能帧；
* attention：先不要使用速度模式，速度模式的帧格式与框架中can的发送有冲突待后续改进
* 后续更新：后续会更新一些MIT算法上的控制已经拓展电机的类型，如果发现错误还请大家多多指正；
*/

typedef enum {
	J4310_Type
} DM_MotorType;


typedef enum{
	MOTOR_MIT_MODE,        //MIT模式
	MOTOR_POSVEL_MODE,		//位置速度模式
	MOTOR_VEL_MODE,        //速度模式
	MOTOR_DISABLE_MODE,		//电机失能		
	MOTOR_ENABLE_MODE,     //电机使能
	MOTOR_SETZERO_MODE,   //保存零点
	MOTOR_CLEARERR_MODE,	//清楚错误
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
//以下函数用于模式切换
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
	
	
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
	int float_to_uint(float x,float x_min, float x_max, int bits);



private:
//以下变量用于存放反馈帧解析到的参数
/*------------------------------------------------------------------------------------*/
	uint8_t Master_Id;                          //反馈帧ID
	uint8_t DM_Id;                              //电机ID
	uint8_t Error_num;                          //错误编号
	
	int p_int,v_int,i_int;
	float angle,speed,tarque;      				//角度 速度 力矩
/*------------------------------------------------------------------------------------*/


//以下参数保证与上位机一致
/*------------------------------------------------------------------------------------*/
	const float P_MIN = -12.5f, P_MAX = 12.5f;   //弧度
    const float V_MIN = -30.0f, V_MAX = 30.0f;   //弧度
    const float I_MAX = 18.0f, I_MIN = -18.0f;		
	const float KP_MAX =500.0f,KP_MIN=0.0f;
	const float KD_MAX =5.0f,KD_MIN=0.0f;
	const float TFF_MAX=10.0f,TFF_MIN=-10.0f;
	DM_MOTOR_MODE dm_mode_;
/*------------------------------------------------------------------------------------*/
	float P_des=0.0f;                           //目标位置
	float V_des=0.0f;							//目标速度
	float Kp=0.0f,Kd=0.0f;						//PD控制器参数
	float T_ff=0.0f;							//	电机扭矩
	const float pi=3.1415926;
	PID_Incremental dm_speed_pid_;
    PID_Position dm_angle_pid_;
	
	DM_MotorType type;
};


//std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;
#endif



#endif