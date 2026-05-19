/**
 * @file Motor_DM.h
 * @author 70er66
 * @brief 锟斤拷锟斤拷锟斤拷锟斤拷
 * @version 1.0
 * 
 * 锟斤拷锟侥硷拷锟斤拷锟斤拷锟斤拷锟斤拷J4310锟斤拷装
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
* 锟斤拷锟节碉拷锟斤拷锟斤拷秃锟侥壳把г褐伙拷锟紻M_J4310
  锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭?锟斤拷锟皆硷拷锟斤拷锟斤拷装锟斤拷锟斤拷锟斤拷锟斤拷锟较碉拷械锟斤拷
* 目前锟斤拷锟斤拷锟阶帮拷锟斤拷俣锟侥Ｊ斤拷锟轿伙拷锟侥Ｊ斤拷锟組IT模式锟斤拷锟斤拷锟叫碉拷模式锟斤拷锟斤拷锟斤拷dm_mode_锟叫伙拷锟斤拷同时锟借保锟斤拷锟斤拷锟斤拷位锟斤拷一锟铰ｏ拷
* 锟斤拷锟斤拷锟斤拷使锟斤拷锟斤拷知锟斤拷
  使锟斤拷前锟斤拷要锟斤拷锟斤拷位锟斤拷锟斤拷确锟斤拷锟斤拷锟侥Ｊ斤拷锟叫Ｗ硷拷锟斤拷锟?(锟斤拷锟斤拷注锟斤拷锟斤拷锟侥碉拷P_MAX,V_MAX,TFF_MAX锟饺诧拷锟斤拷锟斤拷要锟斤拷证锟斤拷锟斤拷位锟斤拷锟借定一锟铰ｏ拷锟斤拷为锟解几锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡碉拷锟斤拷锟斤拷锟斤拷校锟斤拷锟斤拷锟接帮拷斓斤拷锟斤拷锟斤拷锟斤拷帧锟侥达拷锟?)锟斤拷
  确锟斤拷锟斤拷锟絀D锟斤拷(锟斤拷锟斤拷锟斤拷锟斤拷ID锟斤拷锟斤拷要锟斤拷锟斤拷位锟斤拷锟斤拷锟借定锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷ID锟斤拷一锟斤拷锟斤拷Slave_ID,锟斤拷锟斤拷锟絀D锟斤拷锟斤拷帧锟斤拷ID锟斤拷锟节筹拷始锟斤拷锟斤拷时锟斤拷锟斤拷锟絠d锟斤拷锟斤拷一锟斤拷锟斤拷Master_ID,锟斤拷锟絀D锟角凤拷锟斤拷帧锟斤拷ID锟斤拷锟斤拷锟斤拷匹锟戒反锟斤拷帧锟斤拷锟节筹拷始锟斤拷锟斤拷时锟斤拷锟斤拷m_id);
  锟斤拷锟绞癸拷锟角帮拷锟揭?锟斤拷锟斤拷使锟斤拷帧锟斤拷锟节凤拷锟酵匡拷锟斤拷帧锟斤拷使锟斤拷锟斤拷锟斤拷锟斤拷也要锟斤拷一帧失锟斤拷帧锟斤拷
* attention锟斤拷锟饺诧拷要使锟斤拷锟劫讹拷模式锟斤拷锟劫讹拷模式锟斤拷帧锟斤拷式锟斤拷锟斤拷锟斤拷can锟侥凤拷锟斤拷锟叫筹拷突锟斤拷锟斤拷锟斤拷锟侥斤拷
* 锟斤拷锟斤拷锟斤拷锟铰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭恍㎝IT锟姐法锟较的匡拷锟斤拷锟窖撅拷锟斤拷展锟斤拷锟斤拷锟斤拷锟斤拷停锟斤拷锟斤拷锟斤拷锟街达拷锟斤拷锟斤拷锟揭讹拷锟街革拷锟斤拷锟?
*/

typedef enum {
	J4310_Type
} DM_MotorType;


typedef enum{
	MOTOR_MIT_MODE,        //MIT模式
	MOTOR_POSVEL_MODE,		//位锟斤拷锟劫讹拷模式
	MOTOR_VEL_MODE,        //锟劫讹拷模式
	MOTOR_DISABLE_MODE,		//锟斤拷锟绞э拷锟?		
	MOTOR_ENABLE_MODE,     //锟斤拷锟绞癸拷锟?
	MOTOR_SETZERO_MODE,   //锟斤拷锟斤拷锟斤拷锟?
	MOTOR_CLEARERR_MODE,	//锟斤拷锟斤拷锟斤拷锟?
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
//锟斤拷锟铰猴拷锟斤拷锟斤拷锟斤拷模式锟叫伙拷
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
	uint8_t getErrorNum(){return Error_num;}


private:
//锟斤拷锟铰憋拷锟斤拷锟斤拷锟节达拷欧锟斤拷锟街★拷锟斤拷锟斤拷锟斤拷牟锟斤拷锟?
/*------------------------------------------------------------------------------------*/
	uint8_t Master_Id;                          //锟斤拷锟斤拷帧ID
	uint8_t DM_Id;                              //锟斤拷锟絀D
	uint8_t Error_num;                          //锟斤拷锟斤拷锟斤拷
	
	int p_int,v_int,i_int;
	float angle,speed,tarque;      				//锟角讹拷 锟劫讹拷 锟斤拷锟斤拷
/*------------------------------------------------------------------------------------*/


//锟斤拷锟铰诧拷锟斤拷锟斤拷证锟斤拷锟斤拷位锟斤拷一锟斤拷
/*------------------------------------------------------------------------------------*/
	const float P_MIN = -12.5f, P_MAX = 12.5f;   //锟斤拷锟斤拷
    const float V_MIN = -30.0f, V_MAX = 30.0f;   //锟斤拷锟斤拷
    const float I_MAX = 18.0f, I_MIN = -18.0f;		
	const float KP_MAX =500.0f,KP_MIN=0.0f;
	const float KD_MAX =5.0f,KD_MIN=0.0f;
	const float TFF_MAX=10.0f,TFF_MIN=-10.0f;
	DM_MOTOR_MODE dm_mode_;
/*------------------------------------------------------------------------------------*/
	float P_des=0.0f;                           //目锟斤拷位锟斤拷
	float V_des=0.0f;							//目锟斤拷锟劫讹拷
	float Kp=0.0f,Kd=0.0f;						//PD锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	float T_ff=0.0f;							//	锟斤拷锟脚わ拷锟?
	const float pi=3.1415926;
	PID_Incremental dm_speed_pid_;
    PID_Position dm_angle_pid_;
	
	DM_MotorType type;
};


//std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;
#endif



#endif