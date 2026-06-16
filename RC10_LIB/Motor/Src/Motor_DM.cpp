#include "Motor_DM.h"

DM_Motor::DM_Motor(DM_MotorType type, uint32_t m_id,uint32_t id, fdCANbus *bus)
: Motor_Base(id, false, bus),type(type)
{
	DM_Id=id;
	Master_Id=m_id;
}
void DM_Motor::updateFeedback(const CanFrame& cf) 
{
		const uint8_t* can_rx_data = cf.data;
      
			DM_Id = (can_rx_data[0]&0x0F);
			Error_num=((can_rx_data[0]>>4)&0x0F);
            p_int = (can_rx_data[1] << 8) | can_rx_data[2];         // 电机位置数据
            v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);  // 电机速度数据
            i_int = ((can_rx_data[4] & 0xF) << 8) | can_rx_data[5]; // 电机扭矩数据

            angle = uint_to_float(p_int, P_MIN, P_MAX, 16); // 转子机械角度
            speed = uint_to_float(v_int, V_MIN, V_MAX, 12); // 实际转子转速
            tarque = uint_to_float(i_int, I_MIN, I_MAX, 12);        // 实际转矩电流
}
void DM_Motor::setTargetTotalAngle(float v_target ,float totalAngle_set)
{
	V_des=(v_target*pi)/180;
	P_des=(totalAngle_set*pi)/180;
	dm_mode_=MOTOR_POSVEL_MODE;
}

void DM_Motor::setTargetRPM(float rpm_set)
{
	V_des=(rpm_set*pi)/180;
	dm_mode_=MOTOR_VEL_MODE;
}

void DM_Motor::setMIT(float pos,float vel,float kp, float kd ,float t_ff )
{
	P_des=(pos*pi)/180;
	V_des=(vel*pi)/180;
	Kp=kp;
	Kd=kd;
	T_ff=t_ff;
	dm_mode_=MOTOR_MIT_MODE;
}

float DM_Motor::getAngle()const 
	{
		float current_angle;
		current_angle=fmodf(angle,360.0f);
		return current_angle;
	}

float DM_Motor:: uint_to_float(int x_int, float x_min, float x_max, int bits)
{
 /// converts unsigned int to float, given range and number of bits ///
	float span = x_max- x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int DM_Motor::float_to_uint(float x,float x_min, float x_max, int bits)
{
  /// Converts afloat to anunsigned int, given range and number ofbits///
  float span = x_max-x_min;
  float offset =x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

std::size_t DM_Motor:: packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
	if(maxFrames < 1)
        return 0; // 无法打包

    CanFrame &cf = outFrames[0];
    static int32_t sendMsgs = 0;
    cf.DLC = 8; //8
    cf.isextended = false;
    std::memset(cf.data, 0, 8);
	switch (dm_mode_)
	{
		case MOTOR_MIT_MODE:
		{
			
			 if(Error_num==0x01)
            {
				
				int16_t pos=0,vel=0,kp=0,kd=0,t_ff=0;
				pos=float_to_uint(P_des,P_MIN,P_MAX,16);
				vel=float_to_uint(V_des,V_MIN,V_MAX,12);
				kp=float_to_uint(Kp,KP_MIN,KP_MAX,12);
				kd=float_to_uint(Kd,KD_MIN,KD_MAX,12);
				t_ff=float_to_uint(T_ff,TFF_MIN,TFF_MAX,12);
                cf.ID = DM_Id;
                cf.data[0] = (pos >> 8 )& 0xFF;
                cf.data[1] =  pos & 0xFF;
                cf.data[2] = (vel>>4) & 0xFF;
				cf.data[3] = ((vel<<4)|(kp>>8))&0xFF;
				cf.data[4] = kp&0xFF;
				cf.data[5] = (kd>>4);
				cf.data[6] = ((kd&0xF)<<4)|(t_ff>>8);
				cf.data[7] = t_ff;
				
            }
				break;
           }

		case MOTOR_POSVEL_MODE:
		{
			if(Error_num==0x01)
                {
                uint8_t *vbuf,*pbuf;
                vbuf=  (uint8_t *)&this->V_des;
                pbuf = (uint8_t *)&this->P_des;
                cf.ID = 0x100|DM_Id;
                for(int i =0;i<4;i++)
                {
                    cf.data[i]=*(pbuf+i);
                }
                for(int i=4;i<8;i++)
                {
                    cf.data[i]=*(vbuf+i-4);
                }
			}
            else
                return 0;
			break;
		}
		case MOTOR_VEL_MODE:
		{
			if(Error_num==0x01)
			{
			cf.ID = 0x200|DM_Id;
			cf.DLC = 8;
			uint8_t *vbuf;
			vbuf= (uint8_t*)&this->V_des;
			
			for (int i=0;i<4;i++)
			{
				cf.data[i]=*(vbuf+i);
			}
			}
			break;
		}
		case MOTOR_ENABLE_MODE:
		{
			cf.ID=DM_Id;
			for(int i =0;i<7;i++)
			{
				cf.data[i]=0xFF;
			}
			cf.data[7]=0xFC;
			break;
		}
		case MOTOR_DISABLE_MODE:
		{
			cf.ID=DM_Id;
			for(int i =0;i<7;i++)
			{
				cf.data[i]=0xFF;
			}
			cf.data[7]=0xFD;
			break;
		}
		case MOTOR_SETZERO_MODE:
		{
			cf.ID=DM_Id;
			for(int i =0;i<7;i++)
			{
				cf.data[i]=0xFF;
			}
			cf.data[7]=0xFE;
			break;
		}
		case MOTOR_CLEARERR_MODE:
		{
				for(int i =0;i<7;i++)
			{
				cf.data[i]=0xFF;
			}
			cf.data[7]=0xFB;
			break;
		}
		default:
		{
			break;
		}
	}
		return 1;
}


//后续更新，敬请期待！(*^▽^*)
void DM_Motor::update()
{
	switch(dm_mode_)
	{
		case MOTOR_MIT_MODE:
		{
			break;
		}
		case  MOTOR_POSVEL_MODE:
		{
			break;
		}
		case MOTOR_VEL_MODE:
		{
			break;
		}
	}
}