#include "Locate_Setup.h"
float aaa;
void Locate_Setup::loop()
{
    // 在此处添加定位相关的周期性任务代码
    this->update();
	  
}

void Locate_Setup::update()
{
	if(is_startToLRL_)
   		RobotPos_inWorld_caculate(this->Laser_pos_instance);
//   RobotPos_inWorld_caculate(this->Laser_pos_instance);
}

Point2D Locate_Setup::update_Lidar_data()
{
    
}

void Locate_Setup::RobotPos_inWorld_caculate(Laser_InstanceManager* Laser_pos_instance)
{
		for(int i=0;i<4;i++)	
	{
		for(int i=0;i<4;i++)	
	{
		if(Laser_pos_instance->laser_instances[i]!=nullptr)
		{
			if(i==0)
			{
				laser_initData_.x1=Laser_pos_instance->laser_instances[i]->Get_data()+laser_initData_.delta_x1;
			}
			else if(i==1)
			{
				laser_initData_.y1=Laser_pos_instance->laser_instances[i]->Get_data()+laser_initData_.delta_y1;
			}
			else if(i==2)
			{
				laser_initData_.y2=Laser_pos_instance->laser_instances[i]->Get_data()+laser_initData_.delta_y2;
			}
		}
  }
	 float delta;
	 delta=fabs(laser_initData_.y1-laser_initData_.y2);
	 robot_pose_inWorld_.theta=atan(delta/laser_initData_.d);
	 robot_pose_inWorld_.x=laser_initData_.x1*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.y=(laser_initData_.y1+laser_initData_.y2)*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.theta=robot_pose_inWorld_.theta*180/PI;
	
	 if(laser_initData_.y1>laser_initData_.y2)
	 {
		 robot_pose_inWorld_.theta=360-robot_pose_inWorld_.theta;
		 aaa=robot_pose_inWorld_.theta;
	 }
 }
}
Lader_position::Lader_position(USBD_HandleTypeDef *usb_handle) 
    :USB_CDC_(usb_handle)
{
}
Lader_position* Lader_position::GetInstance(USBD_HandleTypeDef *usb_handle)
{
static Lader_position instance(usb_handle);
return &instance;
}
void Lader_position::Callback_DCD_Fuc(uint8_t *buf, uint16_t len)
{
	uint8_t i = 0;
	uint8_t break_flag = 1;
	while (i < len && break_flag == 1)
		{
			/*-----------------------------------------处理数据--------------------*/
			switch (receive_flag)
			{
			case WAIT_HEAD_1:// 0xaa
				if (buf[i] == 0xaa) 
				{
					receive_flag = WAIT_HEAD_2;
				}
				i++;
				break;
				
			case WAIT_HEAD_2:// 0x55
				if (buf[i] == 0x55) receive_flag = WAIT_ID;
				else receive_flag = WAIT_HEAD_1;
			  i++;
				break;
				
			case WAIT_ID:// 1~MAX
				if (buf[i] > MAX_RECEIVE_ID || buf[i] == 0) receive_flag = WAIT_HEAD_1;
				else 
				{
					receive_id = buf[i];
					receive_flag = WAIT_LEN;
				}
				i++;
				break;
				
			case WAIT_LEN:
				if (buf[i] > MAX_RECEIVE_DATA_LEN) receive_flag = WAIT_HEAD_1;
				else
				{
					receive_len = buf[i];
					receive_flag = WAIT_DATA;
					receive_data_dx = 0;
				}
				i++;
				break;
				
			case WAIT_DATA:
				receive_data[receive_data_dx] = buf[i];
				receive_data_dx++;
				if (receive_data_dx >= receive_len) 
				{
					receive_flag = WAIT_CHECK;
					receive_data_dx = 0;
				}
				i++;
				break;
			
			case WAIT_CHECK:
				if (buf[i] == xor_check(receive_data, receive_len)) receive_flag = WAIT_TAIL;
				else receive_flag = WAIT_HEAD_1;
			  i++;
				break;
				
			case WAIT_TAIL:// 0xee
				if (buf[i] == 0xee)
				{
					/*-----------------------分发数据-------------------------*/
				if (receive_len == 16)
				{
					Lad_Data.x   = *(float*)(&receive_data[0]);
					Lad_Data.y   = *(float*)(&receive_data[4]);
					Lad_Data.z   = *(float*)(&receive_data[8]);
					Lad_Data.yaw = *(float*)(&receive_data[12]);
					
				}
					/*-----------------------分发数据-------------------------*/
				  
				}
				break_flag = 0;
				receive_flag = WAIT_HEAD_1;
				break;
			
			default:
				receive_flag = WAIT_HEAD_1;
		    i++;
				break;
			}

			/*-------------------------------------处理数据--------------------*/
		}
}

// XOR校验
uint8_t xor_check(const uint8_t *data, uint32_t length)
{
	uint8_t xor_val = 0;
	for (uint16_t i = 0; i < length; i++)
	{
		xor_val ^= data[i]; // 异或
	}
	return xor_val;
}
