#include "Locate_Setup.h"
#include "semphr.h"
float aaa;
void Locate_Setup::loop()
{
    // 在此处添加定位相关的周期性任务代码
	  uint8_t a =0x11;
    usb_handle->CDC_Send_(0x04,&a,0x01);
	  Get_Rader_Data();
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
	if(laser_mode==LEFT){
	 delta=fabs(laser_initData_.y1-laser_initData_.y2);
	 robot_pose_inWorld_.theta=atan(delta/laser_initData_.d);
	 robot_pose_inWorld_.x=laser_initData_.x1*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.y=0.5*(laser_initData_.y1+laser_initData_.y2)*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.theta=robot_pose_inWorld_.theta*180/PI;
	
	 if(laser_initData_.y1>laser_initData_.y2)
	 {
		 robot_pose_inWorld_.theta=360-robot_pose_inWorld_.theta;
		 aaa=robot_pose_inWorld_.theta;
	 }
 }
	else if(laser_mode==RIGHT)
	{
	 delta=fabs(laser_initData_.y1-laser_initData_.y2);
	 robot_pose_inWorld_.theta=atan(delta/laser_initData_.d);
	 robot_pose_inWorld_.y=laser_initData_.x1*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.x=0.5*(laser_initData_.y1+laser_initData_.y2)*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.theta=robot_pose_inWorld_.theta*180/PI;
	
	 if(laser_initData_.y1>laser_initData_.y2)
	 {
		 robot_pose_inWorld_.theta=360-robot_pose_inWorld_.theta;
		 aaa=robot_pose_inWorld_.theta;
	 }
		
	}
	
}
void Locate_Setup::USB_SendData()
 {
	uint8_t a =0x00;
  usb_handle->CDC_Send_(0x04,&a,0x01);

 }

 void Locate_Setup::Get_Rader_Data()
 {
	  Lad_Data.x   = usb_handle->Data_.data1[0];
    Lad_Data.y   = usb_handle->Data_.data1[1];
    Lad_Data.z   = usb_handle->Data_.data1[2];
	  Lad_Data.roll= usb_handle->Data_.data1[3];
	  Lad_Data.pitch= usb_handle->Data_.data1[4];
	  Lad_Data.yaw= usb_handle->Data_.data1[5];
		Lad_Data.line_x= usb_handle->Data_.data1[6];
		Lad_Data.line_y= usb_handle->Data_.data1[7];
		Lad_Data.line_z= usb_handle->Data_.data1[8];
 }