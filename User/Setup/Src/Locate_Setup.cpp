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