#include "Locate_Setup.h"
float aaa;
void Locate_Setup::loop()
{
    // 在此处添加定位相关的周期性任务代码
    this->update();
	  
}

void Locate_Setup::update()
{
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
				x1=Laser_pos_instance->laser_instances[i]->Get_data()+delta_x1;
			}
			else if(i==1)
			{
				y1=Laser_pos_instance->laser_instances[i]->Get_data()+delta_y1;
			}
			else if(i==2)
			{
				y2=Laser_pos_instance->laser_instances[i]->Get_data()+delta_y2;
			}
		}
  }
	 float delta;
	 delta=fabs(y1-y2);
	 robot_pose_inWorld_.theta=atan(delta/d);
	 robot_pose_inWorld_.x=x1*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.y=(y1+y2)*cos(robot_pose_inWorld_.theta);
	 robot_pose_inWorld_.theta=robot_pose_inWorld_.theta*180/PI;
	
	 if(y1>y2)
	 {
		 robot_pose_inWorld_.theta=360-robot_pose_inWorld_.theta;
		 aaa=robot_pose_inWorld_.theta;
	 }
	
}