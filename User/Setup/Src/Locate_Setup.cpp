#include "Locate_Setup.h"
#include "semphr.h"
float aaa;
void Locate_Setup::loop()
{
    // 锟节此达拷锟斤拷锟接讹拷位锟斤拷氐锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
//	uint8_t a =0x11;
//    usb_handle->CDC_Send_(0x04,&a,0x01);
	Get_Rader_Data();
    this->update();
}

void Locate_Setup::update()
{
    Point2D fk_speed;
    if(SpeedFK_Queue.recv(fk_speed, 0))
    {
        // 底盘正解算速度fk_speed
		fk_chassisSpeed_inWorld_.x = fk_speed.x;
		fk_chassisSpeed_inWorld_.y = fk_speed.y;
		fk_chassisSpeed_inWorld_.theta = fk_speed.theta;
    }

	if(is_startToLRL_)
   		RobotPos_inWorld_caculate(this->Laser_pos_instance);

	update_Lidar_data();

	lader_transform_caculate();

    // yaw_from_position_ = Position::GetInstance(&huart1)->getRealPosData().world_yaw;
	// dyaw_from_position_ = Position::GetInstance(&huart1)->getRealPosData().dyaw;


    yaw_from_position_ = -HWT101CT::GetInstance(&huart1)->get_yaw_rad() / PI * 180.0f;
	dyaw_from_position_ = HWT101CT::GetInstance(&huart1)->get_yaw_speed_rad();

    float ladpos_x = Lad_Data.x * cos(deg_to_rad(-90)) - Lad_Data.y * sin(deg_to_rad(-90));
    float ladpos_y = Lad_Data.x * sin(deg_to_rad(-90)) + Lad_Data.y * cos(deg_to_rad(-90));
	robot_pose_inWorld_.x = -ladpos_x + coordoffset.x_offset;
    robot_pose_inWorld_.y = -ladpos_y + coordoffset.y_offset;
    robot_pose_inWorld_.z = Lad_Data.z;
    robot_pose_inWorld_.yaw = yaw_from_position_;

    float ladvel_x = Lad_Data.line_x * cos(deg_to_rad(-90)) - Lad_Data.line_y * sin(deg_to_rad(-90));
    float ladvel_y = Lad_Data.line_x * sin(deg_to_rad(-90)) + Lad_Data.line_y * cos(deg_to_rad(-90));
    robot_speed_inworld_.x = ladvel_x;
    robot_speed_inworld_.y = ladvel_y;
    robot_speed_inworld_.z = Lad_Data.line_z;

    robot_speed_inworld_.yaw = dyaw_from_position_;

    if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin) == GPIO_PIN_SET)
    {
        swtich1_isOn = true;
    }
    else
    {
        swtich1_isOn = false;
    }

    if(HAL_GPIO_ReadPin(SWTICH2_GPIO_Port, SWTICH2_Pin) == GPIO_PIN_SET)
    {
        swtich2_isOn = true;
    }
    else
    {
        swtich2_isOn = false;
    }

}


void Locate_Setup::lader_transform_caculate()
{
    // 如果还没初始化安装位姿则直接返回
    if (!install_pose_init_) 
        return;

    // 机器人位姿（robot_pose_inWorld_.yaw 为角度，转换为弧度用于变换）
    Point3D robot_pose_rad = robot_pose_inWorld_;
    robot_pose_rad.yaw = deg_to_rad(robot_pose_inWorld_.yaw);

    // 构造 world -> robot 的变换（需要弧度）
    HomogeneousTransform3D T_world_to_robot(robot_pose_rad);

    // 构造 robot -> arm 的 3D 变换（由 arm_install_pose_ 提供 x,y,theta）
    Point3D arm_pose3d;
    arm_pose3d.x = arm_install_pose_.x;
    arm_pose3d.y = arm_install_pose_.y;
    arm_pose3d.z = 0.0f;
    arm_pose3d.roll = 0.0f;
    arm_pose3d.pitch = 0.0f;
    arm_pose3d.yaw = arm_install_pose_.theta; // 假定 arm_install_pose_.theta 为弧度

    T_robot_to_arm_3d.setTransform(arm_pose3d);

    // 组合得到 world -> arm
    HomogeneousTransform3D T_world_to_arm = T_world_to_robot.multiply(T_robot_to_arm_3d);

    // 计算臂基座在世界系的位置（3D）
    Point3D arm_base_world = T_world_to_arm.apply(Point3D{0,0,0,0,0,0});

    // 更新 2D 的臂基位姿（x,y,theta）。theta 以弧度存储
    arm_pose_inWorld_.x = arm_base_world.x;
    arm_pose_inWorld_.y = arm_base_world.y;
    arm_pose_inWorld_.theta = robot_pose_rad.yaw + arm_install_pose_.theta;

    // 更新 2D 变换（若需要在 2D 中使用）
    T_robot_to_arm_2d.setTransform(arm_install_pose_);
    T_lidar_to_robot_2d.setTransform(lidar_install_pose_);
}

void Locate_Setup::update_Lidar_data()
{
    Locate_Setup::Get_Rader_Data();
}


void Locate_Setup::Relocte_ToLader()
{
	Locate_Setup::USB_SendData();
}


//锟截讹拷位
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
	if(laser_mode==LEFT)
    {
        delta=fabs(laser_initData_.y1-laser_initData_.y2);
        robot_pose_inWorld_.yaw=atan(delta/laser_initData_.d);
        robot_pose_inWorld_.x=laser_initData_.x1*cos(robot_pose_inWorld_.yaw);
        robot_pose_inWorld_.y=0.5*(laser_initData_.y1+laser_initData_.y2)*cos(robot_pose_inWorld_.yaw);
        robot_pose_inWorld_.yaw=robot_pose_inWorld_.yaw*180/PI;
	
        if(laser_initData_.y1>laser_initData_.y2)
        {
            robot_pose_inWorld_.yaw=360-robot_pose_inWorld_.yaw;
            aaa=robot_pose_inWorld_.yaw;
        }
    }
	else if(laser_mode==RIGHT)
	{
        delta=fabs(laser_initData_.y1-laser_initData_.y2);
        robot_pose_inWorld_.yaw=atan(delta/laser_initData_.d);
        robot_pose_inWorld_.y=laser_initData_.x1*cos(robot_pose_inWorld_.yaw);
        robot_pose_inWorld_.x=0.5*(laser_initData_.y1+laser_initData_.y2)*cos(robot_pose_inWorld_.yaw);
        robot_pose_inWorld_.yaw=robot_pose_inWorld_.yaw*180/PI;
	
        if(laser_initData_.y1>laser_initData_.y2)
        {
            robot_pose_inWorld_.yaw=360-robot_pose_inWorld_.yaw;
            aaa=robot_pose_inWorld_.yaw;
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