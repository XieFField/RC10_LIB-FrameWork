/**
 * @file Locate_Setup.h
 * @brief 定位 主要是雷达接收 位姿变换 激光重定位等功能
 * @author XieFField    HaJiCao
 */
#ifndef LOCATE_SETUP_H
#define LOCATE_SETUP_H


#pragma once

#ifdef __cplusplus

extern "C" {
    #include <stdint.h>
  //  #include "semphr.h"
}

#include "RTOS_QueueSetup.h"
#include "BSP_USB_UART_Driver.h"
#include "APP_tool.h"
#include "APP_CoordConvert.h"
#include "BSP_TimeStamp.h"
#include "APP_debugTool.h"
#include "BSP_RTOS.h"
#include "Module_LaserPosition.h"
#include "math.h"
#include "Module_Position.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "gpio.h"

#include "Module_HWT.h"

#define PI		3.14159265358979323846f			// 定义圆周率常量PI
#define MAX_SEND_BUF_SIZE 128// 发送缓冲区大小
#define AUTO_FUNCTION 1 //0 表示采用侧吸方案的机械臂，1表示采用顶吸方案的机械臂
#define MAX_RECEIVE_BUF_SIZE 512// 接收缓冲区大小

#define MAX_RECEIVE_ID 10// 最大id

#define MAX_RECEIVE_DATA_LEN 64
	
		typedef enum LASER_MODE 
	{
    LEFT,
		RIGHT
	} LASER_MODE;
	
typedef struct 
{
  /* data */
    float x1;//规定激光实例管理的第一个为x的数据，第二三个为y的数据
    float y1;
    float y2;
    float d=0.25;
    float delta_x1;
    float delta_y1;
    float delta_y2;
}Laser_initData_S;

typedef struct 
{
  /* data */
    float x;//规定激光实例管理的第一个为x的数据，第二三个为y的数据
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float line_x;
    float line_y;
    float line_z;
}Lader_Data;
class Locate_Setup : public RtosTask {
public:
    static Locate_Setup* getInstance()
    {
        static Locate_Setup instance;
        return &instance;
    }

    ~Locate_Setup() = default;    
    /**
     * @brief 无输入则默认在底盘中心
     */
    void init(Laser_InstanceManager* instance_man,USB_CDC_ *usb_handle, Point2D lidar_install_pose = {0}, Point2D arm_install_pose = {0}, 
              Laser_initData_S laser_initData = {0}
            )
    {   
        this->Laser_pos_instance = instance_man;
		this->usb_handle=usb_handle;
        if(install_pose_init_)
            return;
        lidar_install_pose_ = lidar_install_pose;
        arm_install_pose_ = arm_install_pose;


        laser_initData_ = laser_initData;
				
				laser_initData_.d=0.5;
				laser_initData.delta_x1=0.3;

        laser_initData_ = laser_initData;

        install_pose_init_ = true;
    }

    //激光重定位接算
    void RobotPos_inWorld_caculate(Laser_InstanceManager* Laser_pos_instance);
		
    void register_laserManager(Laser_InstanceManager* Laser_pos_instance)
    {
        this->Laser_pos_instance = Laser_pos_instance;
    }

    /**
     * @brief 设置是否启动激光重定位
     */
    void set_startToLRL(bool is_startToLRL)
    {
        this->is_startToLRL_ = is_startToLRL;
    }

    Point2D get_ArmPos_inWorld(){return arm_pose_inWorld_;}

    Point3D get_RobotPos_inWorld(){return robot_pose_inWorld_;}
    Point3D get_RobotSpeed_inWorld(){return robot_speed_inworld_;}
//    Point3D get_LidarPos_inWorld(){return lidar_pose_inWorld_;}
	


    Point2D get_FK_ChassisSpeed_inWorld(){return fk_chassisSpeed_inWorld_;}

	void locate_setup_init(){this->start(osPriorityNormal, 256);}
		
    float get_yaw_from_position(){return yaw_from_position_;}
    float get_dyaw_from_position(){return dyaw_from_position_;}

	Laser_initData_S laser_initData_;

    void Relocte_ToLader();

    /**
     * @brief 获取底盘微动开关状态
     */
    bool ifSwitch1On(){return swtich1_isOn;}
    bool ifSwitch2On(){return swtich2_isOn;}

private:
    void Get_Rader_Data();
    void USB_SendData();
	Locate_Setup():RtosTask("Locate_Setup", 2), Laser_pos_instance(nullptr) {}
	  
    Laser_InstanceManager* Laser_pos_instance;

    Lader_Data Lad_Data={0};
    USB_CDC_ *usb_handle;
    LASER_MODE laser_mode=LEFT;//默认起始位置在左
    bool is_startToLRL_ = false; // 是否启动激光重定位
    void update(); //更新



    /**
     * @brief 雷达坐标变换计算->robot_in_world, arm_in_world
     */
    void lader_transform_caculate(); //雷达坐标变换计算
    void update_Lidar_data();        //更新雷达数据

    Point2D lidar_install_pose_ = {0}; // 雷达安装相对底盘中心
    Point2D arm_install_pose_ = {0};   // 机械臂安装相对底盘中心

    Point3D robot_pose_inWorld_ = {0}; // 机器人在世界坐标系位置
    Point3D robot_speed_inworld_ = {0}; // 机器人在世界坐标系速度



    Point2D arm_pose_inWorld_ = {0};   // 机械臂底座在世界坐标系位置


    // Point3D lidar_pose_inWorld_ = {0}; // 雷达在世界坐标系位置
    

    Point2D fk_chassisSpeed_inWorld_ = {0}; // 正解底盘速度 世界坐标系

    float yaw_from_position_ = 0.0f; // 从里程计position计算得到的偏航角
    float dyaw_from_position_ = 0.0f;

    // 2D/3D 坐标转换矩阵
    HomogeneousTransform2D T_lidar_to_robot_2d; // 雷达 -> 机器人 (2D)
    HomogeneousTransform2D T_robot_to_arm_2d;  // 机器人 -> 机械臂 (2D)
    HomogeneousTransform3D T_robot_to_arm_3d;  // 机器人 -> 机械臂 (3D)

    bool install_pose_init_ = false;

    bool swtich1_isOn = false;
    bool swtich2_isOn = false;

    // struct {
    //     float x_offset = 0.48f;
    //     float y_offset = 0.50f;
    // }coordoffset;
    struct {
        float x_offset = 0.0f;
        float y_offset = 0.0f;
    }coordoffset;
protected:
    void loop() override;
};
uint8_t xor_check(const uint8_t *data, uint32_t length);


#endif


#endif