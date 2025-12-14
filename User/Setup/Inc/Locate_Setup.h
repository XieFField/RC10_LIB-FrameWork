/**
 * @file Locate_Setup.h
 * @brief 定位 主要是雷达接收 位姿变换 激光重定位等功能
 * @author XieFField
 */
#ifndef LOCATE_SETUP_H
#define LOCATE_SETUP_H

#pragma once

#ifdef __cplusplus

extern "C" {
    #include <stdint.h>
}

#include "BSP_USB_UART_Driver.h"
#include "APP_tool.h"
#include "APP_CoordConvert.h"
#include "BSP_TimeStamp.h"
#include "APP_debugTool.h"
#include "BSP_RTOS.h"
#include "Module_LaserPosition.h"
#include "math.h"
#define PI							3.14159265358979323846f			// 定义圆周率常量PI
#define MAX_SEND_BUF_SIZE 128// 发送缓冲区大小

#define MAX_RECEIVE_BUF_SIZE 512// 接收缓冲区大小

#define MAX_RECEIVE_ID 10// 最大id

#define MAX_RECEIVE_DATA_LEN 64
	typedef enum RECEIVE_FLAG
	{
		WAIT_HEAD_1,// 0xaa
		WAIT_HEAD_2,// 0x55
		WAIT_ID,// 1~max
		WAIT_LEN,// 1~64
		WAIT_DATA,
		WAIT_CHECK,//xor
		WAIT_TAIL// 0xee
	} RECEIVE_FLAG;
	
	
	
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
    float yaw;
}Lader_Data;

class Locate_Setup : public RtosTask {
public:
    Locate_Setup(Laser_InstanceManager* instance_man = nullptr):RtosTask("Locate_Setup", 1), Laser_pos_instance(instance_man)
		{}
    ~Locate_Setup() = default;    
    /**
     * @brief 无输入则默认在底盘中心
     */
    void init(Point2D lidar_install_pose = {0}, Point2D arm_install_pose = {0}, 
              Laser_initData_S laser_initData = {0})
    {   
        if(install_pose_init_)
            return;
        lidar_install_pose_ = lidar_install_pose;
        arm_install_pose_ = arm_install_pose;

//        T_lidar_to_robot.setTransform(lidar_install_pose_);

//        T_robot_to_arm.setTransform(arm_install_pose_);

        laser_initData_ = laser_initData;
				
				laser_initData_.d=0.5;
				laser_initData.delta_x1=0.3;

        install_pose_init_ = true;
    }
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

    Point2D get_RobotPos_inWorld(){return robot_pose_inWorld_;}

    Point2D get_LidarPos_inWorld(){return lidar_pose_inWorld_;}
		
		void locate_setup_init(){this->start(osPriorityNormal, 256);}
		
		
		Laser_initData_S laser_initData_;
private:
	
	  
    Laser_InstanceManager* Laser_pos_instance;
    bool is_startToLRL_ = false; // 是否启动激光重定位
    void update(); //更新
    
    Point2D update_Lidar_data(); //更新雷达数据

    Point2D lidar_install_pose_ = {0}; // 雷达安装相对底盘中心
    Point2D arm_install_pose_ = {0};   // 机械臂安装相对底盘中心

    Point2D robot_pose_inWorld_ = {0}; // 机器人在世界坐标系位置
    Point2D arm_pose_inWorld_ = {0};   // 机械臂在世界坐标系位置
    Point2D lidar_pose_inWorld_ = {0}; // 雷达在世界坐标系位置

    static HomogeneousTransform2D T_lidar_to_robot;   // 雷达 -> 机器人
    static HomogeneousTransform2D T_robot_to_arm;     // 机器人 -> 机械臂

    bool install_pose_init_ = false;

protected:
    void loop() override;
};


class Lader_position:public USB_CDC_{
public:
    // ??????
    static Lader_position* GetInstance(USBD_HandleTypeDef *usb_handle);
    // ???UART
    void InitUART();
		//?????.cpp
    void Callback_DCD_Fuc(uint8_t *buf, uint16_t len) override;
    // ??????????????
    Lader_position(const Lader_position&) = delete;
    Lader_position& operator=(const Lader_position&) = delete;

    void Reposition_SendData(float X, float Y);

 //   Lader_position getRealPosData() const { return RealPosData; }
    Lader_Data Get_Rader_Data(){return Lad_Data;}

//接收数据
		RECEIVE_FLAG receive_flag = WAIT_HEAD_1;
		uint8_t receive_id;
		uint8_t receive_len;
		uint8_t receive_check;
		uint8_t receive_data[MAX_RECEIVE_DATA_LEN] = {0};
		uint16_t receive_data_dx = 0;
    
private:
	  Lader_Data Lad_Data={0};
    Lader_position(USBD_HandleTypeDef *usb_handle); // ??????
    ~Lader_position() = default;
    
    void Update_RawPosition(float value[5]);

    // UART??
    UART_* uart_instance_;   
    // ?????
    bool uart_initialized_;

		uint8_t rx_buffer_[35];
};
uint8_t xor_check(const uint8_t *data, uint32_t length);
//class LaerRelocate_Manager : public RtosTask {
//public:
//    LaerRelocate_Manager(LaserPosition laser_module1, LaserPosition laser_module2, LaserPosition laser_module3)
//        :RtosTask("LaerRelocate_Manager", 1)
//    {

//    }


//    ~LaerRelocate_Manager() = default;


//    Point2D get_RobotPos_inWorld(){}

//private:
//    LaserPosition* laser_position_module_[3];

//}


#endif


#endif