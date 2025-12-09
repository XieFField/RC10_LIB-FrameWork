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
class Locate_Setup : public RtosTask {
public:
    Locate_Setup(Laser_InstanceManager* Laser_pos_instance):RtosTask("Locate_Setup", 1)
		{this->Laser_pos_instance=Laser_pos_instance;}
    ~Locate_Setup() = default;    
    /**
     * @brief 无输入则默认在底盘中心
     */
    void init(Point2D lidar_install_pose = {0}, Point2D arm_install_pose = {0})
    {   
        if(install_pose_init_)
            return;
        lidar_install_pose_ = lidar_install_pose;
        arm_install_pose_ = arm_install_pose;

        T_lidar_to_robot.setTransform(lidar_install_pose_);

        T_robot_to_arm.setTransform(arm_install_pose_);

        install_pose_init_ = true;
    }
		void RobotPos_inWorld_caculate(Laser_InstanceManager* Laser_pos_instance);
		
    Point2D get_ArmPos_inWorld(){return arm_pose_inWorld_;}

    Point2D get_RobotPos_inWorld(){return robot_pose_inWorld_;}

    Point2D get_LidarPos_inWorld(){return lidar_pose_inWorld_;}
		
		void locate_setup_init(){this->start(osPriorityNormal, 256);}
private:
	  
	  float x1;//规定激光实例管理的第一个为x的数据，第二三个为y的数据
    float y1;
    float y2;
    float d=0.25;
    float delta_x1;
    float delta_y1;
    float delta_y2;
    Laser_InstanceManager* Laser_pos_instance;
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