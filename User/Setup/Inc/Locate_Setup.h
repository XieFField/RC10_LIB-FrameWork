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

class Locate_Setup : public UART_, public RtosTask {
public:
    Locate_Setup():RtosTask("Locate_Setup", 1){}
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



    Point2D get_ArmPos_inWorld(){return arm_pose_inWorld_;}

    Point2D get_RobotPos_inWorld(){return robot_pose_inWorld_;}

    Point2D get_LidarPos_inWorld(){return lidar_pose_inWorld_;}
private:

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

#endif


#endif