#include "Setup_ConfigInit.h"


DJI_MotorDemo dji_motor_demo;

void ALL_Setup_ConfigInit(void)
{
   dji_motor_demo.init();
   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
}

