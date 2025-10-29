#include "speedplanner_demo.h"

#if trapezoid_Velocitytype
int num = 0;
float speed_tar_temp = 0.0f;
float speed_tar = 10.0f;
float speed_now = 0.0f;
float pos_now = 0.0f;
Td td(10.0f);
// 速度式
ConstantAcc ca(0.02f); // 注意代码运行系统的周期
#endif

#if Positionaltype_1D
int num = 0;
float td_speed_now = 0.0f;
float td_pos_now = 0.0f;
float tp_speed_now = 0.0f;
float tp_pos_now = 0.0f;
Td td(10.0f);
// 1D的位置式
Speedplanner_1D_Param_Config Param_1d{.maxAcc = 2.9f, .maxDec = 2.0f, .maxJerk = 1.5f, .maxSpeed = 6.0f, .initialSpeed = 0.0f, .finalSpeed = 0.0f, .startPos = 0.0000000001f, .targetPos = 30.0f, .deadzone = 0.00001f};
// TrapePlanner1D TP_1d(Param_1d);
SShapedPlanner1D TP_1d(Param_1d);
#endif

#if Positionaltype_2D
int num = 0;
Vector2D tp_pos_now(0.0f, 0.0f);
Vector2D tp_speed_now(0.0f, 0.0f);
Speedplanner_2D_Param_Config Param_2d{.maxAcc = 6.0f, .maxDec = 8.0f, .maxJerk = 5.0f, .maxSpeed = 6.0f, .initialSpeed = 0.0000001f, .finalSpeed = 0.0f, .startPos.x = 0.0f, .startPos.y = 0.0f, .targetPos.x = 12.0f, .targetPos.y = 16.0f, .deadzone = 0.0001f};
// SShapedPlanner2D TP_2d(Param_2d);
TrapePlanner2D TP_2d(Param_2d);
#endif
void SpeedPlanner_Demo::init()
{
    const char *msg = "Hello UART1 on PB6/PB7\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    start(osPriorityNormal, 256);
}

void SpeedPlanner_Demo::loop()
{
    // 任务循环

#if trapezoid_Velocitytype

    num++;
    speed_tar_temp = ca.plan(speed_tar);
    speed_now = td.plan(speed_tar_temp);

    if (num > 2000)
    {
        speed_tar *= (-1.0f);
        num = 0;
    }

    debug_uart.printf_DMA("%f,%f\n", speed_now, speed_tar_temp);

#endif

#if Positionaltype_1D
    num++;

    tp_speed_now = TP_1d.plan(tp_pos_now);
    tp_pos_now += tp_speed_now * 0.001f;

    td_speed_now = td.plan(tp_speed_now);
    td_pos_now += td_speed_now * 0.001f;

    if (num > 10)
    {
        debug_uart.printf_DMA("%f,%f,%f,%f\n", tp_speed_now, td_speed_now, tp_pos_now, td_pos_now);
        num = 0;
    }

#endif

#if Positionaltype_2D
    num++;

    tp_speed_now = TP_2d.plan(tp_pos_now);
    tp_pos_now = tp_pos_now + (tp_speed_now * 0.001f);

    if (num > 2)
    {
        debug_uart.printf_DMA("%f,%f\n", tp_speed_now.magnitude(), tp_pos_now.magnitude());
        num = 0;
    }

#endif
}
