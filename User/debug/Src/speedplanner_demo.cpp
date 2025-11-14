#include "speedplanner_demo.h"

#if Path
Vector2D start_point(-RealPosData.world_x, RealPosData.world_y);
Vector2D control_point(-6.0f, 16.0f);
Vector2D end_point(16.0f, 20.0f);
Vector2D speed(0.0f, 0.0f);
int num = 0;

Speedplanner_1D_Param_Config Param_1d{.maxAcc = 1.0f, .maxDec = 1.0f, .maxJerk = 1.5f, .maxSpeed = 2.0f, .initialSpeed = 0.001f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};
Path_Bezier path(start_point, control_point, end_point, Param_1d);
// Path_Bezier path(start_point, end_point, Param_1d);
// Vector2D point(1.0f, 10.0f);

//Path_Bezier path(end_point, control_point, start_point, Param_1d);
// Path_Bezier path(end_point,start_point,  Param_1d);
Vector2D real_point(0.0f, 0.0f);//实时去更新位置
Vector2D point(-RealPosData.world_x, RealPosData.world_y);
Vector2D error(0.0f, 0.0f);
Vector2D final_speed(0.0f, 0.0f);

#endif

#if Bezier_Curve
Vector2D start_point(0.0f, 0.0f);
Vector2D control_point(-6.0f, 16.0f);
Vector2D end_point(12.0f, 14.0f);
Vector2D point(0.01f, 0.01f);
Vector2D point_last(0.01f, 0.01f);
int num = 0;
float t = 0.0f;
BezierCurve bc(start_point, control_point, end_point);
float tp_pos_now = 0.0001f;
#endif

#if trapezoid_Velocitytype
int num = 0;
float speed_tar_temp = 0.0f;
float speed_tar = 10.0f;
float speed_now = 0.0f;
float pos_now = 0.0f;
Td td(10.0f);
// ????
ConstantAcc ca(0.02f); // ??????????????????
#endif

#if Positionaltype_1D
int num = 0;
float td_speed_now = 0.0f;
float td_pos_now = 0.0f;
float tp_speed_now = 0.0f;
float tp_pos_now = 0.0f;
Td td(10.0f);
// 1D??λ???
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
    const char *msg = "Hello UART2 on PB6/PB7\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    start(osPriorityNormal, 256);
    //pid_track.set_params(track_pid_params, 0.0f);
}

void SpeedPlanner_Demo::loop()
{

#if Path

    //从RealPosData获取机器人实际当前位置实时去更新
    real_point.x = -RealPosData.world_x;
    real_point.y = RealPosData.world_y;
    
    num++;
    if (path.isFinished() == false)
    {
        speed = path.plan(point);
        point = point + (speed * 0.001f);

        error = point-real_point;

        Vector2D correcction;
        correcction=error*0.01f; 

        final_speed = speed + correcction;
        
        if (num > 5)
        {
            debug_uart.printf_DMA("%f,%f,%f\n", point.x, point.y, speed.magnitude());
            num = 0;
        }
    }
    else if (num > 2000)
    {
        path.reset();
        point=start_point;
        num = 0;
    }

#endif

#if Bezier_Curve
    num++;
    if (num > 10)
    {
        if (t <= 1.0f)
        {
            t += 0.001f;
        }

        num = 0;
    }
    point = bc.Get_Point(t);
    tp_pos_now = bc.Get_Current_Len(t);
    debug_uart.printf_DMA("%f,%f,%f\n", point.x, point.y, tp_pos_now);

#endif

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
