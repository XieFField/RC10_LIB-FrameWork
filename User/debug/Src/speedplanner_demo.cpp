#include "speedplanner_demo.h"

#if Path_end
Speedplanner_1D_Param_Config params{.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 1.5f, .maxSpeed = 1.0f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.000f, .targetPos = 0.0f, .deadzone = 0.0001f};
Path path(params);
Path path1(params);
Vector2D speed(0.0f, 0.0f);
Vector2D speed1(0.0f, 0.0f);
// Vector2D point_start(1.0f, 1.0f);
Vector2D point_start(1.2f, 0.0f);

Vector2D point = point_start;
Vector2D point1 = point_start;

int num = 0;

Td td(10.0f);
Td td1(10.0f);
Td td2(10.0f);
Td td12(10.0f);

// int index = 0;
//  float t = 0.0f;
//  float total_ = 0.0f;
//  float distance_=0.0f;
//  float t_ = 0.0f;                             // 贝塞尔曲线参数 t
//  float v_resultant_ = 0.0f;                   // 当前速度
//  Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
//  Vector2D point_last_ = Vector2D(1.2f,0.0f);; // 上一个点
//  SPhase m_phase = S_FINISHED_PHASE;           // 当前规划所处的阶段
//  SShapedPlanner1D sp_(params);

#endif

#if Path_s

Vector2D start_point(0.0f, 0.0f);
Vector2D control_point(0.0f, 16.0f);
Vector2D end_point(16.0f, 16.0f);

Vector2D speed(0.0f, 0.0f);
int num = 0;

Speedplanner_1D_Param_Config Param_1d{.maxAcc = 1.0f, .maxDec = 1.0f, .maxJerk = 1.5f, .maxSpeed = 3.0f, .initialSpeed = 0.1f, .finalSpeed = 3.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f};

Path_Bezier path(start_point, control_point, end_point, Param_1d);
Path_Bezier path1(start_point, control_point, end_point, Param_1d);
// Path_Bezier path(start_point, end_point, Param_1d);

Vector2D point = start_point;
Vector2D point1 = start_point;

Td td(5.0f);
Td td1(5.0f);
Td td2(5.0f);
Td td12(5.0f);
// Path_Bezier path(end_point, control_point, start_point, Param_1d);
// Path_Bezier path(end_point,start_point,Param_1d);
// Vector2D point=end_point;
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
Speedplanner_1D_Param_Config Param_1d{.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 1.5f, .maxSpeed = 1.0f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 19.66666f, .deadzone = 0.001f};
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
#if Path_end

    // 两种用法：
    // 1.控制点给值设为1 （当然也可以设为0到1之间的数，但是会取控制点前后两点之间的十分位点作为贝塞尔曲线的起点和终点）
    // 2.全部点都设为0.5除了起点和终点，但是会取控制点前后两点之间的中点作为贝塞尔曲线的起点和终点（当然也可以取别的值效果同理，但是千万不能取大于0.5的值在此用法下）

    path.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
    path.Add_Point(Vector2D{0, 0}, 0.8f);
    path.Add_Point(Vector2D{0, 1.2}, 0);
    path.Add_Point(Vector2D{0, 6}, 0);
    path.Add_Point(Vector2D{0, 7.2}, 0.8f);
    path.Add_Point(Vector2D{1.2, 7.2}, 0);
    path.Add_End_Point(Vector2D{4.8, 7.2}, 0);

    path1.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
    path1.Add_Point(Vector2D{0, 0}, 0.8f);
    path1.Add_Point(Vector2D{0, 1.2}, 0);
    path1.Add_Point(Vector2D{0, 6}, 0);
    path1.Add_Point(Vector2D{0, 7.2}, 0.8f);
    path1.Add_Point(Vector2D{1.2, 7.2}, 0);
    path1.Add_End_Point(Vector2D{4.8, 7.2}, 0);

/////////////////////////////////////////////////////////////////////////////////////////

//          path.Add_Start_Point(Vector2D{1.2,0},0,0);
//          path.Add_Point(Vector2D{0,0},1.0f);
//          path.Add_Point(Vector2D{0,1.2},0);
//          path.Add_Point(Vector2D{0,6},0);
//          path.Add_Point(Vector2D{0,7.2},1.0f);
//          path.Add_Point(Vector2D{1.2,7.2},0);
//          path.Add_Point(Vector2D{4.8,7.2},0);
//          path.Add_Point(Vector2D{6,7.2},1.0f);
//          path.Add_Point(Vector2D{6,6},0);
//          path.Add_Point(Vector2D{6,1.2},0);
//          path.Add_Point(Vector2D{6,0},1.0f);
//          path.Add_Point(Vector2D{4.8,0},0);
//          path.Add_End_Point(Vector2D{1.2,0},0);
//
//          path1.Add_Start_Point(Vector2D{1.2,0},0,0);
//          path1.Add_Point(Vector2D{0,0},1.0f);
//          path1.Add_Point(Vector2D{0,1.2},0);
//          path1.Add_Point(Vector2D{0,6},0);
//          path1.Add_Point(Vector2D{0,7.2},1.0f);
//          path1.Add_Point(Vector2D{1.2,7.2},0);
//          path1.Add_Point(Vector2D{4.8,7.2},0);
//          path1.Add_Point(Vector2D{6,7.2},1.0f);
//          path1.Add_Point(Vector2D{6,6},0);
//          path1.Add_Point(Vector2D{6,1.2},0);
//          path1.Add_Point(Vector2D{6,0},1.0f);
//          path1.Add_Point(Vector2D{4.8,0},0);
//          path1.Add_End_Point(Vector2D{1.2,0},0);

///////////////////////////////////////////////////////////////////////

//        path.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path.Add_Point(Vector2D{0, 0}, 0.5f);
//        path.Add_Point(Vector2D{0, 1.2}, 0.5f);
//        path.Add_Point(Vector2D{0, 6}, 0.5f);
//        path.Add_Point(Vector2D{0, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{1.2, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{4.8, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{6, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{6, 6}, 0.5f);
//        path.Add_Point(Vector2D{6, 1.2}, 0.5f);
//        path.Add_Point(Vector2D{6, 0}, 0.5f);
//        path.Add_Point(Vector2D{4.8, 0}, 0.5f);
//        path.Add_End_Point(Vector2D{1.2, 0}, 0);
//
//        path1.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path1.Add_Point(Vector2D{0, 0}, 0.5f);
//        path1.Add_Point(Vector2D{0, 1.2}, 0.5f);
//        path1.Add_Point(Vector2D{0, 6}, 0.5f);
//        path1.Add_Point(Vector2D{0, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{1.2, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{4.8, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{6, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{6, 6}, 0.5f);
//        path1.Add_Point(Vector2D{6, 1.2}, 0.5f);
//        path1.Add_Point(Vector2D{6, 0}, 0.5f);
//        path1.Add_Point(Vector2D{4.8, 0}, 0.5f);
//        path1.Add_End_Point(Vector2D{1.2, 0}, 0);

////////////////////////////////////////////////////////////////////

//        path.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path.Add_Point(Vector2D{0, 0}, 0.5f);
//        path.Add_Point(Vector2D{0, 1.2}, 0.5f);
//        path.Add_Point(Vector2D{0, 6}, 0.5f);
//        path.Add_Point(Vector2D{0, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{1.2, 7.2}, 0.5f);
//        path.Add_Point(Vector2D{4.8, 7.2}, 0.5f);
//        path.Add_End_Point(Vector2D{6, 7.2}, 0);
//
//        path1.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path1.Add_Point(Vector2D{0, 0}, 0.5f);
//        path1.Add_Point(Vector2D{0, 1.2}, 0.5f);
//        path1.Add_Point(Vector2D{0, 6}, 0.5f);
//        path1.Add_Point(Vector2D{0, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{1.2, 7.2}, 0.5f);
//        path1.Add_Point(Vector2D{4.8, 7.2}, 0.5f);
//        path1.Add_End_Point(Vector2D{6, 7.2}, 0);

////////////////////////////////////////////////////////////////////

//        path.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path.Add_Point(Vector2D{0, 0}, 0.3f);
//        path.Add_Point(Vector2D{0, 1.2}, 0.3f);
//        path.Add_Point(Vector2D{0, 6}, 0.3f);
//        path.Add_Point(Vector2D{0, 7.2}, 0.3f);
//        path.Add_Point(Vector2D{1.2, 7.2}, 0.3f);
//        path.Add_Point(Vector2D{4.8, 7.2}, 0.3f);
//        path.Add_End_Point(Vector2D{6, 7.2}, 0);
//
//        path1.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path1.Add_Point(Vector2D{0, 0}, 0.3f);
//        path1.Add_Point(Vector2D{0, 1.2}, 0.3f);
//        path1.Add_Point(Vector2D{0, 6}, 0.3f);
//        path1.Add_Point(Vector2D{0, 7.2}, 0.3f);
//        path1.Add_Point(Vector2D{1.2, 7.2}, 0.3f);
//        path1.Add_Point(Vector2D{4.8, 7.2}, 0.3f);
//        path1.Add_End_Point(Vector2D{6, 7.2}, 0);
///////////////////////////////////////////////////////////////////////////////////////////

//        path.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path.Add_Point(Vector2D{0, 0}, 0.8f);
//        path.Add_Point(Vector2D{0, 1.2}, 0.8f);
//        path.Add_Point(Vector2D{0, 6}, 0.8f);
//        path.Add_Point(Vector2D{0, 7.2}, 0.8f);
//        path.Add_Point(Vector2D{1.2, 7.2}, 0.8f);
//        path.Add_Point(Vector2D{4.8, 7.2}, 0.8f);
//        path.Add_End_Point(Vector2D{6, 7.2}, 0);
//
//        path1.Add_Start_Point(Vector2D{1.2, 0}, 0, 0);
//        path1.Add_Point(Vector2D{0, 0}, 0.8f);
//        path1.Add_Point(Vector2D{0, 1.2}, 0.8f);
//        path1.Add_Point(Vector2D{0, 6}, 0.8f);
//        path1.Add_Point(Vector2D{0, 7.2}, 0.8f);
//        path1.Add_Point(Vector2D{1.2, 7.2}, 0.8f);
//        path1.Add_Point(Vector2D{4.8, 7.2}, 0.8f);
//        path1.Add_End_Point(Vector2D{6, 7.2}, 0);

/////////////////////////////////////////////////////////////////

//    path.Add_Start_Point(Vector2D{1, 1}, 0, 0);
//    path.Add_Point(Vector2D{5, 5}, 0.5f);
//    path.Add_Point(Vector2D{6, 6}, 0.5f);
//    path.Add_Point(Vector2D{6, 7}, 0.5f);
//    path.Add_Point(Vector2D{6, 15}, 0.5f);
//    path.Add_Point(Vector2D{6, 16}, 0.5f);
//    path.Add_Point(Vector2D{5, 15}, 0.5f);
//    path.Add_Point(Vector2D{1, 11}, 0.5f);
//    path.Add_Point(Vector2D{0, 10}, 0.5f);
//    path.Add_Point(Vector2D{0, 9}, 0.5f);
//    path.Add_Point(Vector2D{0, 1}, 0.5f);
//    path.Add_Point(Vector2D{0, 0}, 0.5f);
//    path.Add_End_Point(Vector2D{1, 1}, 0);
//
//
//    path1.Add_Start_Point(Vector2D{1, 1}, 0, 0);
//    path1.Add_Point(Vector2D{5, 5}, 0.5f);
//    path1.Add_Point(Vector2D{6, 6}, 0.5f);
//    path1.Add_Point(Vector2D{6, 7}, 0.5f);
//    path1.Add_Point(Vector2D{6, 15}, 0.5f);
//    path1.Add_Point(Vector2D{6, 16}, 0.5f);
//    path1.Add_Point(Vector2D{5, 15}, 0.5f);
//    path1.Add_Point(Vector2D{1, 11}, 0.5f);
//    path1.Add_Point(Vector2D{0, 10}, 0.5f);
//    path1.Add_Point(Vector2D{0, 9}, 0.5f);
//    path1.Add_Point(Vector2D{0, 1}, 0.5f);
//    path1.Add_Point(Vector2D{0, 0}, 0.5f);
//    path1.Add_End_Point(Vector2D{1, 1}, 0);

////////////////////////////////////////////////////////////
#endif
#if Path_s

#endif
    pid_track.set_params(track_pid_params, 0.0f);
    start(osPriorityNormal, 256);
}

void SpeedPlanner_Demo::loop()
{
#if Path_end

    //    if (path.Is_End() == true)
    //    {

    //        point = path.bezier_curve_list[index].Get_Point(t);
    //        t += 0.001f;
    //        if (t > 1.0f)
    //        {
    //            t = 0.0f;
    //            index++;
    //            if (index >= path.bezier_curve_num)
    //            {
    //                index = 0.0f;
    //            }
    //        }
    //    }

    //    num++;
    //    if (path.Is_End() == true)
    //    {
    //        speed = path.plan(point);
    //        point = point + speed * 0.001f; // 返回速度向量
    //    }
    //    else
    //    {
    //        path.plan_reset();
    //        speed={0.0f, 0.0f};
    //        point=point_start;
    //    }

    //    if (num > 2)
    //    {
    //        // debug_uart.printf_DMA("%f,%f\n", point.x, point.y);
    //        debug_uart.printf_DMA("%f,%f,%f\n", point.x, point.y, speed.magnitude());
    //        num = 0;
    //    }

    num++;
    if (path1.Is_End() == true)
    {
        Vector2D speed_temp = path1.plan(point);
        speed1.y = td12.plan(speed_temp.y);
        speed1.x = td2.plan(speed_temp.x);
        point1 = point1 + (speed1 * 0.001f);
    }

    pathEnd = path.get_bezier_curve().Get_Point(1.0f);
    // 新增：机器人到几何终点的真实距离
    float distToEnd = (point - pathEnd).magnitude();

    if (path.Is_End() == true)
    {
        // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest
        nearestPt = GetPathNearestPoint(path.get_bezier_curve(), point, tNearest);
        // 2. 找前视点+前进方向：根据最近点和前视距离，寻找前视点及其参数 tLookahead
        lookaheadPt = FindLookaheadPoint(path.get_bezier_curve(), tNearest, tLookahead);
        lookaheadTangent = path.get_bezier_curve().Get_Tangent_Vector(tLookahead);
        // 3. 计算横向偏差：计算机器人当前位置到路径切线的垂直距离
        lateralError = CalculateLateralError(path.get_bezier_curve(), point, nearestPt, tLookahead);
        // 4. 横向偏差PID控制：计算横向纠偏速度大小
        correctspeed = pid_track.pid_calc(0.0f, lateralError);
        Vector2D corrDir(-lookaheadTangent.y, lookaheadTangent.x); // 纠偏方向（垂直前进方向，左右纠偏）
        corrVelocity = corrDir * correctspeed;                     // 合成纠偏速度（方向+大小）
        // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
        planspeed = path.plan(point);
        // baseVelocity = lookaheadTangent * planspeed.magnitude();
        Vector2D speed_temp = planspeed + corrVelocity; // 最终速度 = 规划的前进速度 + 横向纠偏速度
        // 6. 速度平滑处理
        speed.y = td1.plan(speed_temp.y);
        speed.x = td.plan(speed_temp.x);
        // 7. 更新模拟位置（实际应用中这一步由物理运动代替）
        point = point + (speed * 0.001f);
        if (num > 5)
        {
            debug_uart.printf_DMA("%f,%f,%f,%f,%f\n", point.x, point.y, speed_temp.magnitude(), point1.x, point1.y);
            num = 0;
        }
    }
    else
    {
        // 路径运行结束后的重置逻辑
        if (num > 2000)
        {
            path.plan_reset();
            td.reset();
            td1.reset();
            point = point_start;
            num = 0;

            path1.plan_reset();
            td2.reset();
            td12.reset();
            point1 = point_start;
            speed = {0.0f, 0.0f};
        }
    }
#endif

#if Path_s

    num++;
    if (path1.isFinished() == false)
    {
        Vector2D speed_temp = path1.plan(point);
        speed.y = td12.plan(speed_temp.y);
        speed.x = td2.plan(speed_temp.x);
        point1 = point1 + (speed * 0.001f);
        //           if (num > 5)
        //           {
        //               debug_uart.printf_DMA("%f,%f,%f\n", point.x, point.y, speed.magnitude());
        //               num = 0;
        //           }
    }

    pathEnd = path.get_bezier_curve().Get_Point(1.0f);
    // 新增：机器人到几何终点的真实距离
    float distToEnd = (point - pathEnd).magnitude();

    if (path.isFinished() == false)
    {
        // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest
        nearestPt = GetPathNearestPoint(path.get_bezier_curve(), point, tNearest);
        // 2. 找前视点+前进方向：根据最近点和前视距离，寻找前视点及其参数 tLookahead
        lookaheadPt = FindLookaheadPoint(path.get_bezier_curve(), tNearest, tLookahead);
        lookaheadTangent = path.get_bezier_curve().Get_Tangent_Vector(tLookahead);
        // 3. 计算横向偏差：计算机器人当前位置到路径切线的垂直距离
        lateralError = CalculateLateralError(path.get_bezier_curve(), point, nearestPt, tLookahead);
        // 4. 横向偏差PID控制：计算横向纠偏速度大小
        correctspeed = pid_track.pid_calc(0.0f, lateralError);
        Vector2D corrDir(-lookaheadTangent.y, lookaheadTangent.x); // 纠偏方向（垂直前进方向，左右纠偏）
        corrVelocity = corrDir * correctspeed;                     // 合成纠偏速度（方向+大小）
        // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
        planspeed = path.plan(point);
        // baseVelocity = lookaheadTangent * planspeed.magnitude();
        Vector2D speed_temp = planspeed + corrVelocity; // 最终速度 = 规划的前进速度 + 横向纠偏速度
        // 6. 速度平滑处理
        speed.y = td1.plan(speed_temp.y);
        speed.x = td.plan(speed_temp.x);
        // 7. 更新模拟位置
        point = point + (speed * 0.001f);
        if (num > 7)
        {
            debug_uart.printf_DMA("%f,%f,%f,%f,%f\n", point.x, point.y, speed.magnitude(), point1.x, point1.y);
            num = 0;
        }
    }
    else
    {
        // 路径运行结束后的重置逻辑
        if (num > 2000)
        {
            path.reset();
            td.reset();
            td1.reset();
            point = start_point;
            num = 0;

            path1.reset();
            td2.reset();
            td12.reset();
            point1 = start_point;
            speed = {0.0f, 0.0f};
        }
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
    if (TP_1d.isFinished() == true)
    {
        if (num > 2000)
        {
            tp_pos_now = 0.0f;
            num = 0;
        }
    }
    else
    {
        if (num > 5)
        {
            // debug_uart.printf_DMA("%f,%f,%f,%f\n", tp_speed_now, tp_pos_now, td_speed_now, td_pos_now);
            debug_uart.printf_DMA("%f,%f,%f,%f\n", tp_speed_now, tp_pos_now);
            num = 0;
        }
    }

    //    td_speed_now = td.plan(tp_speed_now);
    //    td_pos_now += td_speed_now * 0.001f;

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

/**
 * @brief 整合已有接口，获取“最近点坐标”和“对应的t值”
 * @param robotPos 输入：机器人当前实际位置（闭环核心输入）
 * @param tNearest 输出：最近点对应的曲线参数t（0~1），给后续找前视点用
 * @return Vector2D 输出：最近点的坐标（给后续算横向偏差用）
 */
Vector2D SpeedPlanner_Demo::GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest)
{
    // 第一步：调用你的Get_Nearest_Distance，拿到tNearest（最近点对应的t值）
    // 重点：第二个参数传 &tNearest（tNearest的地址），因为你的函数是“输出参数”（通过指针赋值）
    path_.Get_Nearest_Distance(robotPos, &tNearest);

    // 第二步：用第一步拿到的tNearest，调用你的Get_Point，拿到最近点坐标
    Vector2D nearestPt = path_.Get_Point(tNearest);

    // 第三步：返回最近点坐标，给后续“算横向偏差”用
    return nearestPt;
}

// 函数作用：输入最近点的编号tNearest，输出前视点坐标和它的编号tLookahead
Vector2D SpeedPlanner_Demo::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
{
    // -------------- 对应第1步：初始化，从最近点开始 --------------
    tLookahead = tNearest;        // 前视点的编号，先从最近点的编号开始（比如t=0.3）
    float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
    float step = 0.01f;           // 每次挪的“小步子”

    // 拿到最近点的坐标（比如(5.2, 6.1)），作为“挪步”的起点
    Vector2D lastPt = path_.Get_Point(tLookahead);

    // -------------- 对应第2步：小步慢挪，直到累计距离够前视距离 --------------
    // 条件：1. 编号t没到终点（<1.0）；2. 累计距离还没到前视距离（<0.4m）
    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
    {
        // 1. 往前挪一小步：t增加0.005（比如0.3→0.305）
        float nextT = tLookahead + step;
        // 防止挪超终点：如果nextT>1.0，就改成1.0（不能超出曲线）
        if (nextT > 1.0f)
        {
            nextT = 1.0f;
        }

        // 2. 拿到这一步挪到的点的坐标（比如t=0.305对应的曲线点(5.22, 6.11)）
        Vector2D nextPt = path_.Get_Point(nextT);

        // 3. 计算这一步走了多远（比如从(5.2,6.1)到(5.22,6.11)，距离≈0.022m）
        float distStep = (nextPt - lastPt).magnitude();

        // 4. 累计距离：把这一步的距离加进去（比如0+0.022=0.022m）
        accumulatedDist += distStep;

        // 5. 更新：准备下一步挪步（把当前点当起点，当前t当下一步的基础）
        tLookahead = nextT; // 编号更新
        lastPt = nextPt;    // 起点更新为(5.22,6.11)
    }

    // -------------- 对应第3步：如果到终点了，直接用终点当前视点 --------------
    if (tLookahead >= 1.0f)
    {
        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
    }

    // -------------- 返回前视点坐标 --------------
    return lastPt;
}

/**
 * @brief 计算横向偏差（带方向：正=偏左，负=偏右，单位：m）
 * @param robotPos 机器人当前位置（主函数的m_robotPos）
 * @param nearestPt 曲线最近点（主函数的nearestPt，即P(t')）
 * @param tLookahead 前视点t值（主函数的tLookahead，用于获取稳定切向量）
 * @return float 纯横向偏差（无前后干扰，直接给PID用）
 */
float SpeedPlanner_Demo::CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead)
{
    // 步骤1：计算原始偏差向量 Δp = 机器人位置 - 最近点（你的定义：Δp = p - p(t')）
    Vector2D delta_p = robotPos - nearestPt;

    // 步骤2：获取前视点的切向量（和主函数一致，确保前进方向基准统一）
    // 主函数里已经调用过一次，但这里再调用一次，保证偏差计算和前进方向完全同步
    lookaheadTangent = path_.Get_Tangent_Vector(tLookahead);

    // 步骤4：定义“横向方向”：垂直于前视点切向量（左转90度，和主函数corrDir方向一致）
    // 主函数纠偏方向是 corrDir = (-lookaheadTangent.y, lookaheadTangent.x)，这里横向方向和它保持一致
    Vector2D lateral_dir = Vector2D(-lookaheadTangent.y, lookaheadTangent.x);
    // 横向方向也归一化：确保点积计算的偏差单位是“米”（无缩放干扰）
    lateral_dir.normalize();

    // 步骤5：核心：计算原始偏差Δp在“横向方向”的投影 → 纯横向偏差
    // 点积公式：delta_p · lateral_dir = |delta_p| * cosθ（θ是Δp和横向方向的夹角）
    // 作用：过滤前后方向干扰（前后方向与横向垂直，cos90°=0），只留左右偏差
    float lateral_err = delta_p * lateral_dir;

    // （可选）调试用：如果发现纠偏方向反了，把偏差乘-1即可
    // lateral_err *= -1;

    return lateral_err;
}
