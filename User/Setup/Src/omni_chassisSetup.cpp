#include "omni_chassisSetup.h"
#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include <cstdint>
#include "APP_Bezier_Curve.h"
#include "AutoCtrler.h"
#include "Module_Position.h"

PID_Position pid_yaw;
static bool pid_yaw_inited = false;


void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
     static uint64_t last_us = 0; 
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds(); 
    if(last_us == 0) 
    { 
        last_us = now_us; 
        return; 
    } 
    uint64_t dt_us = (now_us >= last_us) ? (now_us - last_us) : 0; 
    last_us = now_us; 
    if(dt_us == 0) 
        return; 
    if(dt_us > 200000) 
        dt_us = 200000; 
    float dt = dt_us * 1e-6f; 

    // 读取当前位置（里程计），供本帧使用，避免全局 RealPosData 链接问题
    RealPos __rp = Position::GetInstance(&huart1)->getRealPosData();

    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            this->chassis_manual_control_A();
            this->locked_yaw = __rp.world_yaw;
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {   
            this->chassis_manual_control_B();
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
            this->chassis_auto_control();
            break;
        }

        case CHASSIS_STOP:
        {
            this->chassis_stop();                  
            break;
        }
        default:
            break;
    }
	
	this->setWorldSpeed(this->target_chassis_twist_ );
    this->update();
    //debug_uart.printf_DMA("locked_yaw=%.2f now_yaw=%.2f yaw_ctrl=%.2f\n", this->locked_yaw, this->now_yaw, yaw_ctrl);
}


void OmniChassis_Setup::chassis_manual_control_A()
{
	this->target_chassis_twist_.vx = (static_cast<float>(AirJoy::getinstance().LEFT_X) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.vy = (static_cast<float>(AirJoy::getinstance().LEFT_Y) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.yaw_rate = (static_cast<float>(AirJoy::getinstance().RIGHT_Y) - 1500.0f) / 500.0f;    
}

void OmniChassis_Setup::chassis_manual_control_B()
{
    if (!pid_yaw_inited) {
        PID_Param_Config locked = {
            .kp = 0.2f,   
            .ki = 0.0f,
            .kd = 0.00006f,
            .I_Outlimit = 1.0f,
            .isIOutlimit = true,
            .output_limit = 1.0f,
            .deadband = 0.05f
        };
        pid_yaw.set_params(locked, 0.0f);
        pid_yaw_inited = true;
    }

    
    this->now_yaw = RealPosData.world_yaw;
    yaw_ctrl = pid_yaw.pid_calc(this->locked_yaw, this->now_yaw);

	this->target_chassis_twist_.vx = (static_cast<float>(AirJoy::getinstance().LEFT_X) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.vy = (static_cast<float>(AirJoy::getinstance().LEFT_Y) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.yaw_rate = yaw_ctrl;
    //debug_uart.printf_DMA("locked_yaw=%.2f now_yaw=%.2f yaw_ctrl=%.2f\n", this->locked_yaw, this->now_yaw, yaw_ctrl);
}

void OmniChassis_Setup::chassis_stop()
{
	this->target_chassis_twist_.vx = 0.0f;
    this->target_chassis_twist_.vy = 0.0f;
    this->target_chassis_twist_.yaw_rate = 0.0f;
}

void OmniChassis_Setup::chassis_auto_control()
{
	static bool pid_bc_inited = false;
	static PID_Position pid_bc; 

	if (!pid_bc_inited) {
		PID_Param_Config bc_param = { 
            .kp = 0.1f,
            .ki = 0.0f,
            .kd = 0.005f, 
            .I_Outlimit = 1.0f,
            .isIOutlimit = true, 
            .output_limit = 1.5f,
            .deadband = 0.01f };
        pid_bc.set_params(bc_param, 0.0f);
        pid_bc_inited = true;
	}

	Vector2D start_point(0.0f, 0.0f);
    Vector2D control_point(-6.0f, 16.0f);
    Vector2D end_point(12.0f, 14.0f);
    Vector2D point(0.01f, 0.01f);
    Vector2D point_last(0.01f, 0.01f);
	static BezierCurve bc(start_point, control_point, end_point);

    // 使用 Position 单例获取当前位姿，避免依赖未定义的全局符号
    RealPos rp = Position::GetInstance(&huart1)->getRealPosData();
    Vector2D robot_pos(rp.world_x, rp.world_y);
    float robot_yaw = rp.world_yaw;

    // 找到路径上最近的点，并计算横向误差
    float t_nearest; // 用于接收最近点的 t 参数
    float nearest_distance = bc.Get_Nearest_Distance(robot_pos, &t_nearest); // 最近点距离

    Vector2D nearest_point = bc.Get_Point(t_nearest);         // 最近点坐标
    Vector2D tangent_near = bc.Get_Tangent_Vector(t_nearest); // 切线方向
    Vector2D path_to_robot = robot_pos - nearest_point;      // 路径点指向机器人向量

    // 使用2D向量叉乘的 z 分量来判断机器人在切线的哪一侧
    float error_side = tangent_near.x * path_to_robot.y - tangent_near.y * path_to_robot.x;
    float signed_cross_track_error = (error_side >= 0.0f) ? nearest_distance : -nearest_distance;

    // 检查是否已到达终点附近
    const float goal_radius = 0.1f; 
    if (bc.Get_Current_Len(t_nearest) > (bc.Get_len() - goal_radius))
    {
        this->target_chassis_twist_.vx = 0.0f;
        this->target_chassis_twist_.vy = 0.0f;
        this->target_chassis_twist_.yaw_rate = 0.0f;
        return;
    }

    float current_len = bc.Get_Current_Len(t_nearest);

    // 使用参数步进法在参数空间搜索前视点 t_look
    // 先计算前视距离
    const float L_min = 0.12f, L_max = 1.0f;
    const float k_v = 0.8f; // 比例系数
    const float Vref = max_wheel_speed_ * 0.4f; // 期望切向速度标量
    float L = k_v * Vref;//  前视距离
    if (L < L_min) L = L_min;
    if (L > L_max) L = L_max;

    // 按参数步进搜索 t_look，从 t_nearest 开始，步长为 step_t
    const float step_t = 0.01f; // 参数步长
    float t_look = t_nearest;
    Vector2D p_cur = nearest_point; // P(t_nearest)
    while (t_look < 1.0f) {
        float t_next = t_look + step_t;
        if (t_next > 1.0f) t_next = 1.0f;
        Vector2D p_next = bc.Get_Point(t_next);
        float dist = (p_next - p_cur).magnitude();
        if (dist >= L || t_next >= 1.0f) {
            t_look = t_next;
            break;
        }
        t_look = t_next;
    }

    // 前视点切向单位向量 vdir
    Vector2D vdir = bc.Get_Tangent_Vector(t_look).normalize();

    // 由 PID 速度环得到修正速度 Av（沿法线方向）
    float lateral_correction = pid_bc.pid_calc(0.0f, -signed_cross_track_error); // 输出为速度（m/s）
    Vector2D normal(-vdir.y, vdir.x); 
    Vector2D Av = normal * lateral_correction; // 修正速度向量（世界系）

    // 合速度 Vcmd = Vref * vdir + Av
    Vector2D Vcmd_world = vdir * Vref + Av;

    // 将世界系速度变换到机器人局部坐标系
    float cosy = cosf(-robot_yaw);
    float siny = sinf(-robot_yaw);
    float vx_body = Vcmd_world.x * cosy - Vcmd_world.y * siny;
    float vy_body = Vcmd_world.x * siny + Vcmd_world.y * cosy;

    this->target_chassis_twist_.vx = vx_body;
    this->target_chassis_twist_.vy = vy_body;

    // 航向控制：使底盘朝向切线方向
    float target_yaw = atan2f(vdir.y, vdir.x);
    float yaw_err = target_yaw - robot_yaw;
    while (yaw_err > PI) yaw_err -= 2.0f * PI;
    while (yaw_err < -PI) yaw_err += 2.0f * PI;
    this->target_chassis_twist_.yaw_rate = pid_yaw.pid_calc(0.0f, -yaw_err);
}