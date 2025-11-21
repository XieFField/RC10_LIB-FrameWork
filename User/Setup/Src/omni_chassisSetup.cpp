#include "omni_chassisSetup.h"
#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include "APP_Bezier_Curve.h"

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

    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            this->chassis_manual_control_A();
            this->locked_yaw = RealPosData.world_yaw;
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

    // 测试起点/终点
    Vector2D test_start(0.0f, 0.0f);
    Vector2D test_goal(1.2f, 1.6f);

        const float resolution = 0.1f; 
        const int GRID_W = 60;
        const int GRID_H = 60;
        const int origin_x = GRID_W / 2;
        const int origin_y = GRID_H / 2;

        auto worldToGrid = [&](const Vector2D &p)->std::pair<int,int>{
            int gx = origin_x + static_cast<int>(std::round(p.x / resolution));
            int gy = origin_y + static_cast<int>(std::round(p.y / resolution));
            gx = std::max(0, std::min(GRID_W-1, gx));
            gy = std::max(0, std::min(GRID_H-1, gy));
            return {gx, gy};
        };// 将世界坐标转换为栅格坐标

        auto gridToWorld = [&](int gx, int gy)->Vector2D{
            float wx = (gx - origin_x) * resolution;
            float wy = (gy - origin_y) * resolution;
            return Vector2D(wx, wy);
        };

        // 为 PathPlanner 准备的静态缓冲区
        static uint8_t map_buf[GRID_W * GRID_H];
        static AStarNode nodes_buf[GRID_W * GRID_H];
        static AStarNode* open_list_buf[GRID_W * GRID_H];
        static GridPoint path_buf[GRID_W * GRID_H];
        static bool planner_inited = false;
        static PathPlanner planner(map_buf, nodes_buf, open_list_buf, path_buf, GRID_W, GRID_H, GRID_W*GRID_H, GRID_W*GRID_H);

        if(!planner_inited){

            for(int i=0;i<GRID_W*GRID_H;i++) map_buf[i] = 0;
            planner_inited = true;
        }

        static std::vector<BezierCurve> beziers;
        static size_t follow_index = 0;

        // 将测试起点/终点转换为栅格坐标并调用 planner 进行路径规划
        auto [sx, sy] = worldToGrid(test_start);
        auto [gx, gy] = worldToGrid(test_goal);

        bool ok = planner.findPath((int16_t)sx, (int16_t)sy, (int16_t)gx, (int16_t)gy);
        if(ok){
            uint16_t plen = planner.getPathLength();
            const GridPoint* p = planner.getPath();
            beziers.clear();
            if(plen == 1){} //路径只有一个点
            else if(plen == 2){
                Vector2D p0 = gridToWorld(p[0].x, p[0].y);
                Vector2D p1 = gridToWorld(p[1].x, p[1].y);
                Vector2D ctrl = (p0 + p1) * 0.5f;
                beziers.emplace_back(p0, ctrl, p1);
            } else {
                for(uint16_t i=0;i+1<plen;i++){
                    Vector2D p0 = gridToWorld(p[i].x, p[i].y);
                    Vector2D p1 = gridToWorld(p[i+1].x, p[i+1].y);
                    Vector2D ctrl = (p0 + p1) * 0.5f;
                    beziers.emplace_back(p0, ctrl, p1);
                }//对路径上每一对相邻点 (p[i], p[i+1]) 创建一段贝塞尔
            }
            follow_index = 0;
        }

        // 机器人当前位姿和朝向
        Vector2D robot_pos(0.0f, 0.0f);
        float robot_yaw = 0.0f;

        robot_pos.x = RealPosData.world_x;
        robot_pos.y = RealPosData.world_y;
        robot_yaw = RealPosData.world_yaw;
       
        // 如果存在贝塞尔曲线段，则使用曲线切线方向与贝塞尔提供的最大速度进行跟踪
        if(!beziers.empty()){
            size_t seg = std::min(follow_index, beziers.size()-1);
            // 使用真实里程计位置投影到贝塞尔曲线上，得到最近的参数 t，避免简单累加
            if(seg < beziers.size()){
                BezierCurve &bc = beziers[seg];
                float t_local = 0.0f;
                // Get_Nearest_Distance 会返回距离并通过 t_local 输出最接近点的 t
                float dist_to_curve = bc.Get_Nearest_Distance(robot_pos, &t_local);

                // 如果已经接近当前段的末端，切换到下一段
                if (t_local > 0.98f) {
                    if (follow_index + 1 < beziers.size()) {
                        follow_index++;
                      
                    }
                }

                // 采样曲线上的点与切线
                Vector2D point_on_curve = bc.Get_Point(t_local);
                Vector2D tangent = bc.Get_Tangent_Vector(t_local).normalize();

                // 使用贝塞尔的最大速度作为期望速度并受限
                float b_max = bc.Get_Max_Vel(t_local);
                float desired_speed = b_max;
                if (desired_speed > max_wheel_speed_) desired_speed = max_wheel_speed_;

                // 在世界坐标系下沿切线方向设置底盘速度
                this->target_chassis_twist_.vx = tangent.x * desired_speed;
                this->target_chassis_twist_.vy = tangent.y * desired_speed;

                // 简单的偏航跟踪：朝向切线方向
                float desired_yaw = atan2f(tangent.y, tangent.x);
                float yaw_err = desired_yaw - robot_yaw;
                // 归一化到 [-pi, pi]
                while (yaw_err > PI) yaw_err -= 2.0f * PI;
                while (yaw_err < -PI) yaw_err += 2.0f * PI;
                float kp_yaw = 1.0f; // 用p控制偏航
                this->target_chassis_twist_.yaw_rate = kp_yaw * yaw_err;
            }
        } else {
            // 回退：停止
            this->target_chassis_twist_.vx = 0.0f;
            this->target_chassis_twist_.vy = 0.0f;
            this->target_chassis_twist_.yaw_rate = 0.0f;
        }

}