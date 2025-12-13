#include "omni_chassisSetup.h"

// 地图与格子尺寸配置
static const float CELL_SIZE_M = 1.2f; // 每格 1200mm = 1.2m

// 将栅格 (x,y) 转为世界坐标系中心点 (m)
static Vector2D gridToWorld(int gx, int gy)
{
    Vector2D p;
    p.x = gx * CELL_SIZE_M + CELL_SIZE_M * 0.5f;
    p.y = gy * CELL_SIZE_M + CELL_SIZE_M * 0.5f;
    return p;
}

// 将格号(1..30) 转为 grid坐标 (0-based)
static void cellNumToGrid(int cellNum, int &gx, int &gy)
{
    int idx = cellNum - 1;
    gx = idx % 5;
    gy = idx / 5;
}



void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
//     static uint64_t last_us = 0; 
//    uint64_t now_us = TimeStamp::getInstance().getMicroseconds(); 
//    if(last_us == 0) 
//    { 
//        last_us = now_us; 
//        return; 
//    } 
//    uint64_t dt__us = (now_us >= last_us) ? (now_us - last_us) : 0; 
//    last_us = now_us; 
//    if(dt__us == 0) 
//        return; 
//    if(dt__us > 200000) 
//        dt__us = 200000; 
//    float dt_ = dt__us * 1e-6f; 

    // 1. 定义一个静态的虚拟机器人位姿，并初始化
    if (!virtual_rp_initialized) {
        virtual_rp.world_x = 0.6f; // 起点 (0.5 * 1.2m)
        virtual_rp.world_y = 0.6f; // 起点 (0.5 * 1.2m)
        virtual_rp.world_yaw = 0; // 0, 朝向X轴正方向
        virtual_rp_initialized = true;
    }

    // 2. 根据上一帧的速度指令，对虚拟位置进行积分 (运动学模型更新)
    // 注意：target_chassis_twist_ 是在上一帧的末尾计算的
    float last_vx_body = this->target_chassis_twist_.vx;
    float last_vy_body = this->target_chassis_twist_.vy;
    //float last_yaw_rate = this->target_chassis_twist_.yaw_rate;

    float cos_yaw = cosf(virtual_rp.world_yaw);
    float sin_yaw = sinf(virtual_rp.world_yaw);

    // 将机体速度转换到世界坐标系下，然后积分
    world_vx = last_vx_body * cos_yaw - last_vy_body * sin_yaw;
    world_vy = last_vx_body * sin_yaw + last_vy_body * cos_yaw;

    virtual_rp.world_x += world_vx * dt_;
    virtual_rp.world_y += world_vy * dt_;
    //virtual_rp.world_yaw += last_yaw_rate * dt_;

    // 3. 使用虚拟位置点作为当前帧的输入
    // 不再读取 RealPosData，而是直接使用我们仿真的 virtual_rp
    rp = virtual_rp;

    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
					this->target_chassis_twist_.vx = 0.2f;
					this->target_chassis_twist_.vy = 0.0f;
					this->target_chassis_twist_.yaw_rate = 0.0f;
            //this->chassis_manual_control_A();
            //this->locked_yaw = rp.world_yaw;
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {   
            this->chassis_manual_control_B();
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
            this->chassis_auto_control(dt_);
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
}


void OmniChassis_Setup::chassis_manual_control_A()
{
	this->target_chassis_twist_.vx = (static_cast<float>(AirJoy::getinstance().LEFT_X) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.vy = (static_cast<float>(AirJoy::getinstance().LEFT_Y) - 1500.0f) / 500.0f * max_wheel_speed_;
    this->target_chassis_twist_.yaw_rate = (static_cast<float>(AirJoy::getinstance().RIGHT_Y) - 1500.0f) / 500.0f;    
}

void OmniChassis_Setup::chassis_manual_control_B()
{
    // 使用成员变量 pid_yaw_
    this->now_yaw = RealPosData.world_yaw;
    yaw_ctrl = this->pid_yaw_.pid_calc(this->locked_yaw, this->now_yaw);

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

// 世界坐标 -> grid (0-based)，并裁剪到地图范围
static void worldToGrid(float wx, float wy, int &gx, int &gy)
{
    int ix = static_cast<int>(floorf(wx / CELL_SIZE_M));
    int iy = static_cast<int>(floorf(wy / CELL_SIZE_M));
    if (ix < 0) ix = 0; if (ix > 4) ix = 4;
    if (iy < 0) iy = 0; if (iy > 5) iy = 5;
    gx = ix; gy = iy;
}



// 基于 A* 的路径规划与跟踪实现（dt_: 控制周期秒）
void OmniChassis_Setup::chassis_auto_control(float dt_)
{
    if (path_finished_) {
        this->target_chassis_twist_.vx = 0.0f;
        this->target_chassis_twist_.vy = 0.0f;
        this->target_chassis_twist_.yaw_rate = 0.0f;
			  debug_uart.printf_DMA("path finished\n");
        return;
    }

    // 获取当前位姿
    //RealPos rp;
    //rp.world_x = RealPosData.world_x;
    //rp.world_y = RealPosData.world_y;
    //rp.world_yaw = RealPosData.world_yaw;
    //rp.dx = RealPosData.dx;
    //rp.dy = RealPosData.dy;
    //rp.dyaw = RealPosData.dyaw;

    // 如果还未规划路径，则规划（起点为当前位置所属格或固定格1，终点为格号30）
    if (path_tracer_.getWaypointCount() == 0) {
        // 强制从格号1开始规划
        int sx = 0, sy = 0; 

        int gx, gy;
			
        cellNumToGrid(30, gx, gy); // 终点 cell 30

			
        debug_uart.printf_DMA("A*: planning from (%d,%d) to (%d,%d)\n", sx, sy, gx, gy);

        bool ok = path_planner_.findPath(static_cast<int16_t>(sx), static_cast<int16_t>(sy),
                                        static_cast<int16_t>(gx), static_cast<int16_t>(gy));
        if (!ok) {
            debug_uart.printf_DMA("A*: path not found\n");
            path_finished_ = true;
            return;
        }

        // 简化并转换为 world waypoints
        path_planner_.simplifyPath();
        const GridPoint* p = path_planner_.getPath();
        uint16_t len = path_planner_.getPathLength();
        for (uint16_t i = 0; i < len; ++i) {
            Vector2D wp = gridToWorld(p[i].x, p[i].y);
            path_tracer_.addWaypoint(wp.x, wp.y, 0.0f);
        }
        path_tracer_.planPath();
        debug_uart.printf_DMA("A*: planned %d waypoints\n", len);
    }

    // 跟踪路径
    if (path_tracer_.isPathCompleted()) {
        path_finished_ = true;
        debug_uart.printf_DMA("A*: path completed\n");
        return;
    }

    path_tracer_.setRobotState(rp.world_x, rp.world_y, rp.world_yaw);// 设置当前位置
    path_tracer_.executeOneStep(dt_);// 执行一步路径跟踪

    float linear_vel = 0.0f, angular_vel = 0.0f;// 获取跟踪输出速度
    path_tracer_.calculateMotionCommands(&linear_vel, &angular_vel);

    // 将 Pure Pursuit 输出的全局线速度映射到机器人局部坐标系 (vx, vy)
    // Pure Pursuit 输出的是沿着当前目标方向的线速度大小 linear_vel
    // 我们需要获取当前目标点的方向，或者利用 Pure Pursuit 内部计算出的期望航向
    // 这里简单地假设 linear_vel 是沿着机器人当前朝向到目标点的方向
    
    // 获取当前目标点以计算期望方向向量
    Waypoint target = path_tracer_.getCurrentTarget();
    float dx = target.x - rp.world_x;
    float dy = target.y - rp.world_y;
    float dist = sqrtf(dx*dx + dy*dy);
    
    float global_vx = 0.0f;
    float global_vy = 0.0f;

    if (dist > 0.001f) {
        // 归一化方向向量并乘以期望线速度
        global_vx = (dx / dist) * linear_vel;
        global_vy = (dy / dist) * linear_vel;
    }

    // 将全局速度转换到机器人局部坐标系
    // vx_body =  cos(yaw)*vx_global + sin(yaw)*vy_global
    // vy_body = -sin(yaw)*vx_global + cos(yaw)*vy_global
    float cos_yaw = cosf(rp.world_yaw);
    float sin_yaw = sinf(rp.world_yaw);

    this->target_chassis_twist_.vx =  cos_yaw * global_vx + sin_yaw * global_vy;
    this->target_chassis_twist_.vy = -sin_yaw * global_vx + cos_yaw * global_vy;
    this->target_chassis_twist_.yaw_rate = angular_vel;
}
