#include "omni_chassisSetup.h"
#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include <cstdint>
#include "APP_Bezier_Curve.h"
#include "AutoCtrler.h"

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
    Vector2D test_start(0.0f, 0.0f);//1号
    Vector2D test_goal(5.4f, 6.6f); //30号
        // 把 6m x 9m 的世界映射到 5 x 6 的栅格地图
        const float world_w = 6.0f; // 
        const float world_h = 7.2f; // 
        const int GRID_W = 5; // 列
        const int GRID_H = 6; // 行
        const float cell_w = world_w / static_cast<float>(GRID_W); // 1.2 m
        const float cell_h = world_h / static_cast<float>(GRID_H); // 1.2 m (7.2/6)
        const int origin_x = 0; // world (0,0) -> grid (0,0)
        const int origin_y = 0;

        auto worldToGrid = [&](const Vector2D &p)->std::pair<int,int>{
            int gx = origin_x + static_cast<int>(std::floor(p.x / cell_w));
            int gy = origin_y + static_cast<int>(std::floor(p.y / cell_h));
            gx = std::max(0, std::min(GRID_W-1, gx));
            gy = std::max(0, std::min(GRID_H-1, gy));
            return {gx, gy};
        };// 将世界坐标转换为栅格坐标

        auto gridToWorld = [&](int gx, int gy)->Vector2D{
            // 返回格子中心的世界坐标
            float wx = (gx + 0.5f) * cell_w;
            float wy = (gy + 0.5f) * cell_h;
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

        // 使用 PathPlanner 的 A* 路径点直接跟踪
        static std::vector<Vector2D> path_points;
        static size_t follow_index = 0;

        // 更新地图：先清空，再把指定世界坐标点标为障碍并做膨胀
        const uint8_t OCC = 1;
        // 清空地图（每次更新）
        for (int i = 0; i < GRID_W * GRID_H; ++i) map_buf[i] = 0;

        // 指定为障碍的 MapNum 索引（对应 MF_AutoCtrler::MapNum_RealPos）
        const int obstacle_indices[] = {6,7,8, 11,12,13, 16,17,18, 21,22,23}; // Indices from 0, so 7th is 6, etc.
        const size_t static_obs_count = sizeof(obstacle_indices) / sizeof(obstacle_indices[0]);

        // 标记障碍（直接使用 MapNum_RealPos）并收集障碍点列表用于膨胀
        std::vector<Vector2D> obstacles;
        obstacles.reserve(static_obs_count);
        for (size_t i = 0; i < static_obs_count; ++i) {
            int idx = obstacle_indices[i];
            if (idx < 0 || idx >= 30) continue;
            Point2D p = MF_AutoCtrler::MapNum_RealPos[idx];
            Vector2D obs(p.x, p.y);
            obstacles.push_back(obs);
            auto [ox, oy] = worldToGrid(obs);
            map_buf[oy * GRID_W + ox] = OCC;
        }

        // 膨胀障碍（按机器人半径），基于 5x6 网格：如果格子中心到任一障碍点的距离 <= robot_radius，则视为占用
        const float robot_radius = 0.2f; 
        const float robot_radius_sq = robot_radius * robot_radius;
        for (int y = 0; y < GRID_H; ++y) {
            for (int x = 0; x < GRID_W; ++x) {
                // 计算格子中心世界坐标
                Vector2D center = gridToWorld(x, y);
                for (size_t oi = 0; oi < obstacles.size(); ++oi) {
                    const Vector2D &obs = obstacles[oi];
                    float dx = center.x - obs.x;
                    float dy = center.y - obs.y;
                    if (dx*dx + dy*dy <= robot_radius_sq) {
                        map_buf[y * GRID_W + x] = OCC;
                        break;
                    }
                }
            }
        }

        // 将测试起点/终点转换为栅格坐标并调用 planner 进行路径规划
        auto [sx, sy] = worldToGrid(test_start);
        auto [gx, gy] = worldToGrid(test_goal);

        // 仅在需要时（地图/起点/终点变化或当前无路径）重新规划，避免每帧重算
        static uint32_t prev_map_hash = 0;
        static int prev_sx = -1, prev_sy = -1, prev_gx = -1, prev_gy = -1;

        // 计算简单的 map 哈希（FNV-1a）
        uint32_t map_hash = 2166136261u;
        for (int i = 0; i < GRID_W * GRID_H; ++i) {
            map_hash ^= static_cast<uint32_t>(map_buf[i]);
            map_hash *= 16777619u;
        }

        bool need_plan = path_points.empty() || map_hash != prev_map_hash || sx != prev_sx || sy != prev_sy || gx != prev_gx || gy != prev_gy;
        if (need_plan) {
            bool ok = planner.findPath((int16_t)sx, (int16_t)sy, (int16_t)gx, (int16_t)gy);
            if (ok) {
                uint16_t plen = planner.getPathLength();
                const GridPoint* p = planner.getPath();
                path_points.clear();
                // 将栅格路径点转换为世界坐标并存储
                for (uint16_t i = 0; i < plen; ++i) {
                    path_points.push_back(gridToWorld(p[i].x, p[i].y));
                }
                // 新路径生成时重置跟踪索引
                follow_index = 0;
                // 更新上一次的地图/起点/终点状态
                prev_map_hash = map_hash;
                prev_sx = sx; prev_sy = sy; prev_gx = gx; prev_gy = gy;
            } else {
                // 无可行路径：清空并保留 prev_map_hash 为避免重复无意义重试
                path_points.clear();
            }
        }

        // 机器人当前位姿和朝向
        Vector2D robot_pos(0.0f, 0.0f);
        float robot_yaw = 0.0f;
        robot_pos.x = RealPosData.world_x;
        robot_pos.y = RealPosData.world_y;
        robot_yaw = RealPosData.world_yaw;

        // 如果存在路径点，则逐点跟踪
        if (!path_points.empty()) {
            // 保证索引在范围内
            size_t idx = std::min(follow_index, path_points.size() - 1);
            Vector2D target = path_points[idx];

            Vector2D to_target = target - robot_pos;
            float dist = std::sqrt(to_target.x * to_target.x + to_target.y * to_target.y);

            const float reach_thresh = 0.08f; // 到达一个路径点的阈值（米）
            if (dist < reach_thresh) {
                if (follow_index + 1 < path_points.size()) {
                    follow_index++;
                    idx = follow_index;
                    target = path_points[idx];
                    to_target = target - robot_pos;
                    dist = std::sqrt(to_target.x * to_target.x + to_target.y * to_target.y);
                } else {
                    // 已到达最后一个点，停止
                    this->target_chassis_twist_.vx = 0.0f;
                    this->target_chassis_twist_.vy = 0.0f;
                    this->target_chassis_twist_.yaw_rate = 0.0f;
                    return;
                }
            }

            // 计算期望速度（朝向目标点）
            Vector2D dir(0.0f, 0.0f);
            if (dist > 1e-6f) {
                dir.x = to_target.x / dist;
                dir.y = to_target.y / dist;
            }
            // 简单速度策略：按距离或最大速度限制
            float desired_speed = max_wheel_speed_;
            // 可选：根据距离放慢速度
            float slow_dist = 0.5f;
            if (dist < slow_dist) desired_speed = std::min(desired_speed, dist / slow_dist * max_wheel_speed_);

            this->target_chassis_twist_.vx = dir.x * desired_speed;
            this->target_chassis_twist_.vy = dir.y * desired_speed;

            // 偏航朝向目标点
            float desired_yaw = atan2f(dir.y, dir.x);
            float yaw_err = desired_yaw - robot_yaw;
            while (yaw_err > PI) yaw_err -= 2.0f * PI;
            while (yaw_err < -PI) yaw_err += 2.0f * PI;
            float kp_yaw = 1.0f;
            this->target_chassis_twist_.yaw_rate = kp_yaw * yaw_err;
        } else {
            // 没有路径：停止
            this->target_chassis_twist_.vx = 0.0f;
            this->target_chassis_twist_.vy = 0.0f;
            this->target_chassis_twist_.yaw_rate = 0.0f;
        }

}