// #include <cmath>
// #include "PathPlanner.h"
// #include "PathTracing.h"
// static const float M_PI = 3.14159265358979323846f;
// // 演示用地图尺寸
// const uint16_t MAP_WIDTH = 20;
// const uint16_t MAP_HEIGHT = 20;

// // 缓冲区定义
// uint8_t map_buffer[MAP_WIDTH * MAP_HEIGHT];
// AStarNode nodes_buffer[MAP_WIDTH * MAP_HEIGHT];
// AStarNode* open_list_buffer[500];  // 开放列表缓冲区
// GridPoint path_buffer[200];        // 路径点缓冲区
// Waypoint waypoint_buffer[200];     // 世界坐标路径点缓冲区

// // 网格到世界坐标的转换比例（米/网格）
// const float GRID_RESOLUTION = 0.1f;

// /**
//  * 创建演示地图 - 设置障碍物和可通行区域
//  */
// void createDemoMap(uint8_t* map_data) {
//     // 清空地图（全部设为可通行）- 使用循环替代 memset
//     for (uint32_t i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++) {
//         map_data[i] = CELL_FREE;
//     }
    
//     // 设置边界障碍物
//     for (int x = 0; x < MAP_WIDTH; x++) {
//         map_data[0 * MAP_WIDTH + x] = CELL_OBSTACLE;                    // 上边界
//         map_data[(MAP_HEIGHT-1) * MAP_WIDTH + x] = CELL_OBSTACLE;       // 下边界
//     }
//     for (int y = 0; y < MAP_HEIGHT; y++) {
//         map_data[y * MAP_WIDTH + 0] = CELL_OBSTACLE;                    // 左边界
//         map_data[y * MAP_WIDTH + (MAP_WIDTH-1)] = CELL_OBSTACLE;        // 右边界
//     }
    
//     // 设置内部障碍物 - 创建一个迷宫-like 环境
//     // 障碍物1：水平墙
//     for (int x = 3; x < 15; x++) {
//         map_data[5 * MAP_WIDTH + x] = CELL_OBSTACLE;
//     }
    
//     // 障碍物2：垂直墙
//     for (int y = 3; y < 12; y++) {
//         map_data[y * MAP_WIDTH + 8] = CELL_OBSTACLE;
//     }
    
//     // 障碍物3：L形障碍物
//     for (int x = 12; x < 17; x++) {
//         map_data[12 * MAP_WIDTH + x] = CELL_OBSTACLE;
//     }
//     for (int y = 8; y < 12; y++) {
//         map_data[y * MAP_WIDTH + 16] = CELL_OBSTACLE;
//     }
    
//     // 障碍物4：小方块
//     for (int y = 15; y < 18; y++) {
//         for (int x = 3; x < 6; x++) {
//             map_data[y * MAP_WIDTH + x] = CELL_OBSTACLE;
//         }
//     }
// }

// /**
//  * 将网格路径转换为世界坐标路径
//  */
// void convertGridPathToWorldPath(const GridPoint* grid_path, uint16_t path_length, 
//                                Waypoint* world_path, float resolution) {
//     for (uint16_t i = 0; i < path_length; i++) {
//         world_path[i].x = grid_path[i].x * resolution;
//         world_path[i].y = grid_path[i].y * resolution;
        
//         // 计算朝向角度（指向下一个点）
//         if (i < path_length - 1) {
//             float dx = world_path[i+1].x - world_path[i].x;
//             float dy = world_path[i+1].y - world_path[i].y;
//             world_path[i].theta = atan2f(dy, dx);
//         } else {
//             // 最后一个点保持之前的朝向
//             world_path[i].theta = (i > 0) ? world_path[i-1].theta : 0.0f;
//         }
//     }
// }

// /**
//  * 计算两点之间的角度差（用于显示跟踪误差）
//  */
// float calculateAngleError(float current_angle, float target_angle) {
//     float error = target_angle - current_angle;
//     // 归一化到 [-π, π]
//     while (error > M_PI) error -= 2 * M_PI;
//     while (error < -M_PI) error += 2 * M_PI;
//     return error;
// }

// /**
//  * 演示函数
//  */
// int test1() {
//     // ===============================
//     // 第一步：路径规划
//     // ===============================
    
//     // 创建路径规划器
//     PathPlanner planner(map_buffer, nodes_buffer, open_list_buffer, path_buffer,
//                        MAP_WIDTH, MAP_HEIGHT, 500, 200);
    
//     // 创建并设置地图
//     uint8_t map_data[MAP_WIDTH * MAP_HEIGHT];
//     createDemoMap(map_data);
//     planner.setMapData(map_data);
    
//     // 设置起点和终点
//     int16_t start_x = 1, start_y = 1;      // 网格坐标
//     int16_t goal_x = 18, goal_y = 18;      // 网格坐标
    
//     // 执行路径规划
//     bool planning_success = planner.findPath(start_x, start_y, goal_x, goal_y);
    
//     if (!planning_success) {
//         return -1;
//     }
    
//     // 获取规划结果
//     uint16_t grid_path_length = planner.getPathLength();
//     const GridPoint* grid_path = planner.getPath();
    
//     // 路径简化（可选）
//     planner.simplifyPath();
//     grid_path_length = planner.getPathLength();
    
//     // ===============================
//     // 第二步：路径跟踪准备
//     // ===============================
    
//     // 创建路径跟踪器
//     PathTracing tracker;
//     tracker.init(waypoint_buffer, 200);
    
//     // 配置跟踪参数（适合小型机器人）
//     tracker.setConfig(
//         0.3f,   // 最大线速度: 0.3 m/s
//         1.5f,   // 最大角速度: 1.5 rad/s
//         0.2f,   // 线加速度: 0.2 m/s2
//         1.0f,   // 角加速度: 1.0 rad/s2
//         0.05f,  // 目标容差: 0.05 m
//         0.2f    // 前视距离: 0.2 m
//     );
    
//     // 将网格路径转换为世界坐标路径
//     convertGridPathToWorldPath(grid_path, grid_path_length, waypoint_buffer, GRID_RESOLUTION);
    
//     // 将路径点添加到跟踪器
//     for (uint16_t i = 0; i < grid_path_length; i++) {
//         tracker.addWaypoint(waypoint_buffer[i].x, waypoint_buffer[i].y, waypoint_buffer[i].theta);
//     }
    
//     // 设置机器人初始状态（在世界坐标系中）
//     float start_world_x = start_x * GRID_RESOLUTION;
//     float start_world_y = start_y * GRID_RESOLUTION;
//     float start_theta = 0.0f;  // 初始朝向为0弧度（向右）
    
//     tracker.setRobotState(start_world_x, start_world_y, start_theta);
    
//     // 开始路径跟踪
//     if (!tracker.planPath()) {
//         return -1;
//     }
    
//     // ===============================
//     // 第三步：执行路径跟踪
//     // ===============================
    
//     float simulation_time = 0.0f;
//     const float TIME_STEP = 0.1f;  // 100ms 控制周期
//     const float MAX_SIMULATION_TIME = 60.0f;  // 最大仿真时间60秒
    
//     while (!tracker.isPathCompleted() && simulation_time < MAX_SIMULATION_TIME) {
//         // 执行一步跟踪控制
//         tracker.executeOneStep(TIME_STEP);
        
//         simulation_time += TIME_STEP;
        
//         // 简单延时，模拟真实控制周期（在实际系统中不需要）
//         // std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
    
//     if (!tracker.isPathCompleted()) {
//         return -1;
//     }
    
//     return 0;
// }