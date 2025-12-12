// #include <cmath>
// #include "PathPlanner.h"
// #include "PathTracing.h"
// static const float M_PI = 3.14159265358979323846f;
// // ��ʾ�õ�ͼ�ߴ�
// const uint16_t MAP_WIDTH = 20;
// const uint16_t MAP_HEIGHT = 20;

// // ����������
// uint8_t map_buffer[MAP_WIDTH * MAP_HEIGHT];
// AStarNode nodes_buffer[MAP_WIDTH * MAP_HEIGHT];
// AStarNode* open_list_buffer[500];  // �����б�������
// GridPoint path_buffer[200];        // ·���㻺����
// Waypoint waypoint_buffer[200];     // ��������·���㻺����

// // �������������ת����������/����
// const float GRID_RESOLUTION = 0.1f;

// /**
//  * ������ʾ��ͼ - �����ϰ���Ϳ�ͨ������
//  */
// void createDemoMap(uint8_t* map_data) {
//     // ��յ�ͼ��ȫ����Ϊ��ͨ�У�- ʹ��ѭ����� memset
//     for (uint32_t i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++) {
//         map_data[i] = CELL_FREE;
//     }
    
//     // ���ñ߽��ϰ���
//     for (int x = 0; x < MAP_WIDTH; x++) {
//         map_data[0 * MAP_WIDTH + x] = CELL_OBSTACLE;                    // �ϱ߽�
//         map_data[(MAP_HEIGHT-1) * MAP_WIDTH + x] = CELL_OBSTACLE;       // �±߽�
//     }
//     for (int y = 0; y < MAP_HEIGHT; y++) {
//         map_data[y * MAP_WIDTH + 0] = CELL_OBSTACLE;                    // ��߽�
//         map_data[y * MAP_WIDTH + (MAP_WIDTH-1)] = CELL_OBSTACLE;        // �ұ߽�
//     }
    
//     // �����ڲ��ϰ��� - ����һ���Թ�-like ����
//     // �ϰ���1��ˮƽǽ
//     for (int x = 3; x < 15; x++) {
//         map_data[5 * MAP_WIDTH + x] = CELL_OBSTACLE;
//     }
    
//     // �ϰ���2����ֱǽ
//     for (int y = 3; y < 12; y++) {
//         map_data[y * MAP_WIDTH + 8] = CELL_OBSTACLE;
//     }
    
//     // �ϰ���3��L���ϰ���
//     for (int x = 12; x < 17; x++) {
//         map_data[12 * MAP_WIDTH + x] = CELL_OBSTACLE;
//     }
//     for (int y = 8; y < 12; y++) {
//         map_data[y * MAP_WIDTH + 16] = CELL_OBSTACLE;
//     }
    
//     // �ϰ���4��С����
//     for (int y = 15; y < 18; y++) {
//         for (int x = 3; x < 6; x++) {
//             map_data[y * MAP_WIDTH + x] = CELL_OBSTACLE;
//         }
//     }
// }

// /**
//  * ������·��ת��Ϊ��������·��
//  */
// void convertGridPathToWorldPath(const GridPoint* grid_path, uint16_t path_length, 
//                                Waypoint* world_path, float resolution) {
//     for (uint16_t i = 0; i < path_length; i++) {
//         world_path[i].x = grid_path[i].x * resolution;
//         world_path[i].y = grid_path[i].y * resolution;
        
//         // ���㳯��Ƕȣ�ָ����һ���㣩
//         if (i < path_length - 1) {
//             float dx = world_path[i+1].x - world_path[i].x;
//             float dy = world_path[i+1].y - world_path[i].y;
//             world_path[i].theta = atan2f(dy, dx);
//         } else {
//             // ���һ���㱣��֮ǰ�ĳ���
//             world_path[i].theta = (i > 0) ? world_path[i-1].theta : 0.0f;
//         }
//     }
// }

// /**
//  * ��������֮��ĽǶȲ������ʾ������
//  */
// float calculateAngleError(float current_angle, float target_angle) {
//     float error = target_angle - current_angle;
//     // ��һ���� [-��, ��]
//     while (error > M_PI) error -= 2 * M_PI;
//     while (error < -M_PI) error += 2 * M_PI;
//     return error;
// }

// /**
//  * ��ʾ����
//  */
// int test1() {
//     // ===============================
//     // ��һ����·���滮
//     // ===============================
    
//     // ����·���滮��
//     PathPlanner planner(map_buffer, nodes_buffer, open_list_buffer, path_buffer,
//                        MAP_WIDTH, MAP_HEIGHT, 500, 200);
    
//     // ���������õ�ͼ
//     uint8_t map_data[MAP_WIDTH * MAP_HEIGHT];
//     createDemoMap(map_data);
//     planner.setMapData(map_data);
    
//     // ���������յ�
//     int16_t start_x = 1, start_y = 1;      // ��������
//     int16_t goal_x = 18, goal_y = 18;      // ��������
    
//     // ִ��·���滮
//     bool planning_success = planner.findPath(start_x, start_y, goal_x, goal_y);
    
//     if (!planning_success) {
//         return -1;
//     }
    
//     // ��ȡ�滮���
//     uint16_t grid_path_length = planner.getPathLength();
//     const GridPoint* grid_path = planner.getPath();
    
//     // ·���򻯣���ѡ��
//     planner.simplifyPath();
//     grid_path_length = planner.getPathLength();
    
//     // ===============================
//     // �ڶ�����·������׼��
//     // ===============================
    
//     // ����·��������
//     PathTracing tracker;
//     tracker.init(waypoint_buffer, 200);
    
//     // ���ø��ٲ������ʺ�С�ͻ����ˣ�
//     tracker.setConfig(
//         0.3f,   // ������ٶ�: 0.3 m/s
//         1.5f,   // �����ٶ�: 1.5 rad/s
//         0.2f,   // �߼��ٶ�: 0.2 m/s2
//         1.0f,   // �Ǽ��ٶ�: 1.0 rad/s2
//         0.05f,  // Ŀ���ݲ�: 0.05 m
//         0.2f    // ǰ�Ӿ���: 0.2 m
//     );
    
//     // ������·��ת��Ϊ��������·��
//     convertGridPathToWorldPath(grid_path, grid_path_length, waypoint_buffer, GRID_RESOLUTION);
    
//     // ��·�������ӵ�������
//     for (uint16_t i = 0; i < grid_path_length; i++) {
//         tracker.addWaypoint(waypoint_buffer[i].x, waypoint_buffer[i].y, waypoint_buffer[i].theta);
//     }
    
//     // ���û����˳�ʼ״̬������������ϵ�У�
//     float start_world_x = start_x * GRID_RESOLUTION;
//     float start_world_y = start_y * GRID_RESOLUTION;
//     float start_theta = 0.0f;  // ��ʼ����Ϊ0���ȣ����ң�
    
//     tracker.setRobotState(start_world_x, start_world_y, start_theta);
    
//     // ��ʼ·������
//     if (!tracker.planPath()) {
//         return -1;
//     }
    
//     // ===============================
//     // ��������ִ��·������
//     // ===============================
    
//     float simulation_time = 0.0f;
//     const float TIME_STEP = 0.1f;  // 100ms ��������
//     const float MAX_SIMULATION_TIME = 60.0f;  // ������ʱ��60��
    
//     while (!tracker.isPathCompleted() && simulation_time < MAX_SIMULATION_TIME) {
//         // ִ��һ�����ٿ���
//         tracker.executeOneStep(TIME_STEP);
        
//         simulation_time += TIME_STEP;
        
//         // ����ʱ��ģ����ʵ�������ڣ���ʵ��ϵͳ�в���Ҫ��
//         // std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
    
//     if (!tracker.isPathCompleted()) {
//         return -1;
//     }
    
//     return 0;
// }123