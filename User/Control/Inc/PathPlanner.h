#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
}
#endif

// 地图单元格状态 - 用于表示网格地图中每个单元格的状态
typedef enum {
    CELL_FREE = 0,      // 空闲单元格，可通行
    CELL_OBSTACLE = 1,  // 障碍物单元格，不可通行
    CELL_START = 2,     // 路径起点
    CELL_GOAL = 3,      // 路径终点
    CELL_PATH = 4       // 规划出的路径
} CellState;

// 节点结构体，用于A*算法 - 存储搜索过程中的节点信息
typedef struct {
    int16_t x;          // 节点在网格中的x坐标
    int16_t y;          // 节点在网格中的y坐标
    float g_cost;       // 从起点到当前节点的实际代价
    float h_cost;       // 从当前节点到终点的启发式代价估计
    float f_cost;       // 总代价 (g_cost + h_cost)
    int16_t parent_x;   // 父节点坐标，用于路径回溯
    int16_t parent_y;
    bool is_open;       // 标记节点是否在开放列表中
    bool is_closed;     // 标记节点是否在关闭列表中
} AStarNode;

// 网格路径点 - 用于存储网格坐标系中的路径点，避免与跟踪模块的浮点Waypoint冲突
typedef struct {
    int16_t x;          // 网格x坐标
    int16_t y;          // 网格y坐标
} GridPoint;

// 路径规划器配置参数
typedef struct {
    uint16_t map_width;     // 地图宽度（网格数）
    uint16_t map_height;    // 地图高度（网格数）
    uint16_t max_path_length; // 最大路径长度
    float diagonal_cost;    // 对角线移动代价（通常为√2）
    float straight_cost;    // 直线移动代价（通常为1）
} PathPlannerConfig;

class PathPlanner {
private:
    // 地图数据 (0=空闲, 1=障碍物) - 存储网格地图信息
    uint8_t* map_data_;
    
    // A*节点网格 - 存储所有网格节点的搜索状态
    AStarNode* nodes_;
    
    // 开放列表 (用于存储待检查节点) - 优先队列的简化实现
    AStarNode** open_list_;
    uint16_t open_list_size_;      // 当前开放列表大小
    uint16_t open_list_capacity_;  // 开放列表最大容量
    
    // 最终路径 (网格点) - 存储规划出的路径点序列
    GridPoint* path_;
    uint16_t path_length_;         // 当前路径长度
    
    // 配置参数
    PathPlannerConfig config_;
    
    // 私有方法
    void initNodes();      // 初始化所有节点状态
    void resetNodes();     // 重置节点状态用于新的搜索
    float calculateHeuristic(int16_t x1, int16_t y1, int16_t x2, int16_t y2); // 计算启发式代价
    bool isValidCell(int16_t x, int16_t y);    // 检查坐标是否在地图范围内
    bool isObstacle(int16_t x, int16_t y);     // 检查单元格是否为障碍物
    void addToOpenList(AStarNode* node);       // 添加节点到开放列表
    AStarNode* popBestFromOpenList();          // 从开放列表中取出代价最小的节点
    void removeFromOpenList(AStarNode* node);  // 从开放列表中移除指定节点
    void reconstructPath(int16_t end_x, int16_t end_y); // 从终点回溯重构路径
    void expandNode(AStarNode* current, int16_t goal_x, int16_t goal_y); // 扩展当前节点的邻居
    
public:
    // 构造函数 - 使用外部提供的缓冲区，避免动态内存分配
    PathPlanner(uint8_t* map_buffer, AStarNode* nodes_buffer, 
                AStarNode** open_list_buffer, GridPoint* path_buffer,
                uint16_t map_width, uint16_t map_height, 
                uint16_t max_open_list_size, uint16_t max_path_length);
    
    // 设置地图数据 - 复制外部地图数据到内部缓冲区
    void setMapData(const uint8_t* map_data);
    
    // 设置配置参数 - 调整移动代价权重
    void setConfig(float diagonal_cost, float straight_cost);
    
    // A*路径规划主函数 - 在起点和终点之间寻找最优路径
    bool findPath(int16_t start_x, int16_t start_y, 
                  int16_t goal_x, int16_t goal_y);
    
    // 获取规划结果
    uint16_t getPathLength() const;        // 获取路径长度
    const GridPoint* getPath() const;      // 获取路径点数组
    
    // 工具函数
    bool lineOfSight(int16_t x1, int16_t y1, int16_t x2, int16_t y2); // 检查两点之间是否有直视路径
    void simplifyPath();   // 路径简化，移除不必要的中间点
    
    // 清除规划结果 - 重置路径状态
    void clearPath();
};

#endif // PATH_PLANNER_H