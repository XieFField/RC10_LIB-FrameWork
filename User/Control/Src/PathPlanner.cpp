#include "PathPlanner.h"
#include <math.h>
#include <string.h>

// 8个方向的移动向量 (上, 右上, 右, 右下, 下, 左下, 左, 左上)
// 用于A*算法中的邻居节点扩展
static const int16_t DIRECTION_X[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int16_t DIRECTION_Y[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

// 构造函数 - 初始化路径规划器并使用外部提供的缓冲区
PathPlanner::PathPlanner(uint8_t* map_buffer, AStarNode* nodes_buffer,
                         AStarNode** open_list_buffer, GridPoint* path_buffer,
                         uint16_t map_width, uint16_t map_height,
                         uint16_t max_open_list_size, uint16_t max_path_length) {
    // 初始化配置参数
    config_.map_width = map_width;
    config_.map_height = map_height;
    config_.max_path_length = max_path_length;
    config_.diagonal_cost = 1.414f;  // 对角线移动代价 √2 ≈ 1.414
    config_.straight_cost = 1.0f;    // 直线移动代价
    
    // 设置外部提供的缓冲区
    map_data_ = map_buffer;
    nodes_ = nodes_buffer;
    open_list_ = open_list_buffer;
    path_ = path_buffer;
    
    // 初始化开放列表
    open_list_capacity_ = max_open_list_size;
    open_list_size_ = 0;
    path_length_ = 0;
    
    // 初始化节点网格状态
    initNodes();
}

// 初始化所有节点状态 - 为A*搜索做准备
void PathPlanner::initNodes() {
    uint32_t total_nodes = config_.map_width * config_.map_height;
    for (uint32_t i = 0; i < total_nodes; i++) {
        nodes_[i].x = i % config_.map_width;     // 计算节点x坐标
        nodes_[i].y = i / config_.map_width;     // 计算节点y坐标
        nodes_[i].g_cost = 1e9f;  // 初始化为极大值，表示尚未访问
        nodes_[i].h_cost = 0;
        nodes_[i].f_cost = 1e9f;
        nodes_[i].parent_x = -1;  // 无效父节点坐标
        nodes_[i].parent_y = -1;
        nodes_[i].is_open = false;    // 不在开放列表中
        nodes_[i].is_closed = false;  // 不在关闭列表中
    }
}

// 重置节点状态 - 用于新的路径搜索
void PathPlanner::resetNodes() {
    uint32_t total_nodes = config_.map_width * config_.map_height;
    for (uint32_t i = 0; i < total_nodes; i++) {
        nodes_[i].g_cost = 1e9f;
        nodes_[i].f_cost = 1e9f;
        nodes_[i].parent_x = -1;
        nodes_[i].parent_y = -1;
        nodes_[i].is_open = false;
        nodes_[i].is_closed = false;
    }
    open_list_size_ = 0;    // 清空开放列表
    path_length_ = 0;       // 重置路径长度
}

// 计算启发式代价 - 使用对角线距离（适合8方向移动）
float PathPlanner::calculateHeuristic(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    int16_t dx = abs(x1 - x2);
    int16_t dy = abs(y1 - y2);
    // 对角线距离公式：D*(dx+dy) + (D2-2*D)*min(dx,dy)
    // 其中D是直线代价，D2是对角线代价
    return config_.straight_cost * (dx + dy) + 
           (config_.diagonal_cost - 2 * config_.straight_cost) * fmin(dx, dy);
}

// 检查坐标是否在地图有效范围内
bool PathPlanner::isValidCell(int16_t x, int16_t y) {
    return (x >= 0 && x < config_.map_width && y >= 0 && y < config_.map_height);
}

// 检查指定单元格是否为障碍物
bool PathPlanner::isObstacle(int16_t x, int16_t y) {
    if (!isValidCell(x, y)) return true;  // 地图外视为障碍物
    uint32_t index = y * config_.map_width + x;
    return (map_data_[index] == CELL_OBSTACLE);
}

// 添加节点到开放列表 - 使用插入排序保持列表有序（按f_cost升序）
void PathPlanner::addToOpenList(AStarNode* node) {
    if (open_list_size_ >= open_list_capacity_) return;  // 开放列表已满
    
    open_list_[open_list_size_++] = node;
    node->is_open = true;
    
    // 插入排序 - 将新节点插入到正确位置
    for (int i = open_list_size_ - 1; i > 0; i--) {
        if (open_list_[i]->f_cost < open_list_[i-1]->f_cost) {
            // 交换节点位置
            AStarNode* temp = open_list_[i];
            open_list_[i] = open_list_[i-1];
            open_list_[i-1] = temp;
        } else {
            break;  // 列表已有序
        }
    }
}

// 从开放列表中取出代价最小的节点（列表首元素）
AStarNode* PathPlanner::popBestFromOpenList() {
    if (open_list_size_ == 0) return nullptr;
    
    AStarNode* best = open_list_[0];  // 最小代价节点在列表开头
    
    // 移动剩余元素向前填充
    for (uint16_t i = 1; i < open_list_size_; i++) {
        open_list_[i-1] = open_list_[i];
    }
    
    open_list_size_--;
    best->is_open = false;
    return best;
}

// 从开放列表中移除指定节点
void PathPlanner::removeFromOpenList(AStarNode* node) {
    for (uint16_t i = 0; i < open_list_size_; i++) {
        if (open_list_[i] == node) {
            // 移动后续元素向前填充
            for (uint16_t j = i + 1; j < open_list_size_; j++) {
                open_list_[j-1] = open_list_[j];
            }
            open_list_size_--;
            node->is_open = false;
            break;
        }
    }
}

// 设置地图数据 - 复制外部地图到内部缓冲区
void PathPlanner::setMapData(const uint8_t* map_data) {
    memcpy(map_data_, map_data, config_.map_width * config_.map_height * sizeof(uint8_t));
}

// 设置移动代价配置
void PathPlanner::setConfig(float diagonal_cost, float straight_cost) {
    config_.diagonal_cost = diagonal_cost;
    config_.straight_cost = straight_cost;
}

// A*路径规划主函数 - 在起点和终点之间寻找最优路径
bool PathPlanner::findPath(int16_t start_x, int16_t start_y, 
                          int16_t goal_x, int16_t goal_y) {
    // 重置节点状态，准备新的搜索
    resetNodes();
    
    // 检查起点和终点是否有效
    if (!isValidCell(start_x, start_y) || !isValidCell(goal_x, goal_y) ||
        isObstacle(start_x, start_y) || isObstacle(goal_x, goal_y)) {
        return false;  // 无效的起点或终点
    }
    
    // 获取起点节点
    uint32_t start_index = start_y * config_.map_width + start_x;
    AStarNode* start_node = &nodes_[start_index];
    
    // 初始化起点节点
    start_node->g_cost = 0;  // 起点到起点的代价为0
    start_node->h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
    start_node->f_cost = start_node->h_cost;  // f = g + h
    start_node->parent_x = -1;  // 起点没有父节点
    start_node->parent_y = -1;
    
    addToOpenList(start_node);  // 将起点加入开放列表
    
    // A*主循环 - 直到开放列表为空或找到路径
    while (open_list_size_ > 0) {
        // 获取当前代价最小的节点
        AStarNode* current = popBestFromOpenList();
        if (current == nullptr) break;
        
        current->is_closed = true;  // 将当前节点标记为已处理
        
        // 检查是否到达目标点
        if (current->x == goal_x && current->y == goal_y) {
            reconstructPath(goal_x, goal_y);  // 回溯重构路径
            return true;  // 成功找到路径
        }
        
        // 扩展当前节点的邻居节点
        expandNode(current, goal_x, goal_y);
    }
    
    return false;  // 没有找到路径
}

// 扩展当前节点的8个邻居节点
void PathPlanner::expandNode(AStarNode* current, int16_t goal_x, int16_t goal_y) {
    for (int i = 0; i < 8; i++) {
        // 计算邻居节点坐标
        int16_t neighbor_x = current->x + DIRECTION_X[i];
        int16_t neighbor_y = current->y + DIRECTION_Y[i];
        
        // 检查邻居是否有效且不是障碍物
        if (!isValidCell(neighbor_x, neighbor_y) || isObstacle(neighbor_x, neighbor_y)) {
            continue;
        }
        
        // 对角线移动的特殊检查：确保相邻单元格可通行
        if (DIRECTION_X[i] != 0 && DIRECTION_Y[i] != 0) {
            if (isObstacle(current->x + DIRECTION_X[i], current->y) &&
                isObstacle(current->x, current->y + DIRECTION_Y[i])) {
                continue;  // 如果两个相邻单元格都是障碍物，禁止对角线移动
            }
        }
        
        // 获取邻居节点
        uint32_t neighbor_index = neighbor_y * config_.map_width + neighbor_x;
        AStarNode* neighbor = &nodes_[neighbor_index];
        
        // 跳过已关闭的节点
        if (neighbor->is_closed) continue;
        
        // 计算移动到邻居节点的代价
        float move_cost = (DIRECTION_X[i] != 0 && DIRECTION_Y[i] != 0) ? 
                         config_.diagonal_cost : config_.straight_cost;
        
        float tentative_g = current->g_cost + move_cost;  // 经过当前节点到邻居的代价
        
        // 如果找到更好的路径到邻居节点
        if (tentative_g < neighbor->g_cost) {
            neighbor->parent_x = current->x;  // 更新父节点
            neighbor->parent_y = current->y;
            neighbor->g_cost = tentative_g;   // 更新实际代价
            neighbor->h_cost = calculateHeuristic(neighbor_x, neighbor_y, goal_x, goal_y);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;  // 更新总代价
            
            if (!neighbor->is_open) {
                addToOpenList(neighbor);  // 首次发现，加入开放列表
            } else {
                // 节点已在开放列表中，需要重新排序
                removeFromOpenList(neighbor);
                addToOpenList(neighbor);
            }
        }
    }
}

// 从终点回溯重构路径
void PathPlanner::reconstructPath(int16_t end_x, int16_t end_y) {
    // 第一遍遍历：从终点回溯计数路径长度
    uint16_t temp_length = 0;
    int16_t cx = end_x;
    int16_t cy = end_y;
    
    // 回溯直到起点（父节点为-1）或达到最大路径长度
    while (cx != -1 && cy != -1 && isValidCell(cx, cy) && temp_length < config_.max_path_length) {
        temp_length++;
        uint32_t idx = cy * config_.map_width + cx;
        AStarNode* node = &nodes_[idx];
        cx = node->parent_x;
        cy = node->parent_y;
    }

    if (temp_length == 0) {
        path_length_ = 0;
        return;  // 空路径
    }

    // 第二遍遍历：从终点回溯，将路径点按从起点到终点的顺序存储
    uint16_t write_pos = (temp_length > 0) ? (temp_length - 1) : 0;
    cx = end_x;
    cy = end_y;
    uint16_t written = 0;
    
    while (cx != -1 && cy != -1 && isValidCell(cx, cy) && written < temp_length) {
        path_[write_pos].x = cx;  // 存储路径点坐标
        path_[write_pos].y = cy;
        written++;
        if (write_pos == 0) break; // 防止下溢
        write_pos--;
        uint32_t idx = cy * config_.map_width + cx;
        AStarNode* node = &nodes_[idx];
        cx = node->parent_x;
        cy = node->parent_y;
    }

    path_length_ = written;  // 设置最终路径长度
}

// 获取当前路径长度
uint16_t PathPlanner::getPathLength() const {
    return path_length_;
}

// 获取路径点数组
const GridPoint* PathPlanner::getPath() const {
    return path_;
}

// 检查两点之间是否有直视路径（无障碍物）
bool PathPlanner::lineOfSight(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    // Bresenham直线算法变种，用于检查直线上的障碍物
    int16_t dx = abs(x2 - x1);
    int16_t dy = abs(y2 - y1);
    int16_t sx = (x1 < x2) ? 1 : -1;
    int16_t sy = (y1 < y2) ? 1 : -1;
    int16_t err = dx - dy;
    
    int16_t current_x = x1;
    int16_t current_y = y1;
    
    // 遍历直线上的所有点
    while (current_x != x2 || current_y != y2) {
        if (isObstacle(current_x, current_y)) {
            return false;  // 发现障碍物，没有直视路径
        }
        
        // Bresenham算法步进
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            current_x += sx;
        }
        if (e2 < dx) {
            err += dx;
            current_y += sy;
        }
    }
    
    return true;  // 没有发现障碍物，存在直视路径
}

// 路径简化 - 使用贪婪算法移除不必要的中间点
void PathPlanner::simplifyPath() {
    if (path_length_ <= 2) return;  // 路径太短无需简化
    
    // 原地贪婪法：从起点开始，每次寻找最远的可视点作为下一个保留点
    uint16_t write_idx = 0;
    uint16_t read_idx = 0;
    
    // 保留起点
    path_[write_idx++] = path_[0];

    while (read_idx < path_length_ - 1) {
        // 寻找从当前点能直接看到的最远点
        uint16_t farthest = read_idx + 1;
        for (uint16_t j = read_idx + 1; j < path_length_; ++j) {
            if (lineOfSight(path_[read_idx].x, path_[read_idx].y, path_[j].x, path_[j].y)) {
                farthest = j;  // 更新最远可视点
            } else {
                break;  // 发现不可视点，停止搜索
            }
        }

        // 将最远可视点加入简化路径
        path_[write_idx++] = path_[farthest];

        if (farthest >= path_length_ - 1) break;  // 到达终点
        read_idx = farthest;  // 从新的点继续搜索
    }

    path_length_ = write_idx;  // 更新简化后的路径长度
}

// 清除当前路径
void PathPlanner::clearPath() {
    path_length_ = 0;
}