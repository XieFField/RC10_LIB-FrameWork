#include "PathA.h"
#include <cmath>
#include <cstring>
#include <algorithm>

// 构造函数
AStarPathFinder::AStarPathFinder(int width, int height, float size, const Vector2D& origin) 
    : gridWidth(width), gridHeight(height), cellSize(size), gridOrigin(origin), 
      closedIndex(0), currentIndex(0), pathPointCount(0) {
    
    // 初始化障碍物网格
    clearObstacles();
    
    // 初始化起点和终点
    startPos = {0.0f, 0.0f};
    goalPos = {0.0f, 0.0f};
}

// 坐标转换：世界坐标转网格坐标
GridPoint AStarPathFinder::worldToGrid(const Vector2D& worldPos) {
    GridPoint gridPos;
    gridPos.x = static_cast<int>((worldPos.x - gridOrigin.x) / cellSize);
    gridPos.y = static_cast<int>((worldPos.y - gridOrigin.y) / cellSize);
    return gridPos;
}

// 坐标转换：网格坐标转世界坐标
Vector2D AStarPathFinder::gridToWorld(const GridPoint& gridPos) {
    Vector2D worldPos;
    worldPos.x = gridPos.x * cellSize + gridOrigin.x;
    worldPos.y = gridPos.y * cellSize + gridOrigin.y;
    return worldPos;
}

// 网格管理
void AStarPathFinder::initializeGrid(int width, int height, float size, const Vector2D& origin) {
    gridWidth = width;
    gridHeight = height;
    cellSize = size;
    gridOrigin = origin;
    
    // 确保不超过最大尺寸
    if (gridWidth > MAX_GRID_WIDTH) gridWidth = MAX_GRID_WIDTH;
    if (gridHeight > MAX_GRID_HEIGHT) gridHeight = MAX_GRID_HEIGHT;
    
    // 清空障碍物网格
    clearObstacles();
}

void AStarPathFinder::clearGrid() {
    clearObstacles();
    openList.clear();
    closedIndex = 0;
    currentIndex = 0;
    pathPointCount = 0;
}

// 障碍物管理
void AStarPathFinder::setObstacle(int gridX, int gridY, bool isObstacle) {
    if (isValidGridPosition(gridX, gridY)) {
        obstacleGrid[gridY * gridWidth + gridX] = isObstacle;
    }
}

void AStarPathFinder::setObstacle(const GridPoint& gridPos, bool isObstacle) {
    setObstacle(gridPos.x, gridPos.y, isObstacle);
}

bool AStarPathFinder::isObstacle(int gridX, int gridY) {
    if (!isValidGridPosition(gridX, gridY)) {
        return true; // 超出边界的视为障碍物
    }
    return obstacleGrid[gridY * gridWidth + gridX];
}

bool AStarPathFinder::isObstacle(const GridPoint& gridPos) {
    return isObstacle(gridPos.x, gridPos.y);
}

// 便捷障碍物设置方法
void AStarPathFinder::setPointObstacle(const Vector2D& worldPos, bool isObstacle) {
    GridPoint gridPos = worldToGrid(worldPos);
    setObstacle(gridPos, isObstacle);
}

void AStarPathFinder::setRectangleObstacle(const Vector2D& minCorner, const Vector2D& maxCorner, bool isObstacle) {
    GridPoint minGrid = worldToGrid(minCorner);
    GridPoint maxGrid = worldToGrid(maxCorner);
    
    // 确保minGrid是左下角，maxGrid是右上角
    if (minGrid.x > maxGrid.x) std::swap(minGrid.x, maxGrid.x);
    if (minGrid.y > maxGrid.y) std::swap(minGrid.y, maxGrid.y);
    
    // 设置矩形区域内的所有网格点
    for (int y = minGrid.y; y <= maxGrid.y; y++) {
        for (int x = minGrid.x; x <= maxGrid.x; x++) {
            setObstacle(x, y, isObstacle);
        }
    }
}

void AStarPathFinder::setCircleObstacle(const Vector2D& center, float radius, bool isObstacle) {
    GridPoint centerGrid = worldToGrid(center);
    int radiusGrid = static_cast<int>(radius / cellSize) + 1; // 向上取整
    
    // 使用Bresenham算法画圆
    int x = 0;
    int y = radiusGrid;
    int d = 3 - 2 * radiusGrid;
    
    while (x <= y) {
        // 设置圆周上的8个对称点
        setObstacle(centerGrid.x + x, centerGrid.y + y, isObstacle);
        setObstacle(centerGrid.x - x, centerGrid.y + y, isObstacle);
        setObstacle(centerGrid.x + x, centerGrid.y - y, isObstacle);
        setObstacle(centerGrid.x - x, centerGrid.y - y, isObstacle);
        setObstacle(centerGrid.x + y, centerGrid.y + x, isObstacle);
        setObstacle(centerGrid.x - y, centerGrid.y + x, isObstacle);
        setObstacle(centerGrid.x + y, centerGrid.y - x, isObstacle);
        setObstacle(centerGrid.x - y, centerGrid.y - x, isObstacle);
        
        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
    
    // 填充圆内部
    for (int dy = -radiusGrid; dy <= radiusGrid; dy++) {
        for (int dx = -radiusGrid; dx <= radiusGrid; dx++) {
            if (dx*dx + dy*dy <= radiusGrid*radiusGrid) {
                setObstacle(centerGrid.x + dx, centerGrid.y + dy, isObstacle);
            }
        }
    }
}

void AStarPathFinder::setLineObstacle(const Vector2D& start, const Vector2D& end, bool isObstacle) {
    GridPoint startGrid = worldToGrid(start);
    GridPoint endGrid = worldToGrid(end);
    
    // 使用Bresenham算法画线
    int dx = abs(endGrid.x - startGrid.x);
    int dy = abs(endGrid.y - startGrid.y);
    int sx = (startGrid.x < endGrid.x) ? 1 : -1;
    int sy = (startGrid.y < endGrid.y) ? 1 : -1;
    int err = dx - dy;
    
    int x = startGrid.x;
    int y = startGrid.y;
    
    while (true) {
        setObstacle(x, y, isObstacle);
        
        if (x == endGrid.x && y == endGrid.y) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

// 清除所有障碍物
void AStarPathFinder::clearObstacles() {
    memset(obstacleGrid, 0, sizeof(obstacleGrid));
}

// 私有辅助方法
AStarNode AStarPathFinder::createNode(const GridPoint& gridPos) {
    AStarNode node;
    node.gridPosition = gridPos;
    node.worldPosition = gridToWorld(gridPos);
    node.nodeType = NodeType::nt_normal;
    node.gCost = 0.0f;
    node.hCost = 0.0f;
    node.fCost = 0.0f;
    node.parentIndex = -1;
    node.isOpen = false;
    node.isClosed = false;
    return node;
}

float AStarPathFinder::calculatehCost(const GridPoint& from, const GridPoint& to) {
    // 使用欧几里得距离作为启发式函数
    float dx = static_cast<float>(to.x - from.x);
    float dy = static_cast<float>(to.y - from.y);
    return sqrtf(dx * dx + dy * dy) * 10.0f; // 乘以10使代价与移动代价匹配
}

bool AStarPathFinder::isValidGridPosition(const GridPoint& gridPos) {
    return isValidGridPosition(gridPos.x, gridPos.y);
}

bool AStarPathFinder::isValidGridPosition(int x, int y) {
    return (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight);
}

bool AStarPathFinder::isInClosedList(const GridPoint& gridPos) {
    for (int i = 0; i < closedIndex; i++) {
        if (closedList[i].gridPosition.x == gridPos.x && 
            closedList[i].gridPosition.y == gridPos.y) {
            return true;
        }
    }
    return false;
}

bool AStarPathFinder::buildPath(int targetNodeIndex) {
    // 直接从后往前填充pathPoints数组，避免最后反转
    pathPointCount = 0;
    
    // 从目标节点开始回溯
    int currentIndex = targetNodeIndex;
    
    // 计算路径长度
    int pathLength = 0;
    while (currentIndex != -1) {
        pathLength++;
        currentIndex = closedList[currentIndex].parentIndex;
    }
    
    // 检查路径长度是否超出限制
    if (pathLength > MAX_PATH_POINTS) {
        return false;
    }
    
    // 重新开始回溯，直接填充pathPoints
    currentIndex = targetNodeIndex;
    int insertIndex = pathLength - 1;  // 从后往前插入
    
    while (currentIndex != -1) {
        pathPoints[insertIndex--] = closedList[currentIndex].worldPosition;
        currentIndex = closedList[currentIndex].parentIndex;
    }
    
    pathPointCount = pathLength;
    return true;
}

// 路径规划
bool AStarPathFinder::findPath(const Vector2D& start, const Vector2D& goal) {
    // 设置起点和终点
    startPos = start;
    goalPos = goal;
    
    return findPath();
}

bool AStarPathFinder::findPath() {
    // 重置算法状态
    openList.clear();
    closedIndex = 0;
    currentIndex = 0;
    pathPointCount = 0;
    
    // 转换起点和终点为网格坐标
    GridPoint startGrid = worldToGrid(startPos);
    GridPoint goalGrid = worldToGrid(goalPos);
    
    // 验证起点和终点是否有效
    if (!isValidGridPosition(startGrid) || !isValidGridPosition(goalGrid)) {
        return false;
    }
    
    // 检查起点或终点是否是障碍物
    if (isObstacle(startGrid) || isObstacle(goalGrid)) {
        return false;
    }
    
    // 如果起点和终点是同一个点
    if (startGrid.x == goalGrid.x && startGrid.y == goalGrid.y) {
        pathPoints[0] = startPos;
        pathPointCount = 1;
        return true;
    }
    
    // 创建起始节点
    AStarNode startNode = createNode(startGrid);
    startNode.nodeType = NodeType::nt_start;
    startNode.gCost = 0.0f;
    startNode.hCost = calculatehCost(startGrid, goalGrid);
    startNode.fCost = startNode.gCost + startNode.hCost;
    startNode.isOpen = true;
    startNode.parentIndex = -1; // 起点的父节点索引为-1
    
    // 将起始节点加入开放列表
    openList.enqueue(startNode);
    
    // A*主循环
    while (!openList.isEmpty()) {
        // 从开放列表中取出f值最小的节点
        AStarNode currentNode = openList.dequeue();
        
        // 标记为已关闭
        currentNode.isOpen = false;
        currentNode.isClosed = true;
        
        // 将当前节点添加到关闭列表
        if (closedIndex < MAX_NODES) {
            closedList[closedIndex] = currentNode;
            currentIndex = closedIndex;
            closedIndex++;
        } else {
            return false; // 关闭列表已满
        }
        
        // 检查是否到达目标
        if (currentNode.gridPosition.x == goalGrid.x && currentNode.gridPosition.y == goalGrid.y) {
            return buildPath(currentIndex - 1); // 使用当前节点索引的前一个节点（目标节点）
        }
        
        // 检查所有邻居节点
        for (int i = 0; i < 8; i++) {
            GridPoint neighborGrid = {
                currentNode.gridPosition.x + gridDirections[i].x,
                currentNode.gridPosition.y + gridDirections[i].y
            };
            
            // 检查邻居是否有效且不是障碍物
            if (!isValidGridPosition(neighborGrid) || isObstacle(neighborGrid)) {
                continue;
            }
            
            // 检查邻居是否已在关闭列表中
            if (isInClosedList(neighborGrid)) {
                continue;
            }
            
            // 计算移动代价（直线为10，对角线为14）
            float moveCost = (i < 4) ? 10.0f : 14.0f;
            
            // 创建邻居节点
            AStarNode neighborNode = createNode(neighborGrid);
            neighborNode.gCost = currentNode.gCost + moveCost;
            neighborNode.hCost = calculatehCost(neighborGrid, goalGrid);
            neighborNode.fCost = neighborNode.gCost + neighborNode.hCost;
            neighborNode.parentIndex = currentIndex;
            neighborNode.isOpen = true;
            
            // 尝试更新开放列表中的节点，如果不存在则添加新节点
            if (!openList.updateNodeIfBetter(neighborNode)) {
                if (openList.isFull()) {
                    return false; // 开放列表已满
                }
                openList.enqueue(neighborNode);
            }
        }
    }
    
    // 没有找到路径
    return false;
}




//#include "PathA.h"

//bool AStarPathFinder::findPath()
//{
//    reset();

//    if(!isValidPosition(startPos) || !isValidPosition(goalPos))
//        {return false;}
//    if(startPos.x == goalPos.x && startPos.y == goalPos.y)
//        {
//            pathPoints[0] = startPos;
//            pathPointCount = 1;
//            return true;
//        }
//    AStarNode startNode = createNode(startPos);
//    startNode.nodeType = NodeType::nt_start;
//    startNode.gCost = 0.0f;
//    startNode.hCost = calculatehCost(startPos, goalPos);
//    startNode.fCost = startNode.gCost + startNode.hCost;
//    startNode.isOpen = true;
//    startNode.parentIndex = -1;// 起点的父节点索引为-1

//    openList.enqueue(startNode);
//    while(!openList.isEmpty())
//    {
//        AStarNode currentNode = openList.dequeue();

//        currentNode.isOpen = false;
//        currentNode.isClosed = true;

//        //将当前节点加入到关闭列表中也就是我们的路径点中去
//        while(closedList[closedIndex].position.x != 0 || closedList[closedIndex].position.y != 0) {
//            closedIndex++;
//            if(closedIndex >= 1000) break; // 防止溢出
//        }
//        if(closedIndex < 1000) {
//            closedList[closedIndex] = currentNode;
//            currentIndex = closedIndex;
//            closedIndex++;
//        }

//        if(currentNode.position.x == goalPos.x && currentNode.position.y == goalPos.y)
//        {
//            if(closedIndex<1000)
//            {
//                closedList[closedIndex] = currentNode;
//                int targetNodeIndex = closedIndex;  // 记录目标节点索引
//                closedIndex++;
//                return buildPath(targetNodeIndex);  

//            }
//            return false;            
//        }
//        for(int i = 0; i < 8; i++)
//        {
//            Vector2D neighborPos = currentNode.position + directions[i];
//            if(!isValidPosition(neighborPos)||isInClosedList(neighborPos))
//                {continue;}
//                float moveCost = (i < 4) ? 10.f : 14.f;
//                AStarNode neighborNode = createNode(neighborPos);
//                neighborNode.gCost = currentNode.gCost + moveCost;
//                neighborNode.hCost = calculatehCost(neighborPos, goalPos);
//                neighborNode.fCost = neighborNode.gCost + neighborNode.hCost;
//                neighborNode.parentIndex = currentIndex;
//                neighborNode.isOpen = true;
//                
//                // 尝试更新开放列表中的节点，如果不存在则添加新节点
//                if (!openList.updateNodeIfBetter(neighborNode)) {
//                     openList.enqueue(neighborNode);
//    }
//    }
//    }
//    
//    return false;
//}

//void AStarPathFinder::reset()
//{
//     openList.clear();  // 清空开放列表
//     closedIndex = 0;  // 重置关闭列表索引（相当于清空）
//     pathPointCount = 0;  // 清空路径
//     // 重置关闭列表中所有节点的状态
//    for(int i = 0; i < 1000; i++) {
//        closedList[i].isOpen = false;
//        closedList[i].isClosed = false;
//        closedList[i].position = {0.0f, 0.0f};  // 重置位置
//    }
//}

//bool AStarPathFinder::buildPath(int targetNodeIndex) {
//    // 直接从后往前填充pathPoints数组，避免最后反转
//    pathPointCount = 0;
//    
//    // 从目标节点开始回溯
//    int currentIndex = targetNodeIndex;
//    
//    // 计算路径长度
//    int pathLength = 0;
//    while (currentIndex != -1) {
//        pathLength++;
//        currentIndex = closedList[currentIndex].parentIndex;
//    }
//    
//    // 检查路径长度是否超出限制
//    if (pathLength > MAX_PATH_POINTS) {
//        return false;
//    }
//    
//    // 重新开始回溯，直接填充pathPoints
//    currentIndex = targetNodeIndex;
//    int insertIndex = pathLength - 1;  // 从后往前插入
//    
//    while (currentIndex != -1) {
//        pathPoints[insertIndex--] = closedList[currentIndex].position;
//        currentIndex = closedList[currentIndex].parentIndex;
//    }
//    
//    pathPointCount = pathLength;
//    return true;
//}

//// 初始化障碍物网格
//void AStarPathFinder::initializeObstacleGrid(float size, const Vector2D& origin) {
//    cellSize = size;
//    gridOrigin = origin;
//    
//    // 清空障碍物网格
//    for (int y = 0; y < GRID_HEIGHT; y++) {
//        for (int x = 0; x < GRID_WIDTH; x++) {
//            obstacleGrid[y][x] = false;
//        }
//    }
//}
