#ifndef PATHA_H
#define PATHA_H

#include "APP_Vector2D.h"
#include <cstdint>

// 网格点结构体，使用Vector2D表示网格坐标（原点位于左下�?�?
typedef Vector2D GridPoint;

// 节点类型枚举
enum class NodeType {
    nt_start,    // 起点
    nt_goal,     // 终点
    nt_normal    // �?��节�?
};

// A*节点结构�?
struct AStarNode {
    GridPoint gridPosition;    // 网格坐标
    Vector2D worldPosition;     // 世界坐标（可选，用于最终路径输出）
    NodeType nodeType;          // 节点类型
    float gCost;                // 从起点到当前节点的实际代�?
    float hCost;                // 从当前节点到终点的启发式代价
    float fCost;                // f = g + h，总代�?
    int parentIndex;            // 父节点在closedList�?��索引
    bool isOpen;                // �?��在开放列表中
    bool isClosed;              // �?��在关�?��表中
};

// 固定大小的优先队列（用于开放列�?��
class FixedSizePriorityQueue {
private:
    static const int MAX_SIZE = 1000;
    AStarNode nodes[MAX_SIZE];
    int count;

public:
    FixedSizePriorityQueue() : count(0) {}
    
    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count >= MAX_SIZE; }
    
    void clear() { count = 0; }
    
    bool enqueue(const AStarNode& node) {
        if (isFull()) return false;
        
        // 插入节点
        int i = count;
        nodes[i] = node;
        count++;
        
        // 上浮调整
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (nodes[i].fCost >= nodes[parent].fCost) break;
            
            // 交换节点
            AStarNode temp = nodes[i];
            nodes[i] = nodes[parent];
            nodes[parent] = temp;
            
            i = parent;
        }
        
        return true;
    }
    
    AStarNode dequeue() {
        if (isEmpty()) return AStarNode(); // 返回空节�?
        
        AStarNode result = nodes[0];
        count--;
        
        // 将最后一�?��点移到顶�?
        nodes[0] = nodes[count];
        
        // 下沉调整
        int i = 0;
        while (true) {
            int left = 2 * i + 1;
            int right = 2 * i + 2;
            int smallest = i;
            
            if (left < count && nodes[left].fCost < nodes[smallest].fCost) {
                smallest = left;
            }
            
            if (right < count && nodes[right].fCost < nodes[smallest].fCost) {
                smallest = right;
            }
            
            if (smallest == i) break;
            
            // 交换节点
            AStarNode temp = nodes[i];
            nodes[i] = nodes[smallest];
            nodes[smallest] = temp;
            
            i = smallest;
        }
        
        return result;
    }
    
    // 如果新节点的fCost更小，则更新队列�?��同位�?��节点
    bool updateNodeIfBetter(const AStarNode& newNode) {
        for (int i = 0; i < count; i++) {
            if (nodes[i].gridPosition.x == newNode.gridPosition.x && 
                nodes[i].gridPosition.y == newNode.gridPosition.y) {
                
                if (newNode.fCost < nodes[i].fCost) {
                    nodes[i] = newNode;
                    
                    // 重新调整�?
                    int current = i;
                    // 上浮调整
                    while (current > 0) {
                        int parent = (current - 1) / 2;
                        if (nodes[current].fCost >= nodes[parent].fCost) break;
                        
                        AStarNode temp = nodes[current];
                        nodes[current] = nodes[parent];
                        nodes[parent] = temp;
                        
                        current = parent;
                    }
                    
                    // 下沉调整
                    while (true) {
                        int left = 2 * current + 1;
                        int right = 2 * current + 2;
                        int smallest = current;
                        
                        if (left < count && nodes[left].fCost < nodes[smallest].fCost) {
                            smallest = left;
                        }
                        
                        if (right < count && nodes[right].fCost < nodes[smallest].fCost) {
                            smallest = right;
                        }
                        
                        if (smallest == current) break;
                        
                        AStarNode temp = nodes[current];
                        nodes[current] = nodes[smallest];
                        nodes[smallest] = temp;
                        
                        current = smallest;
                    }
                    
                    return true;
                }
                return true; // 找到节点但不更新
            }
        }
        return false; // 没有找到相同位置的节�?
    }
};

class AStarPathFinder {
private:
    // 网格参数 - 使用静态常量定义最大网格大�?
    static const int MAX_GRID_WIDTH = 100;
    static const int MAX_GRID_HEIGHT = 100;
    
    int gridWidth;              // 实际使用的网格�?�?
    int gridHeight;             // 实际使用的网格高�?
    float cellSize;             // 每个网格单元的大小（世界坐标单位�?
    Vector2D gridOrigin;        // 网格原点在世界坐标系�?��位置（左下�?�?
    
    // 障�?物网�? - 使用静态数�?
    bool obstacleGrid[MAX_GRID_WIDTH * MAX_GRID_HEIGHT];  // true表示有障碍物
    
    // A*算法数据结构
    static const int MAX_PATH_POINTS = 500;  // 最大路径点�?
    static const int MAX_NODES = 1000;       // 最大节点数
    
    FixedSizePriorityQueue openList;         // 开放列�?
    AStarNode closedList[MAX_NODES];         // 关闭列表
    int closedIndex;                         // 关闭列表�?��当前索引
    int currentIndex;                        // 当前处理的节点索�?
    
    // �?��结果
    Vector2D pathPoints[MAX_PATH_POINTS];    // �?��点数组（世界坐标�?
    int pathPointCount;                      // �?��点数�?
    
    // 起点和终点（世界坐标�?
    Vector2D startPos;
    Vector2D goalPos;
    
    // 8�?��向的网格偏移（上、下、左、右、左上、右上、左下、右下）
    const GridPoint gridDirections[8] = {
        {0, 1},   // �?
        {0, -1},  // �?
        {-1, 0},  // �?
        {1, 0},   // �?
        {-1, 1},  // 左上
        {1, 1},   // 右上
        {-1, -1}, // 左下
        {1, -1}   // 右下
    };
    
    // 私有辅助方法
    AStarNode createNode(const GridPoint& gridPos);
    float calculatehCost(const GridPoint& from, const GridPoint& to);
    bool isValidGridPosition(const GridPoint& gridPos);
    bool isValidGridPosition(int x, int y);
    bool isInClosedList(const GridPoint& gridPos);
    bool buildPath(int targetNodeIndex);

public:
    // 构造函�?
    AStarPathFinder(int width = 100, int height = 100, float size = 1.0f, const Vector2D& origin = {0.0f, 0.0f});
    
    // 坐标�?��
    GridPoint worldToGrid(const Vector2D& worldPos);
    Vector2D gridToWorld(const GridPoint& gridPos);
    
    // 网格管理
    void initializeGrid(int width, int height, float cellSize, const Vector2D& origin);
    void clearGrid();
    
    // 障�?物�?�?
    void setObstacle(int gridX, int gridY, bool isObstacle);
    void setObstacle(const GridPoint& gridPos, bool isObstacle);
    bool isObstacle(int gridX, int gridY);
    bool isObstacle(const GridPoint& gridPos);
    
    // 便捷障�?物�?�?���?
    void setPointObstacle(const Vector2D& worldPos, bool isObstacle);
    void setRectangleObstacle(const Vector2D& minCorner, const Vector2D& maxCorner, bool isObstacle);
    void setCircleObstacle(const Vector2D& center, float radius, bool isObstacle);
    void setLineObstacle(const Vector2D& start, const Vector2D& end, bool isObstacle);
    
    // 清除所有障碍物
    void clearObstacles();
    
    // �?��规划
    bool findPath(const Vector2D& start, const Vector2D& goal);
    bool findPath();  // 使用预�?的起点和终点
    
    // 获取�?��结果
    const Vector2D* getPathPoints() const { return pathPoints; }
    int getPathPointCount() const { return pathPointCount; }
    
    
};

#endif // PATHA_H

//#ifndef __PATH_A_H__
//#define __PATH_A_H__

//#include "APP_Vector2D.h"
//#include <vector>
//#include <queue>
//#include <unordered_set>
//#include <stdint.h> 
//#include <cmath>

//typedef enum{
//    A_PATH_STATUS_IDLE=0,
//    A_PATH_STATUS_RUNNING=1,
//    A_PATH_STATUS_FINISHED=2,
//    A_PATH_STATUS_ERROR=3,
//}A_Path_Status;

//// 地图节点类型 
//enum class NodeType{ 
//    nt_free,        // �?��区域 
//    nt_obstacle,    // 障�?�? 
//    nt_start,       // 起点 
//    nt_goal,        // 终点 
//};

//// �?��式类型枚�?
//enum HeuristicType
//{
//    EUCLIDEAN,
//    MANHATTAN,
//    DIAGONAL
//};

//// A*节点结构 
//struct AStarNode { 
//    Vector2D position;  // 节点位置 
//    float gCost;        // 从起点到当前节点的实际代�? 
//    float hCost;        // 从当前节点到终点的启发式估�?代价 
//    float fCost;        // 总代�? f = g + h    
//    bool isOpen;        // �?��在开放列表中 
//    bool isClosed;      // �?��在关�?��表中 
//    NodeType nodeType;  // 节点类型
//    bool operator<(const AStarNode& other) const {
//        return fCost < other.fCost; 
//    } 
//    int parentIndex;        // 父节点索�?
//}; 

////优先队列的实现，使用静态分�?
//// 固定大小优先队列
//template<typename T, int Capacity> 
//class FixedSizePriorityQueue { 
//private: 
//    T data[Capacity];       // 存储队列元素的数�? 
//    int size;               // 当前队列大小 
//    
//    // 堆调整函�? 
//    void heapifyUp(int index) { 
//        while (index > 0) { 
//            int parent = (index - 1) / 2; 
//            if (data[index] < data[parent]) { 
//                // 交换父节点和当前节点 
//                T temp = data[parent]; 
//                data[parent] = data[index]; 
//                data[index] = temp; 
//                index = parent; 
//            } else { 
//                break; 
//            } 
//        } 
//    } 
//    //使用的方法，如果你有新加入的节点数据，这样子他就�?��重新调整这个获取最小元素的方法
//    //void addToOpenList(const AStarNode& node) {
//    //   data[size] = node;
//    //   heapifyUp(size);
//    //   size++;
//    // }
//  
//    void heapifyDown(int index) { 
//        while (true) { 
//            int leftChild = 2 * index + 1; 
//            int rightChild = 2 * index + 2; 
//            int smallest = index; 
//            
//            // �??比较逻辑：应该找最小的元素
//            if (leftChild < size && data[leftChild] < data[smallest]) { 
//                smallest = leftChild; 
//            } 
//            
//            if (rightChild < size && data[rightChild] < data[smallest]) { 
//                smallest = rightChild; 
//            } 
//            
//            if (smallest != index) { 
//                // 交换当前节点和最小子节点 
//                T temp = data[index]; 
//                data[index] = data[smallest]; 
//                data[smallest] = temp; 
//                index = smallest; 
//            } else { 
//                break; 
//            } 
//        } 
//    } 
//public: 
//    FixedSizePriorityQueue() : size(0) {} 
//    
//    // 检查队列是否为�? 
//    bool isEmpty() const { return size == 0; } 
//    
//    // 检查队列是否已�? 
//    bool isFull() const { return size == Capacity; } 
//    
//    // 返回队列�?��元素数量 
//    int queueSize() const { return size; } 
//    
//    // 入队操作 
//    bool enqueue(const T& item) { 
//        if (isFull()) { 
//            return false; // 队列已满，入队失�? 
//        } 
//        
//        data[size] = item; 
//        heapifyUp(size); 
//        size++; 
//        return true; 
//    } 
//    
//    // 出队操作（获取最小元素） 
//    bool dequeue(T& item) { 
//        if (isEmpty()) { 
//            return false; // 队列为空，出队失�? 
//        } 
//        
//        item = data[0]; 
//        data[0] = data[size - 1]; 
//        size--; 
//        heapifyDown(0); 
//        return true; 
//    } 
//    
//    T dequeue(){
//         T item;
//         if (dequeue(item)) {
//           return item;
//         }
//         return T(); // 返回默�?构造的对象
//    }
//    // 查看队�?元素 
//    bool peek(T& item) const { 
//        if (isEmpty()) { 
//            return false; // 队列为空，查看失�? 
//        } 
//        
//        item = data[0]; 
//        return true; 
//    } 
//    
//    // 清空队列 
//    void clear() { size = 0; } 
//     // 检查节点是否在开放列表中，若存在且新gCost更优则更�?

//     // 更新开放列表中节点的函�?
//    bool updateNodeIfBetter(const T& newNode) {
//        // 查找相同位置的节�?
//        for (int i = 0; i < size; i++) {
//            if (data[i].position.x == newNode.position.x && 
//                data[i].position.y == newNode.position.y) {
//                // 如果新路径更好，更新节点
//                if (newNode.gCost < data[i].gCost) {
//                    data[i].gCost = newNode.gCost;
//                    data[i].fCost = newNode.fCost;
//                    data[i].parentIndex = newNode.parentIndex;
//                    
//                    // 调整堆结�?
//                    heapifyUp(i);
//                    return true;
//                }
//                return false;
//            }
//        }
//        return false; // 没找到相同位�?��节点
//    }
//}; 

//class AStarPathFinder {
//public:
//    AStarPathFinder() {}
//    ~AStarPathFinder() {}
//    bool findPath();
//    void init(float width, float height){
//        mapWidth = width;
//        mapHeight = height;
//    }
//    const Vector2D* getPathPoints()
//    {
//        return pathPoints;
//    }
//    int getPathPointCount()
//    {
//        return pathPointCount;
//    }

//    // 网格障�?物�?理方�?
//    void initializeObstacleGrid(float size, const Vector2D& origin);
//    void setObstacle(int gridX, int gridY);
//    void removeObstacle(int gridX, int gridY);
//    void clearObstacles();
//    bool isObstacle(const Vector2D& position) const;
//    
//    // 规则障�?物�?�?
//    void addRectangleObstacle(int gridX, int gridY, int width, int height);
//    void addCircleObstacle(int centerX, int centerY, int radius);
//

//private:
//    FixedSizePriorityQueue<AStarNode, 100> openList;
//    AStarNode closedList[1000];
//    int closedIndex=0;
//    int currentIndex=0;
//    float Radius=0.5f;
//    Vector2D startPos;                  // 起点 
//    Vector2D goalPos;                   // 终点
//    float mapWidth;                     // 地图宽度
//    float mapHeight;                    // 地图高度
//    static const int MAX_PATH_POINTS = 300;
//    int pathPointCount;//�?��点数�?
//    Vector2D pathPoints[MAX_PATH_POINTS];

//    // 网格障�?物�?�?
//    static const int GRID_WIDTH = 100;   // �?���?��要调�?
//    static const int GRID_HEIGHT = 100;  // �?���?��要调�?
//    bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH];  // 障�?物网�?
//    float cellSize;  // 每个网格单元的实际尺寸（米）
//    Vector2D gridOrigin;  // 网格原点在世界坐标系�?��位置

//    void reset();
//    const Vector2D directions[8] = {
//        {0.0f, Radius}, {0.0f, -Radius}, {-Radius, 0.0f}, {Radius, -0.0f},
//        {-Radius, Radius}, {-Radius, -Radius}, {Radius, Radius}, {Radius, -Radius}
//    };//上下左右+斜�?
//    float calculatehCost(Vector2D a, Vector2D b) {
//        return abs(a.x - b.x) + abs(a.y - b.y);
//    }
//    
//    void setPos(Vector2D start, Vector2D goal){
//        startPos = start;
//        goalPos = goal;
//    }
//    bool isValidPosition(Vector2D pos)
//    {
//        return pos.x >= 0 && pos.x < mapWidth && pos.y >= 0 && pos.y < mapHeight;
//    }
//    void setObstacle(Vector2D pos,float length,float width)
//    {

//    }
//    // 检查节点是否在关闭列表�?
//    bool isInClosedList(Vector2D pos) {
//    for (int i = 0; i < closedIndex; i++) {  // 用closedIndex记录已存储的节点数，而非遍历整个数组
//        if (closedList[i].position.x == pos.x && closedList[i].position.y == pos.y) 
//        return true;
//    }
//    return false;
//    }
//    bool buildPath(int targetNodeIndex);
//    // 基�?节点创建函数 - �?��要位�?
//    AStarNode createNode(Vector2D pos) {
//        AStarNode node;
//        node.position = pos;
//        // 其他字�?使用默�?值，后续再�?�?
//        node.gCost = 0.0f;
//        node.hCost = 0.0f;
//        node.fCost = node.gCost + node.hCost;
//        node.isOpen = false;
//        node.isClosed = false;
//        node.nodeType = NodeType::nt_free;
//        node.parentIndex = -1;
//        return node;
//    }
//};

//#endif