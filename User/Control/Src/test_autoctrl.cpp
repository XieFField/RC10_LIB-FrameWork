```cpp
#include "AutoCtrler.h"
#include <iostream>
#include <vector>

// 引入被测试的源文件，以便直接调用其静态函数
#include "AutoCtrler.cpp" 

// --- Helper Functions for Printing ---
void print_point(const Point2D& p, const char* name) {
    std::cout << name << ": (" << p.x << ", " << p.y << ")" << std::endl;
}

void print_path_node(const MF_AutoCtrler::PathNode_S& path, int mf1, int mf2) {
    std::cout << "----------------------------------------\n";
    std::cout << "Test Case: MF1=" << mf1 << ", MF2=" << mf2 << "\n";
    std::cout << "  - Entrance: " << static_cast<int>(path.entranceMap) << "\n";
    std::cout << "  - B1:       " << static_cast<int>(path.bestB1) << "\n";
    std::cout << "  - BMF1:     " << static_cast<int>(path.bestBMF1) << "\n";
    if (mf2 > 0) {
        std::cout << "  - B2:       " << static_cast<int>(path.bestB2) << "\n";
        std::cout << "  - BMF2:     " << static_cast<int>(path.bestBMF2) << "\n";
    }
    std::cout << "  - Exit:     " << static_cast<int>(path.exitMap) << "\n";
    std::cout << "----------------------------------------\n\n";
}

// --- Test Cases ---

// 测试1: 单个任务，机器人在场外下方
void test_single_task_from_outside() {
    Point2D robot_pos = {3.0, 0.5, 0}; // 场外下方
    int8_t mf1 = 5;
    int8_t mf2 = 0; // 无第二个任务

    MF_AutoCtrler::PathNode_S result = MF_AutoCtrler::PathNodeResult_calc(robot_pos, mf1, mf2);
    
    std::cout << "Running Test 1: Single Task (MF=" << static_cast<int>(mf1) << ") from Outside Below\n";
    print_point(robot_pos, "Robot Position");
    print_path_node(result, mf1, mf2);
}

// 测试2: 两个任务，机器人在场外上方
void test_double_task_from_outside() {
    Point2D robot_pos = {1.0, 10.0, 0}; // 场外上方
    int8_t mf1 = 1;
    int8_t mf2 = 12;

    MF_AutoCtrler::PathNode_S result = MF_AutoCtrler::PathNodeResult_calc(robot_pos, mf1, mf2);

    std::cout << "Running Test 2: Double Task (MF1=" << static_cast<int>(mf1) << ", MF2=" << static_cast<int>(mf2) << ") from Outside Above\n";
    print_point(robot_pos, "Robot Position");
    print_path_node(result, mf1, mf2);
}

// 测试3: 机器人在场内，靠近障碍物
void test_single_task_from_inside() {
    // 机器人位置在地图点1附近
    Point2D robot_pos = MF_AutoCtrler::MapNum_RealPos[0]; 
    robot_pos.x += 0.1; // 稍微偏移
    
    int8_t mf1 = 10;
    int8_t mf2 = 0;

    MF_AutoCtrler::PathNode_S result = MF_AutoCtrler::PathNodeResult_calc(robot_pos, mf1, mf2);

    std::cout << "Running Test 3: Single Task (MF=" << static_cast<int>(mf1) << ") from Inside\n";
    print_point(robot_pos, "Robot Position");
    print_path_node(result, mf1, mf2);
}

// 测试4: 验证BFS步数计算
void test_bfs() {
    std::cout << "Running Test 4: BFS Steps Calculation\n";
    
    int8_t start = 1;
    int8_t goal_walkable = 30;
    int8_t goal_unwalkable = 8; // 障碍点

    int steps1 = BFS_Steps(start, goal_walkable);
    std::cout << "  - Steps from " << static_cast<int>(start) << " to " << static_cast<int>(goal_walkable) << " (walkable): " << steps1 << "\n";

    int steps2 = BFS_Steps(start, goal_unwalkable);
    std::cout << "  - Steps from " << static_cast<int>(start) << " to " << static_cast<int>(goal_unwalkable) << " (unwalkable): ";
    if (steps2 >= MF_AutoCtrler::BFS_INF) {
        std::cout << "INF (unreachable)\n";
    } else {
        std::cout << steps2 << "\n";
    }
    std::cout << "----------------------------------------\n\n";
}


int main() {
    // 运行所有测试用例
    test_bfs();
    test_single_task_from_outside();
    test_double_task_from_outside();
    test_single_task_from_inside();

    return 0;
}

/*
// 编译指令 (g++):
// g++ -o test_autoctrl test_autoctrl.cpp -std=c++11
//
// 预期输出示例:
//
// Running Test 4: BFS Steps Calculation
//   - Steps from 1 to 30 (walkable): 9
//   - Steps from 1 to 8 (unwalkable): INF (unreachable)
// ----------------------------------------
//
// Running Test 1: Single Task (MF=5) from Outside Below
// Robot Position: (3, 0.5)
// ----------------------------------------
// Test Case: MF1=5, MF2=0
//   - Entrance: 3
//   - B1:       10
//   - BMF1:     15
//   - Exit:     30
// ----------------------------------------
//
// Running Test 2: Double Task (MF1=1, MF2=12) from Outside Above
// Robot Position: (1, 10)
// ----------------------------------------
// Test Case: MF1=1, MF2=12
//   - Entrance: 26
//   - B1:       1
//   - BMF1:     6
//   - B2:       25
//   - BMF2:     20
//   - Exit:     30
// ----------------------------------------
//
// Running Test 3: Single Task (MF=10) from Inside
// Robot Position: (0.7, 2.6)
// ----------------------------------------
// Test Case: MF1=10, MF2=0
//   - Entrance: 0
//   - B1:       20
//   - BMF1:     15
//   - Exit:     30
// ----------------------------------------
*/
```
