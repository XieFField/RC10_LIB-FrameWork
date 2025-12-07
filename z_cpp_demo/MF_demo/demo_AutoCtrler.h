/**
 * @file AutoCtrler.h
 * @author XieFField
 * @brief 自动控制相关
 */


#ifndef AUTOCTRLER_H
#define AUTOCTRLER_H
#pragma once

#ifdef __cplusplus

#include <cstdint>
#include <cmath>
#include "demo_tool.h" // 确保你有这个文件，或者替换为 Point2D 定义
using std::sqrt;

// 计算最少步数 BFS
int BFS_Steps(int8_t startMap, int8_t goalMap);

namespace MF_AutoCtrler{

static constexpr int   MAP_COLS = 5;
static constexpr int   MAP_ROWS = 6;
static constexpr float CELL_M   = 1.2f;
static constexpr int   BFS_INF  = 30000;

extern const Point2D MapNum_RealPos[30];

typedef enum{
    Positive_X,
    Negative_X,
    Positive_Y,
    Negative_Y,
    NONE,
}Direction_E;

// 移除 static，允许外部链接
int8_t MFNum_TransforMapNum(int8_t MFNum);
int8_t MapNum_TransforMFNum(int8_t mapNum);

typedef struct{
    int8_t result1 = 0;
    int8_t result2 = 0;
    int8_t result3 = 0;
}RoadResult_S;

typedef struct{
    int8_t entranceMap = 0; // 补全缺失成员
    int8_t bestB1 = 0;
    int8_t bestBMF1 = 0;
    int8_t bestB2 = 0;
    int8_t bestBMF2 = 0;
    int8_t exitMap = 30;    // 移除 const，允许赋值
}PathNode_S;

RoadResult_S MFNum_ToRoadResult(int8_t MFNum);

// 移除 static
bool IsWalkable(int8_t map);
Point2D MapNum_ToMatrixPos(int8_t MapNum);

int8_t CR_ToMap(int8_t c, int8_t r);
void Map_ToCR(int8_t map, int8_t& c, int8_t& r);

// 移除 static
float euclid(Point2D a, Point2D b);

Point2D MapCenterWorld(int8_t map);

PathNode_S PathNodeResult_calc(Point2D robotPos, int8_t MF1, int8_t MF2);

RoadResult_S MFNum_ToCatchRoadResult(int8_t MFNum);

void get_MoveDiretion(Point2D robotPos, int8_t MF1, int8_t MF2, Direction_E Diresult[]);

float Get_ArmBaseTargetAngle(int8_t mapNum, Direction_E dir);

float Get_ArmWorldAngle(float chassis_yaw_deg, float gimbal_angle_deg);

}
#endif
#endif // AUTOCTRLER_H