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
#include "APP_tool.h"
using std::sqrt;

// 计算最少步数 BFS
int BFS_Steps(int8_t startMap, int8_t goalMap);

//梅花林自动控制
namespace MF_AutoCtrler{

static constexpr int   MAP_COLS = 5;
static constexpr int   MAP_ROWS = 6;
static constexpr float CELL_M   = 1.2f;
static constexpr int   BFS_INF  = 30000;

const Point2D MapNum_RealPos[30] = {
    {0.6, 2.6, 0}, {1.8, 2.6, 0}, {3.0, 2.6, 0}, {4.2, 2.6, 0}, {5.4, 2.6, 0},
    {0.6, 3.8, 0}, {1.8, 3.8, 0}, {3.0, 3.8, 0}, {4.2, 3.8, 0}, {5.4, 3.8, 0},
    {0.6, 5.0, 0}, {1.8, 5.0, 0}, {3.0, 5.0, 0}, {4.2, 5.0, 0}, {5.4, 5.0, 0},
    {0.6, 6.2, 0}, {1.8, 6.2, 0}, {3.0, 6.2, 0}, {4.2, 6.2, 0}, {5.4, 6.2, 0},
    {0.6, 7.4, 0}, {1.8, 7.4, 0}, {3.0, 7.4, 0}, {4.2, 7.4, 0}, {5.4, 7.4, 0},
    {0.6, 8.6, 0}, {1.8, 8.6, 0}, {3.0, 8.6, 0}, {4.2, 8.6, 0}, {5.4, 8.6, 0}
};


// 将梅花桩编号映射为梅花林方格地图所对应的编号。
static int8_t MFNum_TransforMapNum(int8_t MFNum);

// 将梅花林方格地图编号映射为梅花桩编号。
static int8_t MapNum_TransforMFNum(int8_t mapNum);

typedef struct{
	int8_t result1 = 0;
	int8_t result2 = 0;
	int8_t result3 = 0;
}RoadResult_S;

typedef struct{
    int8_t entranceMap;
    int8_t bestB1;     //前一桩
    int8_t bestBMF1;   //正对桩
    int8_t bestB2;
    int8_t bestBMF2;
    const int8_t exitMap = 30; //固定出口
}PathNode_S;


// 求解梅花桩所有前一通道结果
RoadResult_S MFNum_ToRoadResult(int8_t MFNum);
static bool IsWalkable(int8_t map);
// 求解方格的行列坐标
static Point2D MapNum_ToMatrixPos(int8_t MapNum);

// // 计算最优入口点
// int8_t BestEntrance_calc(Point2D robotPos,
//                          const RoadResult_S* B1,
//                          const RoadResult_S* B2,
//                          int8_t* outBestB1,
//                          int8_t* outBestB2);
// 行列转地图编号
int8_t CR_ToMap(int8_t c, int8_t r);
// 地图编号转行列
void Map_ToCR(int8_t map, int8_t& c, int8_t& r);

// 计算两点欧氏距离
static float euclid(Point2D a, Point2D b);

// 计算地图格子中心的世界坐标
static Point2D MapCenterWorld(int8_t map);

// 计算路径节点结果
PathNode_S PathNodeResult_calc(Point2D robotPos,
                                 int8_t MF1, int8_t MF2);

RoadResult_S MFNum_ToCatchRoadResult(int8_t MFNum); //求解拾取KFS时候所处通道 最多两解
}
#endif
#endif // AUTOCTRLER_H