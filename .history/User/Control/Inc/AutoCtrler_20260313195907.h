/**
 * @file AutoCtrler.h
 * @author XieFField
 * @brief 自动控制相关
 * @version 1.0
 *          优化MF_AutoCtrler，规范入口位置。
 *          采用右手系，Y轴为基准0度，逆时针旋转正方向
 */

#ifndef AUTOCTRLER_H
#define AUTOCTRLER_H
#pragma once

#ifdef __cplusplus

#include <cstdint>
#include <cmath>
#include "APP_tool.h"
#include <array>
#include "APP_Vector2D.h"
using std::sqrt;

namespace MF_AutoCtrler
{ // 梅花林中必经点位输出

    static constexpr int MAP_COLS = 5;
    static constexpr int MAP_ROWS = 6;
    static constexpr float CELL_M = 1.2f;
    static constexpr int BFS_INF = 30000;

    extern const Point2D MapNum_RealPos[30];

    typedef enum
    {
        Positive_X,
        Negative_X,

        Positive_Y,
        Negative_Y,

        NONE,
    } Direction_E;

    // 将梅花桩编号映射为梅花林方格地图所对应的编号。
    int8_t MFNum_TransforMapNum(int8_t MFNum);

    // 将梅花林方格地图编号映射为梅花桩编号。
    int8_t MapNum_TransforMFNum(int8_t mapNum);

    typedef struct
    {
        int8_t result1 = 0;
        int8_t result2 = 0;
        int8_t result3 = 0;
    } RoadResult_S;

    typedef struct
    {
        int8_t entranceMap;
        int8_t bestB1;   // 前一桩
        int8_t bestBMF1; // 正对桩
        int8_t bestB2;
        int8_t bestBMF2;
        int8_t exitMap = 26; // 固定出口
    } PathNode_S;            // 值为0就意味着没有这个节点

    // 求解梅花桩所有前一通道结果
    RoadResult_S MFNum_ToRoadResult(int8_t MFNum);
    static bool IsWalkable(int8_t map);
    // 求解方格的行列坐标
    static Point2D MapNum_ToMatrixPos_point(int8_t MapNum);
    
    Vector2D MapNum_ToMatrixPos(int8_t MapNum);
    Vector2D MapCenterWorld_Vector2D(int8_t map);

    // // 计算最优入口点
    // int8_t BestEntrance_calc(Point2D robotPos,
    //                          const RoadResult_S* B1,
    //                          const RoadResult_S* B2,
    //                          int8_t* outBestB1,
    //                          int8_t* outBestB2);
    // 行列转地图编号
    int8_t CR_ToMap(int8_t c, int8_t r);
    // 地图编号转行列
    void Map_ToCR(int8_t map, int8_t &c, int8_t &r);

    // 计算两点欧氏距离
    static float euclid(Point2D a, Point2D b);

    // 计算地图格子中心的世界坐标
    Point2D MapCenterWorld(int8_t map);

    // 计算路径节点结果
    PathNode_S PathNodeResult_calc(Point2D robotPos, int8_t MF1, int8_t MF2, int8_t EXIT = 26);

    RoadResult_S MFNum_ToCatchRoadResult(int8_t MFNum); // 求解拾取KFS时候所处通道 最多两解

    void get_MoveDiretion(Point2D robotPos,
                          int8_t MF1, int8_t MF2,
                          Direction_E Diresult[]);

    // 根据当前所在的地图格(bestB1)和行进方向计算出
    float Get_ArmBaseTargetAngle(int8_t mapNum, Direction_E dir);

    /**
     * @brief 计算机械臂在世界坐标系下的绝对角度
     * @param chassis_yaw_deg 底盘在世界系下的Yaw角 (度)
     * @param gimbal_angle_deg 机械臂云台相对于底盘的角度 (度)
     * @return float 机械臂在世界系下的角度 (度, 0度对应Y轴, 逆时针为正)
     */
    float Get_ArmWorldAngle(float chassis_yaw_deg, float gimbal_angle_deg);

    // 计算最少步数 BFS
    int BFS_Steps(int8_t startMap, int8_t goalMap);

/**
 * @brief 计算机械臂在世界坐标系下的绝对角度
 * @param chassis_yaw_deg 底盘在世界系下的Yaw角 (度)
 * @param gimbal_angle_deg 机械臂云台相对于底盘的角度 (度)
 * @return float 机械臂在世界系下的角度 (度, 0度对应Y轴, 逆时针为正)
 */
float Get_ArmWorldAngle(float chassis_yaw_deg, float gimbal_angle_deg);

/**
 * @brief 计算梅花林行进过程中，底盘在林道的yaw角，使得机械臂一端能贴靠梅花林
 *        由梅林上下左右四条通道，分别四个不同yaw角，可以通过既有的PathNode计算得到
 */
float Get_ChassisYawForArmAlign(int8_t targetKFS, int8_t B1, int8_t BMF1);

    // 获取BFS最短路径序列 (返回路径长度, -1表示缓冲区不足, 0表示不可达)
    int BFS_GetPath(int8_t startMap, int8_t goalMap, int8_t *outPath, int maxLen);
    /**
     * @brief 根据世界坐标判断所在的地图网格编号
     * @param pos 世界坐标点 (x, y)
     * @return int8_t 地图编号 (1-30), 如果超出范围返回 0
     */
    int8_t GetMapNumFromPos(Point2D pos);

    /**
      * @brief 判断当前坐标是否处于目标方格中心范围内
      * @param robotPos 机器人当前坐标
      * @param targetMap 目标方格编号 (1-30)
      * @param tolerance 容差范围，单位米
     */
    bool isInTargetMap(Point2D robotPos, int targetMap, float tolerance);
}
#endif
#endif // AUTOCTRLER_H