#include "AutoCtrler.h"
// #include <iostream>

#include <iostream>

namespace MF_AutoCtrler
{

const Point2D MapNum_RealPos[30] = {
{0.6, 2.6, 0}, {1.8, 2.6, 0}, {3.0, 2.6, 0}, {4.2, 2.6, 0}, {5.4, 2.6, 0}, 
{0.6, 3.8, 0}, {1.8, 3.8, 0}, {3.0, 3.8, 0}, {4.2, 3.8, 0}, {5.4, 3.8, 0}, 
{0.6, 5.0, 0}, {1.8, 5.0, 0}, {3.0, 5.0, 0}, {4.2, 5.0, 0}, {5.4, 5.0, 0}, 
{0.6, 6.2, 0}, {1.8, 6.2, 0}, {3.0, 6.2, 0}, {4.2, 6.2, 0}, {5.4, 6.2, 0}, 
{0.6, 7.4, 0}, {1.8, 7.4, 0}, {3.0, 7.4, 0}, {4.2, 7.4, 0}, {5.4, 7.4, 0}, 
{0.6, 8.6, 0}, {1.8, 8.6, 0}, {3.0, 8.6, 0}, {4.2, 8.6, 0}, {5.4, 8.6, 0}};

/**
 * @brief 判断当前坐标是否处于目标方格中心范围内
 * @param robotPos 机器人当前坐标
 * @param targetMap 目标方格编号 (1-30)
 * @param tolerance 容差范围，单位米
 */

bool isInTargetMap(Point2D robotPos, int targetMap, float tolerance)
{
    if(targetMap < 1 || targetMap > 30)
    {
        return false; // 无效的目标方格编号
    }
    Point2D map_center = MapNum_RealPos[targetMap - 1];
    
    if (robotPos.x >= map_center.x - tolerance && robotPos.x <= map_center.x + tolerance &&
        robotPos.y >= map_center.y - tolerance && robotPos.y <= map_center.y + tolerance)
    {
        return true; // 在目标格内
    }
    else
        return false; // 不在目标格内
}




void get_MoveDiretion(Point2D robotPos,
                        int8_t MF1, int8_t MF2,
                        Direction_E Diresult[])
{
    PathNode_S path = PathNodeResult_calc(robotPos, MF1, MF2);

    int8_t bestB1_c_, bestB1_r_, // 列 行
        bestB2_c_, bestB2_r_,
        bestBMF1_c_, bestBMF1_r_,
        bestBMF2_c_, bestBMF2_r_;

    Direction_E result_[2] = {NONE, NONE};

    Map_ToCR(path.bestB1, bestB1_c_, bestB1_r_);
    Map_ToCR(path.bestB2, bestB2_c_, bestB2_r_);
    Map_ToCR(path.bestBMF1, bestBMF1_c_, bestBMF1_r_);
    Map_ToCR(path.bestBMF2, bestBMF2_c_, bestBMF2_r_);

    if (MF1 != 0 && MF2 != 0)
    {
        if (bestB1_c_ == bestBMF1_c_) // 同列不同行
        {
            if (bestB1_r_ < bestBMF1_r_)
                result_[0] = Positive_Y;
            else
                result_[0] = Negative_Y;
        }
        else if (bestB1_r_ == bestBMF1_r_) // 同行不同列
        {
            if (bestB1_c_ < bestBMF1_c_)
                result_[0] = Positive_X;
            else
                result_[0] = Negative_X;
        }
        else
            result_[0] = NONE;

        if (bestB2_c_ == bestBMF2_c_) // 同列不同行
        {
            if (bestB2_r_ < bestBMF2_r_)
                result_[1] = Positive_Y;
            else
                result_[1] = Negative_Y;
        }
        else if (bestB2_r_ == bestBMF2_r_) // 同行不同列
        {
            if (bestB2_c_ < bestBMF2_c_)
                result_[1] = Positive_X;
            else
                result_[1] = Negative_X;
        }
        else
            result_[1] = NONE;
    }

    else
    {
        if (MF1 != 0)
        {
            if (bestB1_c_ == bestBMF1_c_) // 同行不同列
            {
                if (bestB1_r_ < bestBMF1_r_)
                    result_[0] = Positive_Y;
                else
                    result_[0] = Negative_Y;
            }
            else if (bestB1_r_ == bestBMF1_r_) // 同列不同航
            {
                if (bestB1_c_ < bestBMF1_c_)
                    result_[0] = Positive_X;
                else
                    result_[0] = Negative_X;
            }
            else
                result_[0] = NONE;
        }
        else
            result_[0] = NONE;

        if (MF2 != 0)
        {
            if (bestB2_c_ == bestBMF2_c_) // 同行不同列
            {
                if (bestB2_r_ < bestBMF2_r_)
                    result_[1] = Positive_Y;
                else
                    result_[1] = Negative_Y;
            }
            else if (bestB2_r_ == bestBMF2_r_) // 同列不同航
            {
                if (bestB2_c_ < bestBMF2_c_)
                    result_[1] = Positive_X;
                else
                    result_[1] = Negative_X;
            }
            else
                result_[1] = NONE;
        }
        else
            result_[1] = NONE;
    }

    Diresult[0] = result_[0];
    Diresult[1] = result_[1];
}   


float Get_ChassisYawForArmAlign(int8_t targetKFS, int8_t B1, int8_t BMF1)
{
    int8_t c1, r1, c2, r2;
    Map_ToCR(B1, c1, r1);
    Map_ToCR(BMF1, c2, r2);

    float target_yaw = 0.0f;

    /**
     * 1. 左侧 targetyaw = -180,
     * 2. 右侧 targetyaw = 0
     * 3. 上侧 targetyaw = 90
     * 4. 下侧 targetyaw = -90
     */

    //上下侧时候，同行不同列 即走x方向
    if(r1 == r2)
    {
        if(r1 == 1 && r2 ==1) //下侧
            target_yaw = -90.0f;
        else if (r1 ==6 && r2 ==6) //上侧
            target_yaw = 90.0f;
    }

    //左右侧时候， 同列不同行 即走y方向
    if(c1 == c2)
    {
        if(c1 ==1 && c2 ==1) //左侧
            target_yaw = 180.0f;
        else if (c1 ==5 && c2 ==5) //右侧
            target_yaw = 0.0f;
    }
    
    return target_yaw;
}



//依旧屎上堆屎
/**
 * @brief 根据当前所在的地图格(bestB1)和行进方向，计算机械臂初始朝向；
 * @param mapNum 输入bestB的地图编号
 * @param dir 机械臂行进方向
 */
float Get_ArmBaseTargetAngle(int8_t mapNum, Direction_E dir)
{
    int8_t c, r;
    Map_ToCR(mapNum, c, r);

    float tar = 0.0f;

    switch(dir)
    {
        case Positive_Y:
        {
            if (c == 1) // 左侧
                tar = 180.0f;

            else if (c == 5) // 右侧
                tar = 0.0f;

            break;
        }

        case Negative_Y:
        {
            if (c == 1) // 左侧
                tar = 0.0f;

            else if (c == 5) // 右侧
                tar = 180.0f;

            break;
        }

        case Positive_X:
        {
            if (r == 1) // 下侧
                tar = 0.0f;
            else if (r == 6) // 上侧
                tar = 180.0f;

            break;
        }

        case Negative_X:
        {
            if (r == 1) // 下侧
                tar = 180.0f;
            else if (r == 6) // 上侧
                tar = 0.0f;

            break;
        }

    }
        return tar;
}

float Get_ArmWorldAngle(float chassis_yaw_deg, float gimbal_angle_deg)
{
    float arm_world_angle = chassis_yaw_deg + gimbal_angle_deg;
    // 归一化到 [0, 360)
    while (arm_world_angle > 360.0f)
    {
        arm_world_angle -= 360.0f;
    }
    while (arm_world_angle < 0.0f)
    {
        arm_world_angle += 360.0f;
    }
    return arm_world_angle;
}

// 行列转地图编号
int8_t CR_ToMap(int8_t c, int8_t r)
{
    return (int8_t)((r - 1) * MAP_COLS + c);
}

// 撞墙判断(梅花桩)
static bool IsWalkable(int8_t map)
{
    if (map < 1 || map > 30)
        return false;
    // int8_t mf = MapNum_TransforMFNum(map);
    // // mf==-1 → 通道；否则为梅花桩格（障碍）
    // return (mf == -1);

    int8_t c, r;
    Map_ToCR(map, c, r);
    // 中心区域 c=2..4 且 r=2..5 为不可走
    return !(c >= 2 && c <= 4 && r >= 2 && r <= 5);
}

// 地图编号转行列
void Map_ToCR(int8_t map, int8_t &c, int8_t &r)
{
    r = (int8_t)((map - 1) / MAP_COLS + 1);
    c = (int8_t)((map - 1) % MAP_COLS + 1);
}

int8_t MFNum_TransforMapNum(int8_t MFNum) // 将梅花桩编号转换为梅花林方格地图编号
{
    if (MFNum < 1 || MFNum > 12)
        return -1;

    return MFNum + 6 + 2 * (static_cast<int8_t>((MFNum - 1) / 3.0));
}

int8_t MapNum_TransforMFNum(int8_t mapNum) // 将梅花林方格地图编号转换为梅花桩编号
{
    int8_t MFNum_ = mapNum - 6 - 2 * ((mapNum - 7) / 3);
    if (MFNum_ < 1 || MFNum_ > 12)
        return -1;
    return MFNum_;
}

static bool IsAdjacent4(int8_t a, int8_t b)
{
    if (a < 1 || a > 30 || b < 1 || b > 30)
        return false;

    int8_t c1, r1, c2, r2;
    Map_ToCR(a, c1, r1);
    Map_ToCR(b, c2, r2);

    int dc = c1 - c2;
    if (dc < 0)
        dc = -dc;
    int dr = r1 - r2;
    if (dr < 0)
        dr = -dr;

    return (dc + dr) == 1;
}

RoadResult_S MFNum_ToCatchRoadResult(int8_t MFNum) // 求解拾取KFS时候所处通道 最多两解
{
    RoadResult_S result_ = {0, 0, 0};
    if (MFNum < 1 || MFNum > 12)
        return result_;

    int8_t mapNum = MFNum_TransforMapNum(MFNum);

    int8_t candidate[4];

    /*
            3
        1   t   2
            0
    */

    candidate[0] = mapNum - 5;
    candidate[1] = mapNum - 1;
    candidate[2] = mapNum + 1;
    candidate[3] = mapNum + 5;

    for (int i = 0; i < 4; i++)
    {
        if (candidate[i] < 1 || candidate[i] > 30)
        {
            candidate[i] = 0;
            continue;
        }

        // 过滤非通道
        if (!IsWalkable(candidate[i]))
        {
            candidate[i] = 0;
            continue;
        }
    }

    int8_t validResults[3] = {0};
    int validCount = 0;

    // 过滤0
    for (int i = 0; i < 4 && validCount < 3; i++)
    {
        if (candidate[i] != 0)
            validResults[validCount++] = candidate[i];
    }

    for (int i = 0; i < validCount - 1; i++)
    {
        for (int j = 0; j < validCount - 1 - i; j++)
        {
            if (validResults[j] > validResults[j + 1])
            {
                int8_t temp = validResults[j];
                validResults[j] = validResults[j + 1];
                validResults[j + 1] = temp;
            }
        }
    }

    result_.result1 = validResults[0];
    result_.result2 = validResults[1];
    result_.result3 = validResults[2];

    return result_;
}

RoadResult_S MFNum_ToRoadResult(int8_t MFNum) // 求解梅花桩所有前一通道结果(进入通道、开启预判)
{
    RoadResult_S result = {0, 0, 0};
    if (MFNum < 1 || MFNum > 12)
        return result;

    int8_t mapNum_ = MFNum_TransforMapNum(MFNum);

    /*
        2   3
            t
        0   1
    */

    int8_t candidate[4];
    candidate[0] = mapNum_ - 6;
    candidate[1] = mapNum_ - 4;
    candidate[2] = mapNum_ + 4;
    candidate[3] = mapNum_ + 6;

    for (int i = 0; i < 4; i++)
    {
        if (candidate[i] < 1 || candidate[i] > 30)
        {
            candidate[i] = 0;
            continue;
        }

        // for(int j = 1; j <=12; j++)
        // {
        //     int8_t mf_map = MFNum_TransforMapNum(j);
        //     if(candidate[i] == mf_map)
        //         candidate[i] = 0;

        // }

        if (IsWalkable(candidate[i]) == false)
        {
            candidate[i] = 0;
            continue;
        }
    }

    int8_t validResults[3] = {0};
    int validCount = 0;
    // 过滤0
    for (int i = 0; i < 4 && validCount < 3; i++)
    {
        if (candidate[i] != 0)
            validResults[validCount++] = candidate[i];
    }

    // 排列
    for (int i = 0; i < validCount - 1; i++)
    {
        for (int j = 0; j < validCount - 1 - i; j++)
        {
            if (validResults[j] > validResults[j + 1])
            {
                int8_t temp = validResults[j];
                validResults[j] = validResults[j + 1];
                validResults[j + 1] = temp;
            }
        }
    }

    result.result1 = validResults[0];
    result.result2 = validResults[1];
    result.result3 = validResults[2];

    return result;
}

static Point2D MapNum_ToMatrixPos_point(int8_t MapNum) // 求解方格的行列坐标
{
    Point2D result_ = {0, 0, 0};

    result_.y = static_cast<float>((MapNum - 1) / 5 + 1);
    result_.x = static_cast<float>((MapNum - 1) % 5 + 1);
    return result_;
}

Vector2D MapNum_ToMatrixPos(int8_t MapNum) // 求解方格的行列坐标
{
    Vector2D result_ = {0, 0};

    result_.y = static_cast<float>((MapNum - 1) / 5 + 1);
    result_.x = static_cast<float>((MapNum - 1) % 5 + 1);
    return result_;
}

static float euclid(Point2D a, Point2D b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

// 仅用于把 map 号变成格中心世界坐标（米）
Point2D MapCenterWorld(int8_t map)
{
    if (map < 1 || map > 30)
    {
        Point2D z{0, 0, 0};
        return z;
    }
    return MapNum_RealPos[(int)map - 1];
}
    
Vector2D MapCenterWorld_Vector2D(int8_t map)
{
    Vector2D z{0, 0};
    if (map < 1 || map > 30)
    {
        
        return z;
    }
    z.x=MapNum_RealPos[(int)map - 1].x;
    z.y=MapNum_RealPos[(int)map - 1].y;
    return z;
}

// 计算最佳入口   卖掉了，应该是不用这段函数了
int8_t BestEntrance_calc(Point2D robotPos, RoadResult_S *B1)
{
    int8_t entrance[30];
    uint8_t ecount = 0;
    for (int8_t i = 1; i <= 30; ++i)
    {
        if (IsWalkable(i))
            entrance[ecount++] = i;
    }

    if (!B1)
        return -1;
    uint8_t B1Count = 0;
    if (B1->result1 != 0)
        B1Count++;
    if (B1->result2 != 0)
        B1Count++;
    if (B1->result3 != 0)
        B1Count++;

    float bestJ = 1.0e6f; // 无穷大

    int8_t bestE = -1;

    for (uint8_t i = 0; i < ecount; ++i)
    {
        int8_t e = entrance[i];
        float d_out = euclid(robotPos, MapNum_RealPos[(int)e - 1]);

        int minSteps = BFS_INF;

        for (uint8_t j = 0; j < B1Count; ++j)
        {
            // if (B1[j] == 0) continue;
            // int s = BFS_Steps(e, B1[j]);
            // if (s < minSteps)
            //     minSteps = s;

            switch (j)
            {
            case 0:
            {
                int s = BFS_Steps(e, B1->result1);
                if (s < minSteps)
                    minSteps = s;
                break;
            }
                /* code */
            case 1:
            {
                int s = BFS_Steps(e, B1->result2);
                if (s < minSteps)
                    minSteps = s;
                break;
            }

            case 2:
            {
                int s = BFS_Steps(e, B1->result3);
                if (s < minSteps)
                    minSteps = s;
                break;
            }
            }
        }

        if (minSteps >= BFS_INF)
            continue;

        float J = d_out + (float)minSteps * CELL_M;
        if (J < bestJ)
        {
            bestJ = J;
            bestE = e;
        }
    }

    if (bestE < 0 && ecount > 0)
    {
        float bestD = 1.0e6f;
        int8_t e0 = entrance[0];

        for (uint8_t i = 0; i < ecount; ++i)
        {
            float d = euclid(robotPos, MapNum_RealPos[(int)entrance[i] - 1]);
            if (d < bestD)
            {
                bestD = d;
                e0 = entrance[i];
            }
        }
        bestE = e0;
    }

    return bestE;
} // BestEntrance_calc

PathNode_S PathNodeResult_calc(Point2D robotPos,int8_t MF1, int8_t MF2,int8_t EXIT)
{
    PathNode_S out{0, 0, 0, 0, 0, 26};
    out.exitMap=EXIT;
    // 候选 B1
    RoadResult_S B1_can = MFNum_ToRoadResult(MF1);
    int8_t B1set[3] = {B1_can.result1, B1_can.result2, B1_can.result3};
    uint8_t nB1 = 0;

    for (int i = 0; i < 3; i++)
    {
        if (B1set[i])
            nB1++;
    }

    if (nB1 == 0)
        return out;

    // 候选 B2
    RoadResult_S B2_can = MFNum_ToRoadResult(MF2);

    int8_t B2set[3] =
        {B2_can.result1, B2_can.result2, B2_can.result3};
    uint8_t nB2 = 0;

    for (int i = 0; i < 3; i++)
    {
        if (B2set[i])
            nB2++;
    }

    // 候选bestMF1
    RoadResult_S bestBMF1_can = MFNum_ToCatchRoadResult(MF1);
    int8_t bestMF1set[2] = // 最多两解
        {bestBMF1_can.result1, bestBMF1_can.result2};
    uint8_t nbestBMF1 = 0;
    for (int i = 0; i < 2; i++)
    {
        if (bestMF1set[i])
            nbestBMF1++;
    }

    if (nbestBMF1 == 0)
        return out;

    // 候选bestMF2
    RoadResult_S bestBMF2_can = MFNum_ToCatchRoadResult(MF2);
    int8_t bestBMF2set[2] = // 最多两解
        {bestBMF2_can.result1, bestBMF2_can.result2};

    uint8_t nbestBMF2 = 0;
    for (int i = 0; i < 2; i++)
    {
        if (bestBMF2set[i])
            nbestBMF2++;
    }

    // 可选入口集合（外圈通道格）
    int8_t entrances[30];
    uint8_t eCount = 0;

    const bool isBelow = (robotPos.y < MapNum_RealPos[0].y);  // 梅花林下
    const bool isAbove = (robotPos.y > MapNum_RealPos[29].y); // 梅花林上
    const bool isInside = (!isBelow && !isAbove);             // 梅花林中

    // for(int8_t m=1; m<=30; ++m)
    // {
    //     if(IsWalkable(m))
    //         entrances[eCount++] = m;
    // }
    if (isBelow)
    {
        for (int8_t m = 1; m <= 5; ++m)
        {
            if (IsWalkable(m))
                entrances[eCount++] = m;
        }
    }
    else if (isAbove)
    {
        for (int8_t m = 26; m <= 30; ++m)
        {
            if (IsWalkable(m))
                entrances[eCount++] = m;
        }
    }
    else // inside
    {
        eCount = 0; // 林内无需入口，以B1为起点
    }

    float bestCost = 1.0e9f;
    int8_t bestE = 0, bestB1 = 0, bestB2 = 0, bestBMF1 = 0, bestBMF2 = 0; // 最优

    // 全组合搜索全局最优
    for (uint8_t ie = 0; ie < eCount; ++ie)
    {
        int8_t E = entrances[ie];
        float d_out = euclid(robotPos, MapCenterWorld(E)); // robot→入口 欧氏

        for (uint8_t i1 = 0; i1 < nB1; i1++)
        {
            int8_t B1 = B1set[i1];
            if (!B1)
                continue;

            int sE1 = BFS_Steps(E, B1);
            if (sE1 >= BFS_INF)
                continue;

            // BMF1 必须与 B1 4-邻接
            for (uint8_t m1 = 0; m1 < nbestBMF1; m1++)
            {
                int8_t BMF1 = bestMF1set[m1];
                if (!BMF1)
                    continue;
                if (!IsAdjacent4(B1, BMF1))
                    continue;

                int s1m1 = BFS_Steps(B1, BMF1);
                if (s1m1 >= BFS_INF)
                    continue;

                if (nB2 == 0 || nbestBMF2 == 0)
                {
                    // 无第二段：E→B1→BMF1→Exit
                    int s_m1_X = BFS_Steps(BMF1, out.exitMap);
                    if (s_m1_X >= BFS_INF)
                        continue;

                    float J = d_out + CELL_M * (sE1 + s1m1 + s_m1_X);
                    if (J < bestCost)
                    {
                        bestCost = J;
                        bestE = E;
                        bestB1 = B1;
                        bestBMF1 = BMF1;
                        bestB2 = 0;
                        bestBMF2 = 0;
                    }
                    continue;
                }

                // 有第二段：E→B1→BMF1→B2→BMF2→Exit
                for (uint8_t i2 = 0; i2 < nB2; i2++)
                {
                    int8_t B2 = B2set[i2];
                    if (!B2)
                        continue;

                    int s_m1_2 = BFS_Steps(BMF1, B2);
                    if (s_m1_2 >= BFS_INF)
                        continue;

                    // BMF2 必须与 B2 4-邻接
                    for (uint8_t m2 = 0; m2 < nbestBMF2; m2++)
                    {
                        int8_t BMF2 = bestBMF2set[m2];
                        if (!BMF2)
                            continue;
                        if (!IsAdjacent4(B2, BMF2))
                            continue;

                        int s_2_m2 = BFS_Steps(B2, BMF2);
                        if (s_2_m2 >= BFS_INF)
                            continue;

                        int s_m2_X = BFS_Steps(BMF2, out.exitMap);
                        if (s_m2_X >= BFS_INF)
                            continue;

                        float J = d_out + CELL_M * (sE1 + s1m1 + s_m1_2 + s_2_m2 + s_m2_X);
                        if (J < bestCost)
                        {
                            bestCost = J;
                            bestE = E;
                            bestB1 = B1;
                            bestBMF1 = BMF1;
                            bestB2 = B2;
                            bestBMF2 = BMF2;
                        }
                    }
                }
            }
        }
    }

    // 回退策略：若没有任何可达链路
    if (bestE == 0)
    {
        // 简单回退：选离机器人最近的入口；再选入口→B1 步数最小；再选 B1→B2 最小
        if (eCount == 0)
            return out;
        float bestD = 1.0e9f;
        bestE = entrances[0];
        for (uint8_t ie = 0; ie < eCount; ++ie)
        {
            float d = euclid(robotPos, MapCenterWorld(entrances[ie]));
            if (d < bestD)
            {
                bestD = d;
                bestE = entrances[ie];
            }
        }
        int bestS1 = BFS_INF;
        for (uint8_t i1 = 0; i1 < nB1; i1++)
        {
            int8_t B1 = B1set[i1];
            int s = BFS_Steps(bestE, B1);
            if (s < bestS1)
            {
                bestS1 = s;
                bestB1 = B1;
            }
        }
        if (nB2)
        {
            int bestS2 = BFS_INF;
            for (uint8_t i2 = 0; i2 < nB2; i2++)
            {
                int8_t B2 = B2set[i2];
                int s = BFS_Steps(bestB1, B2);
                if (s < bestS2)
                {
                    bestS2 = s;
                    bestB2 = B2;
                }
            }
        }

        // 为回退分支补充 BMF1/BMF2（各自需与 B1/B2 四邻接）
        //  选择使剩余代价最小的相邻通道
        //  1) BMF1
        int bestCost_m1 = BFS_INF;
        for (uint8_t m1 = 0; m1 < nbestBMF1; ++m1)
        {
            int8_t cand = bestMF1set[m1];
            if (!cand)
                continue;
            if (!IsAdjacent4(bestB1, cand))
                continue;
            int s = BFS_Steps(bestB1, cand) + BFS_Steps(cand, out.exitMap);

            if (s < bestCost_m1)
            {
                bestCost_m1 = s;
                bestBMF1 = cand;
            }
        }
        // 2) BMF2（若存在第二段）
        if (nB2 && bestB2)
        {
            int bestCost_m2 = BFS_INF;
            for (uint8_t m2 = 0; m2 < nbestBMF2; ++m2)
            {
                int8_t cand = bestBMF2set[m2];
                if (!cand)
                    continue;
                if (!IsAdjacent4(bestB2, cand))
                    continue;
                int s = BFS_Steps(bestB2, cand) + BFS_Steps(cand, out.exitMap);

                if (s < bestCost_m2)
                {
                    bestCost_m2 = s;
                    bestBMF2 = cand;
                }
            }
        }
    }

    out.entranceMap = bestE;
    out.bestB1 = bestB1;
    out.bestB2 = bestB2;
    out.bestBMF1 = bestBMF1;
    out.bestBMF2 = bestBMF2;
    return out;
}
    
    
int BFS_Steps(int8_t startMap, int8_t goalMap) // BFS 最少步数
{
    using namespace MF_AutoCtrler;
    if (startMap == goalMap)
        return 0;
    if (!IsWalkable(startMap) || !IsWalkable(goalMap))
        return BFS_INF;

     int16_t dist[31];
     uint8_t vis[31];
    for (int i = 1; i <= 30; ++i)
    {
        dist[i] = (int16_t)BFS_INF;
        vis[i] = 0;
    }

    // 简易环形队列（容量32）
     int8_t q[32];
    uint8_t h = 0, t = 0;
    auto qpush = [&](int8_t v)
    { q[t++ & 31] = v; };
    auto qpop = [&]()
    { return q[h++ & 31]; };
    auto qempty = [&]()
    { return h == t; };

    dist[startMap] = 0;
    vis[startMap] = 1;
    qpush(startMap);

    while (!qempty())
    {
        int8_t u = qpop();
        if (u == goalMap)
            break;

        int8_t c, r;
        Map_ToCR(u, c, r);
        const int8_t dc[4] = {0, 0, -1, 1}, dr[4] = {-1, 1, 0, 0};
        for (int k = 0; k < 4; k++)
        {
            int8_t cc = (int8_t)(c + dc[k]), rr = (int8_t)(r + dr[k]);

            if (cc < 1 || cc > MAP_COLS || rr < 1 || rr > MAP_ROWS)
                continue;

            int8_t v = CR_ToMap(cc, rr);
            if (!IsWalkable(v) || vis[v])
                continue;
            vis[v] = 1;
            dist[v] = (int16_t)(dist[u] + 1);
            qpush(v);
        }
    }

    return (int)dist[goalMap];
}

static bool IsCornerMapByList(int8_t map, const int *cornerMap, int cornerCount)
{
    for (int i = 0; i < cornerCount; ++i)
    {
        if ((int8_t)cornerMap[i] == map)
            return true;
    }
    return false;
}

static void PushMustPastNode(int8_t *mustPastMap, int cap, int &len, int8_t node)
{
    if (node <= 0)
        return;
    if (len > 0 && mustPastMap[len - 1] == node)
        return;
    if (len < cap)
        mustPastMap[len++] = node;
}

PathInformation_S PathInformation_calc(Point2D robotPos, int8_t MF1, int8_t MF2)
{
    /**
     * 
     */
    PathInformation_S result;
    RoadResult_S MF1Road = MFNum_ToCatchRoadResult(MF1);
    RoadResult_S MF2Road = MFNum_ToCatchRoadResult(MF2);

    int cornerMap[4] = {1, 5, 26, 30}; // 四个角落的地图编号

    //解出目标梅花桩相邻的通道
    int roadMF1[2] = {MF1Road.result1, MF1Road.result2}; //集合，后续取出最优，result为0表示无效
    int roadMF2[2] = {MF2Road.result1, MF2Road.result2};

    //根据当前机器人距离第一个MF1相邻通道的距离，生成最优路径的第一段路径
    //如果机器人不在梅花林内，则entranceMap只能是1~5以及26~30的范围内，同时允许entranceMap和roadMF1的通道重合
    //如果机器人在梅花林内，则entranceMap就是去roadMF1的路径上离机器人最近的一个通道，同时允许entranceMap和roadMF1的通道重合
    //生成路径时候要包含转角，如果路径上包含转角，则需要在路径中添加转角点（cornerMap）作为路径节点
    //PathInformation_S当中的mustPastMap就是路径上的必经点，包含entranceMap、roadMF1、转角点（如果有的话）以及roadMF2（如果有的话）
    //且数组的顺序要按照路径顺序来，允许折返(如果需要的话)
    result.entranceMap = 0;
    result.MFroad[0] = 0;
    result.MFroad[1] = 0;

    // 林外上方时，禁止路径规划
    if (robotPos.y > MapNum_RealPos[29].y)
        return result;

    // 入口集合
    int8_t entrances[30] = {0};
    uint8_t entranceCount = 0;

    int8_t robotMap = GetMapNumFromPos(robotPos);
    bool isRobotInsideMap = (robotMap >= 1 && robotMap <= 30 && IsWalkable(robotMap));

    if (isRobotInsideMap)
    {
        // 林内起点直接取当前所在通道，避免出现与真实起点不一致的入口
        entrances[entranceCount++] = robotMap;
    }
    else
    {
        bool isBelow = (robotPos.y < MapNum_RealPos[0].y);  // 梅花林下 ,flase则为在梅林上方
        if (isBelow)
        {
            for (int8_t m = 1; m <= 5; ++m)
            {
                if (IsWalkable(m))
                    entrances[entranceCount++] = m;
            }
        }
        else
        {
            for (int8_t m = 26; m <= 30; ++m)
            {
                if (IsWalkable(m))
                    entrances[entranceCount++] = m;
            }
        }
    }

    if (entranceCount == 0)
        return result;

    int8_t bestEntrance = 0;
    int8_t bestRoad1 = 0;
    int8_t bestRoad2 = 0;
    float bestCost = 1.0e9f;

    bool hasMF2 = (MF2 != 0 && (roadMF2[0] != 0 || roadMF2[1] != 0));

    for (uint8_t ie = 0; ie < entranceCount; ++ie)
    {
        int8_t E = entrances[ie];
        float dRobotToE = euclid(robotPos, MapCenterWorld(E));

        for (int i1 = 0; i1 < 2; ++i1)
        {
            int8_t R1 = roadMF1[i1];
            if (R1 == 0)
                continue;

            int sE1 = BFS_Steps(E, R1);
            if (sE1 >= BFS_INF)
                continue;

            if (!hasMF2)
            {
                int s1X = BFS_Steps(R1, result.exitMap);
                if (s1X >= BFS_INF)
                    continue;

                float J = dRobotToE + CELL_M * (float)(sE1 + s1X);
                if (J < bestCost)
                {
                    bestCost = J;
                    bestEntrance = E;
                    bestRoad1 = R1;
                    bestRoad2 = 0;
                }
            }
            else
            {
                for (int i2 = 0; i2 < 2; ++i2)
                {
                    int8_t R2 = roadMF2[i2];
                    if (R2 == 0)
                        continue;

                    int s12 = BFS_Steps(R1, R2);
                    if (s12 >= BFS_INF)
                        continue;

                    int s2X = BFS_Steps(R2, result.exitMap);
                    if (s2X >= BFS_INF)
                        continue;

                    float J = dRobotToE + CELL_M * (float)(sE1 + s12 + s2X);
                    if (J < bestCost)
                    {
                        bestCost = J;
                        bestEntrance = E;
                        bestRoad1 = R1;
                        bestRoad2 = R2;
                    }
                }
            }
        }
    }

    if (bestEntrance == 0 || bestRoad1 == 0)
        return result;

    result.entranceMap = bestEntrance;
    result.MFroad[0] = bestRoad1;
    result.MFroad[1] = bestRoad2;

    int8_t seg1[32] = {0}, seg2[32] = {0}, seg3[32] = {0};
    int len1 = BFS_GetPath(bestEntrance, bestRoad1, seg1, 32);
    if (len1 <= 0)
        return result;

    int len2 = 0;
    int len3 = 0;
    if (bestRoad2 != 0)
    {
        len2 = BFS_GetPath(bestRoad1, bestRoad2, seg2, 32);
        if (len2 <= 0)
            return result;

        len3 = BFS_GetPath(bestRoad2, result.exitMap, seg3, 32);
        if (len3 <= 0)
            return result;
    }
    else
    {
        len3 = BFS_GetPath(bestRoad1, result.exitMap, seg3, 32);
        if (len3 <= 0)
            return result;
    }

    int8_t fullPath[96] = {0};
    int fullLen = 0;

    for (int i = 0; i < len1 && fullLen < 96; ++i)
        fullPath[fullLen++] = seg1[i];

    if (bestRoad2 != 0)
    {
        for (int i = 1; i < len2 && fullLen < 96; ++i)
            fullPath[fullLen++] = seg2[i];
    }

    for (int i = 1; i < len3 && fullLen < 96; ++i)
        fullPath[fullLen++] = seg3[i];

    int mustLen = 0;
    PushMustPastNode(result.mustPastMap, 12, mustLen, result.entranceMap);

    bool pushedRoad1 = (result.entranceMap == bestRoad1);
    bool pushedRoad2 = (bestRoad2 != 0 && result.entranceMap == bestRoad2);

    for (int i = 0; i < fullLen; ++i)
    {
        int8_t node = fullPath[i];

        if (node == bestRoad1)
        {
            if (!pushedRoad1)
            {
                PushMustPastNode(result.mustPastMap, 12, mustLen, node);
                pushedRoad1 = true;
            }
            continue;
        }

        if (bestRoad2 != 0 && node == bestRoad2)
        {
            if (!pushedRoad2)
            {
                PushMustPastNode(result.mustPastMap, 12, mustLen, node);
                pushedRoad2 = true;
            }
            continue;
        }

        if (node == result.exitMap || IsCornerMapByList(node, cornerMap, 4))
        {
            PushMustPastNode(result.mustPastMap, 12, mustLen, node);
        }
    }

    PushMustPastNode(result.mustPastMap, 12, mustLen, result.exitMap);

    // 记录MFroad在mustPastMap中的索引
    for (int i = 0; i < mustLen; ++i)
    {
        if (result.mustPastMap[i] == result.MFroad[0])
            result.Index_MFroad[0] = i;
        if (result.mustPastMap[i] == result.MFroad[1])
            result.Index_MFroad[1] = i;
    }

    return result;
}

int BFS_GetPath(int8_t startMap, int8_t goalMap, int8_t *outPath, int maxLen)
{
    using namespace MF_AutoCtrler;

    if (maxLen < 1)
        return -1;
    if (startMap == goalMap)
    {
        outPath[0] = startMap;
        return 1;
    }
    if (!IsWalkable(startMap) || !IsWalkable(goalMap))
        return 0;

    static int16_t dist[31];
    static uint8_t vis[31];
    static int8_t parent[31]; // 记录路径回溯
    for (int i = 1; i <= 30; ++i)
    {
        dist[i] = (int16_t)BFS_INF;
        vis[i] = 0;
        parent[i] = 0;
    }

    // 简易环形队列
    static int8_t q[32];
    uint8_t h = 0, t = 0;
    auto qpush = [&](int8_t v)
    { q[t++ & 31] = v; };
    auto qpop = [&]()
    { return q[h++ & 31]; };
    auto qempty = [&]()
    { return h == t; };

    dist[startMap] = 0;
    vis[startMap] = 1;
    qpush(startMap);

    bool found = false;

    while (!qempty())
    {
        int8_t u = qpop();
        if (u == goalMap)
        {
            found = true;
            break;
        }

        int8_t c, r;
        Map_ToCR(u, c, r);
        const int8_t dc[4] = {0, 0, -1, 1}, dr[4] = {-1, 1, 0, 0};
        for (int k = 0; k < 4; k++)
        {
            int8_t cc = (int8_t)(c + dc[k]), rr = (int8_t)(r + dr[k]);

            if (cc < 1 || cc > MAP_COLS || rr < 1 || rr > MAP_ROWS)
                continue;

            int8_t v = CR_ToMap(cc, rr);
            if (!IsWalkable(v) || vis[v])
                continue;
            vis[v] = 1;
            dist[v] = (int16_t)(dist[u] + 1);
            parent[v] = u; // 记录父节点
            qpush(v);
        }
    }

    if (!found)
        return 0;

    int steps = (int)dist[goalMap];
    int pathLen = steps + 1; // 包含起点

    if (pathLen > maxLen)
        return -1; // 缓冲区不足

    // 回溯路径
    int8_t curr = goalMap;
    for (int i = pathLen - 1; i >= 0; --i)
    {
        outPath[i] = curr;
        curr = parent[curr];
    }

    return pathLen;
}

/**
 * @brief 根据世界坐标判断所在的地图网格编号
 * @param pos 世界坐标点 (x, y)
 * @return int8_t 地图编号 (1-30), 如果超出范围返回 0
 */
int8_t GetMapNumFromPos(Point2D pos)
{
    //定义原点偏移和网格尺寸
    const float GRID_SIZE = 1.2f;      // 网格边长
    const float Y_OFFSET_START = 2.0f; // Y轴起始坐标

    // 范围检查
    if (pos.x < 0 || pos.x > (5 * GRID_SIZE) ||
        pos.y < Y_OFFSET_START || pos.y > (Y_OFFSET_START + 6 * GRID_SIZE))
    {
        return 0; // 超出地图范围
    }

    // 3. 计算行列 (1-based index)
    // Col 由 x 决定: 0~1.2 -> 1, 1.2~2.4 -> 2 ...
    int8_t c = (int8_t)(pos.x / GRID_SIZE) + 1;

    // Row 由 y 决定: 2.0~3.2 -> 1, 3.2~4.4 -> 2 ...
    int8_t r = (int8_t)((pos.y - Y_OFFSET_START) / GRID_SIZE) + 1;

    // 4. 再次检查行列是否越界
    if (c < 1 || c > MF_AutoCtrler::MAP_COLS ||
        r < 1 || r > MF_AutoCtrler::MAP_ROWS)
    {
        return 0;
    }

    return MF_AutoCtrler::CR_ToMap(c, r);
}

float chassisMoveDir(int8_t startmapNum, int8_t next_mapNum)
{
    if(startmapNum < 1 || startmapNum >30 || next_mapNum < 1 || next_mapNum >30)
    {
        return -1.0f; // 无效输入
    }
    
    bool isinMFstart = IsWalkable(startmapNum);
    bool isinMFnext = IsWalkable(next_mapNum);


    if(isinMFstart && isinMFnext)
    {
        // 两个都不在梅花桩内，直接计算方向
        Point2D startPos = MapCenterWorld(startmapNum);
        Point2D nextPos = MapCenterWorld(next_mapNum);
        float dx = nextPos.x - startPos.x;
        float dy = nextPos.y - startPos.y;
        float deg = rad_to_deg(atan2f(dy, dx));
        deg = normalize_deg_0_360(deg);
        return deg;
    }
    else
    {
        return -1.0f; //无效
    }
}

} // namespace MF_AutoCtrler


