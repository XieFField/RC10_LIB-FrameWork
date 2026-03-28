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
 * @brief 判断底盘当前坐标是否处于目标放ge
 */
bool 




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

    static int16_t dist[31];
    static uint8_t vis[31];
    for (int i = 1; i <= 30; ++i)
    {
        dist[i] = (int16_t)BFS_INF;
        vis[i] = 0;
    }

    // 简易环形队列（容量32）
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
    // 1. 定义原点偏移和网格尺寸
    // 注意：Row 1 的中心是 y=2.6，说明 Row 1 的 y 起始边是 2.6 - 0.6 = 2.0
    const float GRID_SIZE = 1.2f;      // 网格边长
    const float Y_OFFSET_START = 2.0f; // Y轴起始坐标

    // 2. 简单的范围检查 (可选)
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

    // 5. 转换为地图编号
    return MF_AutoCtrler::CR_ToMap(c, r);
}

} // namespace MF_AutoCtrler


