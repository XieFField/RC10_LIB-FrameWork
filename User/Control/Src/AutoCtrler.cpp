#include "AutoCtrler.h"



namespace MF_AutoCtrler{

//行列转地图编号
int8_t CR_ToMap(int8_t c, int8_t r) 
{
    return (int8_t)((r - 1) * MAP_COLS + c);
}

//地图编号转行列
void Map_ToCR(int8_t map, int8_t& c, int8_t& r) 
{
    r = (int8_t)((map - 1) / MAP_COLS + 1);
    c = (int8_t)((map - 1) % MAP_COLS + 1);

}

static int8_t MFNum_TransforMapNum(int8_t MFNum)//将梅花桩编号转换为梅花林方格地图编号
{
    if(MFNum < 1 || MFNum > 12 || (MFNum == 5 || MFNum == 8))
        return -1;

    return MFNum + 6 + 2 * ((MFNum - 1) / 3.0);
}

static int8_t MapNum_TransformMFNum(int8_t mapNum)//将梅花林方格地图编号转换为梅花桩编号
{
    int8_t MFNum_ = mapNum - 6 - 2 * ((mapNum - 7) / 3);
    if(MFNum_ < 1 || MFNum_ > 12 || (MFNum_ == 5 || MFNum_ == 8))
        return -1;
    return MFNum_;
}

static RoadResult_S MFNum_ToRoadResult(int8_t MFNum) //求解梅花桩所有前一通道结果
{
	RoadResult_S result = {0, 0, 0};
	if(MFNum < 1 || MFNum > 12 || (MFNum == 5 || MFNum == 8))
        return result;
	
	int8_t mapNum_ = MFNum_TransforMapNum(MFNum);
	
	int8_t candidate[4];
	candidate[0] = mapNum_ - 6;
	candidate[1] = mapNum_ - 4;
	candidate[2] = mapNum_ + 4;
	candidate[3] = mapNum_ + 6;
	
	for(int i = 0; i < 4; i++)
	{
		int8_t mf = MapNum_TransformMFNum(candidate[i]);
		//过滤梅花桩块
        if(mf < 1 || mf > 12 || mf == 5 || mf == 8)
            candidate[i] = 0;
	}
	

	int8_t validResults[3] = {0}; 
    int validCount = 0;
    //过滤0
    for(int i = 0; i < 4 && validCount < 3; i++)
    {
        if(candidate[i] != 0)
            validResults[validCount++] = candidate[i];
        
    }

	//排列
	for(int i = 0; i < validCount - 1; i++)
    {
        for(int j = 0; j < validCount - 1 - i; j++)
        {
            if(validResults[j] > validResults[j+1])
            {
                int8_t temp = validResults[j];
                validResults[j] = validResults[j+1];
                validResults[j+1] = temp;
            }
        }
    }

	result.result1 = validResults[0];
    result.result2 = validResults[1];
    result.result3 = validResults[2];
    
    return result;
}

//撞墙判断(梅花桩)
static bool IsWalkable(int8_t map) 
{
    if (map < 1 || map > 30) 
        return false;
    int8_t mf = MapNum_TransformMFNum(map);
    // mf==-1 → 通道；否则为梅花桩格（障碍）
    return (mf == -1);
}

static Point2D MapNum_ToMatrixPos(int8_t MapNum) //求解方格的行列坐标
{
	Point2D result_ = {0, 0, 0}; 
	
	result_.y = static_cast<float>((MapNum - 1) / 5 + 1);
	result_.x = static_cast<float>((MapNum - 1) % 5 + 1);
	return result_;
}

static float euclid(Point2D a, Point2D b) 
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx*dx + dy*dy);
}


// 仅用于把 map 号变成格中心世界坐标（米）
static Point2D MapCenterWorld(int8_t map)
{
    if (map < 1 || map > 30) 
    {
        Point2D z{0,0,0};
        return z;
    }
    return MapNum_RealPos[(int)map - 1];
}

//计算最佳入口
int8_t BestEntrance_calc(Point2D robotPos, RoadResult_S* B1) 
{
    int8_t entrance[30]; uint8_t ecount = 0;
    for(int8_t i =1; i <=30; ++i)
    {
        if(IsWalkable(i))
            entrance[ecount++] = i;
    }

    static uint8_t B1Count = 0;
    if(B1->result1 != 0) B1Count++;
    if(B1->result2 != 0) B1Count++;
    if(B1->result3 != 0) B1Count++;

    float bestJ = 1.0e6f; //无穷大

    int8_t bestE = -1;

    for(uint8_t i = 0; i < ecount; ++i)
    {
        int8_t e = entrance[i];
        float d_out = euclid(robotPos, MapCenterWorld(e));

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

        if(minSteps >= BFS_INF)
            continue;

        float J = d_out + (float)minSteps * CELL_M;
        if (J < bestJ) 
        {
            bestJ = J;
            bestE = e;
        }
    }

    if(bestE < 0 && ecount > 0)
    {
        float bestD = 1.0e6f;
        int8_t e0 = entrance[0];

        for(uint8_t i = 0; i < ecount; ++i)
        {
            float d = euclid(robotPos, MapCenterWorld(entrance[i]));
            if(d < bestD)
            {
                bestD = d;
                e0 = entrance[i];
            }
        }
        bestE = e0;
    }


    return bestE;
}//BestEntrance_calc

PathNode_S PathNodeResult_calc(Point2D robotPos, 
                                 int8_t MF1, int8_t MF2)
{
    PathNode_S out{0,0,0,19};

    //候选B1

    RoadResult_S B1_can = MFNum_ToRoadResult(MF1);

    out.entranceMap = BestEntrance_calc(robotPos, &B1_can);

    uint8_t B1_canCount = 0;
    if(B1_can.result1 != 0) B1_canCount++;
    if(B1_can.result2 != 0) B1_canCount++;
    if(B1_can.result3 != 0) B1_canCount++;

    //最优B1
    int bestS = BFS_INF;
    for(uint8_t i = 0; i < B1_canCount; ++i)
    {
        switch(i)
        {
            case 0:
            {
                int s = BFS_Steps(out.entranceMap, B1_can.result1);
                if(s < bestS)
                {
                    bestS = s;
                    out.bestB1 = B1_can.result1;
                }
                break;
            }
            case 1:
            {
                int s = BFS_Steps(out.entranceMap, B1_can.result2);
                if(s < bestS)
                {
                    bestS = s;
                    out.bestB1 = B1_can.result2;
                }
                break;
            }
            case 2:
            {
                int s = BFS_Steps(out.entranceMap, B1_can.result3);
                if(s < bestS)
                {
                    bestS = s;
                    out.bestB1 = B1_can.result3;
                }
                break;
            }
        }
    }

    //候选B2
    RoadResult_S B2_can = MFNum_ToRoadResult(MF2);

    int8_t B2_canCount = 0;
    if(B2_can.result1 != 0) B2_canCount++;
    if(B2_can.result2 != 0) B2_canCount++;
    if(B2_can.result3 != 0) B2_canCount++;

    //最佳B2
    int bestS2 = BFS_INF;

    for(uint8_t i = 0; i < B2_canCount; ++i)
    {
        switch(i)
        {
            case 0:
            {    
                int s = BFS_Steps(out.entranceMap, B2_can.result1);
                if(s < bestS2)
                {
                    bestS2 = s;
                    out.bestB2 = B2_can.result1;
                }
                break;
            }
            case 1:
            {
                int s = BFS_Steps(out.entranceMap, B2_can.result2);
                if(s < bestS2)
                {
                    bestS2 = s;
                    out.bestB2 = B2_can.result2;
                }
                break;
            }
            case 2:
            {
                int s = BFS_Steps(out.entranceMap, B2_can.result3);
                if(s < bestS2)
                {
                    bestS2 = s;
                    out.bestB2 = B2_can.result3;
                }
                break;
            }
        }
    }

    return out;
}//PathNodeResult_calc


}//namespace MF_AutoCtrler

int BFS_Steps(int8_t startMap, int8_t goalMap)// BFS 最少步数
{
    using namespace MF_AutoCtrler;
    if (startMap == goalMap) return 0;
    if (!IsWalkable(startMap) || !IsWalkable(goalMap)) 
        return BFS_INF;

    static int16_t dist[31];
    static uint8_t vis[31];
    for (int i = 1; i <= 30; ++i) { dist[i] = (int16_t)BFS_INF; vis[i] = 0; }

    // 简易环形队列（容量32）
    static int8_t q[32]; uint8_t h=0, t=0;
    auto qpush = [&](int8_t v){ q[t++ & 31] = v; };
    auto qpop  = [&](){ return q[h++ & 31]; };
    auto qempty= [&](){ return h==t; };

    dist[startMap]=0; 
    vis[startMap]=1; 
    qpush(startMap);

    while(!qempty())
    {
        int8_t u = qpop();
        if (u == goalMap) 
            break;

        int8_t c,r; Map_ToCR(u,c,r);
        const int8_t dc[4]={0,0,-1,1}, dr[4]={-1,1,0,0};
        for(int k=0;k<4;k++)
        {
            int8_t cc=(int8_t)(c+dc[k]), rr=(int8_t)(r+dr[k]);

            if (cc < 1 || cc > MAP_COLS || rr < 1 || rr > MAP_ROWS) 
                continue;

            int8_t v = CR_ToMap(cc,rr);
            if (!IsWalkable(v) || vis[v]) 
                continue;
            vis[v]=1; 
            dist[v]=(int16_t)(dist[u]+1);
            qpush(v);
        }
    }

    return (int)dist[goalMap];
}