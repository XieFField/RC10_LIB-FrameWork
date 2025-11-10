/**
 * @file autoctrlerdemo.cpp
 * @author XieFField
 * @brief 梅花林路径点测试程序
 */

#include "demo_AutoCtrler.h"
#include <iostream>

int main(void)
{
    using namespace MF_AutoCtrler;
    using namespace std;
    int8_t MF1 = 4;
    int8_t MF2 = 12;

    Point2D robotPos = {3.0f, 0.0f, 0.0f};
    
    PathNode_S result = PathNodeResult_calc(robotPos, MF1, MF2);

    RoadResult_S road1 = MFNum_ToCatchRoadResult(MF1);
    RoadResult_S road2 = MFNum_ToCatchRoadResult(MF2);

    cout << "MF1 Road Results: " << (int)road1.result1 << ", "
         << (int)road1.result2 << ", " << (int)road1.result3 << endl;

    cout << "MF2 Road Results: " << (int)road2.result1 << ", "
         << (int)road2.result2 << ", " << (int)road2.result3 << endl;

    cout << "Entrance Map: " << (int)result.entranceMap << endl;
    cout << "Best B1: " << (int)result.bestB1 << endl;
    cout << "Best BMF1: " << (int)result.bestBMF1 << endl;
    cout << "Best B2: " << (int)result.bestB2 << endl;
    cout << "Best BMF2: " << (int)result.bestBMF2 << endl;
    cout << "Exit Map: " << (int)result.exitMap << endl;

    return 0;
}