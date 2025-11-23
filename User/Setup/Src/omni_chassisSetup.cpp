#include "omni_chassisSetup.h"

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
    
    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
         if (path_.isFinished() == false)
        {
          
           speed = path_.plan(path_point_);
           path_point_ = path_point_ + (speed * 0.001f);
           if (num > 5)
          {
            debug_uart.printf_DMA("%f,%f,%f\n", path_point_.x, path_point_.y, speed.magnitude());
            num = 0;
          }
       }
        else
       {
          speed = {0.0f, 0.0f};
        //path.reset();
        //point = {16.0f, 20.0f};
        //num = 0;
       }
        target_chassis_twist_.vx=speed.x;
        target_chassis_twist_.vy=speed.y;
            break;
        }
        case CHASSIS_STOP:
        {
        target_chassis_twist_.vx=0;
        target_chassis_twist_.vy=0;
            break;
        }
        default:
            break;
        }
       setWorldSpeed(target_chassis_twist_);
       this->update();
}
//void OmniChassis_Setup::loop()
//{
//    if (!init_flag)
//        return;
//    
//    switch (chassis_status_)
//    {
//        case CHASSIS_MANUAL_CONTROL_A:
//        {
//            break;
//        }

//        case CHASSIS_MANUAL_CONTROL_B:
//        {
//            break;
//        }
//        case CHASSIS_AUTO_CONTROL:
//        {
//        Vector2D pathEnd = path_.Get_Point(1.0f);
//        // 新增：机器人到几何终点的真实距离
//        float distToEnd =  (getRobotposition() - pathEnd).magnitude();
//         if (path_.isFinished() == false&&!is_path_completed_)
//        {
//           //1.找最近点+t值
//           nearestPt = GetPathNearestPoint(getRobotposition(), tNearest);
//           //2.找前视点+前进方向
//           lookaheadPt = FindLookaheadPoint(tNearest, tLookahead);
//           lookaheadTangent = path_.Get_Tangent_Vector(tLookahead);
//           //3.计算横向偏差
//           lateralError = CalculateLateralError(getRobotposition(), nearestPt, tLookahead);
//           //4.横向偏差PID控制
//           correctspeed = pid_track.pid_calc(0.0f, lateralError);
//           Vector2D corrDir(-lookaheadTangent.y, lookaheadTangent.x); // 纠偏方向（垂直前进，左右纠偏）
//           corrVelocity = corrDir * correctspeed; // 合成纠偏速度（方向+大小）
//            //5.规划速度+叠加纠偏速度
//           planspeed = path_.plan(getRobotposition());
//           baseVelocity = lookaheadTangent * planspeed.magnitude(); // 保证速度方向和前进方向一致
//           speed = baseVelocity + corrVelocity; // 叠加纠偏速度
//          
//       }
//        else
//       {
//        // 新增：计算当前机器人实际速度
//        float currentSpeed = speed.magnitude();

//        // 三重条件：全部满足才彻底停稳（速度归0）
//        if (path_.isFinished() &&          // 条件1：你的原有判断（预设距离跑完）
//            distToEnd <= DIST_TO_END &&    // 条件2：真的到终点附近（≤5cm）
//            currentSpeed <= 0.05f)    // 条件3：速度足够慢（≤1cm/s）
//        {
//            speed = {0.0f, 0.0f};          // 彻底停稳
//            is_path_completed_ = true;     // 标记完成，避免重复进入
//            
//        }
//        else
//        {
//            // 新增：渐进减速（关键！避免冲终点）
//            speed = speed * 0.95f;
//        }
//       }
//        target_chassis_twist_.vx=speed.x;
//        target_chassis_twist_.vy=speed.y;
//            break;
//        }
//        case CHASSIS_STOP:
//        {
//        target_chassis_twist_.vx=0;
//        target_chassis_twist_.vy=0;
//            break;
//        }
//        default:
//            break;
//        }
//       setWorldSpeed(target_chassis_twist_);
//       this->update();
//}
// /**
// * @brief 整合已有接口，获取“最近点坐标”和“对应的t值”
// * @param robotPos 输入：机器人当前实际位置（闭环核心输入）
// * @param tNearest 输出：最近点对应的曲线参数t（0~1），给后续找前视点用
// * @return Vector2D 输出：最近点的坐标（给后续算横向偏差用）
// */
//Vector2D OmniChassis_Setup::GetPathNearestPoint(const Vector2D& robotPos, float& tNearest) {
//    // 第一步：调用你的Get_Nearest_Distance，拿到tNearest（最近点对应的t值）
//    // 重点：第二个参数传 &tNearest（tNearest的地址），因为你的函数是“输出参数”（通过指针赋值）
//     path_.Get_Nearest_Distance(robotPos, &tNearest);

//    // 第二步：用第一步拿到的tNearest，调用你的Get_Point，拿到最近点坐标
//    Vector2D nearestPt = path_.Get_Point(tNearest);

//    // 第三步：返回最近点坐标，给后续“算横向偏差”用
//    return nearestPt;
//}

//// 函数作用：输入最近点的编号tNearest，输出前视点坐标和它的编号tLookahead
//Vector2D OmniChassis_Setup::FindLookaheadPoint(float tNearest, float& tLookahead) {
//    // -------------- 对应第1步：初始化，从最近点开始 --------------
//    tLookahead = tNearest; // 前视点的编号，先从最近点的编号开始（比如t=0.3）
//    float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
//    float step = 0.005f; // 每次挪的“小步子”（t增加0.005，比如0.3→0.305）

//    // 拿到最近点的坐标（比如(5.2, 6.1)），作为“挪步”的起点
//    Vector2D lastPt = path_.Get_Point(tLookahead);

//    // -------------- 对应第2步：小步慢挪，直到累计距离够前视距离 --------------
//    // 条件：1. 编号t没到终点（<1.0）；2. 累计距离还没到前视距离（<0.4m）
//    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist) {
//        // 1. 往前挪一小步：t增加0.005（比如0.3→0.305）
//        float nextT = tLookahead + step;
//        // 防止挪超终点：如果nextT>1.0，就改成1.0（不能超出曲线）
//        nextT = std::min(nextT, 1.0f);

//        // 2. 拿到这一步挪到的点的坐标（比如t=0.305对应的曲线点(5.22, 6.11)）
//        Vector2D nextPt = path_.Get_Point(nextT);

//        // 3. 计算这一步走了多远（比如从(5.2,6.1)到(5.22,6.11)，距离≈0.022m）
//        float distStep = (nextPt - lastPt).magnitude();

//        // 4. 累计距离：把这一步的距离加进去（比如0+0.022=0.022m）
//        accumulatedDist += distStep;

//        // 5. 更新：准备下一步挪步（把当前点当起点，当前t当下一步的基础）
//        tLookahead = nextT; // 编号更新为0.305
//        lastPt = nextPt;    // 起点更新为(5.22,6.11)
//    }

//    // -------------- 对应第3步：如果到终点了，直接用终点当前视点 --------------
//    if (tLookahead >= 1.0f) {
//        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
//    }

//    // -------------- 返回前视点坐标 --------------
//    return lastPt;
//}

///**
// * @brief 计算横向偏差（带方向：正=偏左，负=偏右，单位：m）
// * @param robotPos 机器人当前位置（主函数的m_robotPos）
// * @param nearestPt 曲线最近点（主函数的nearestPt，即P(t')）
// * @param tLookahead 前视点t值（主函数的tLookahead，用于获取稳定切向量）
// * @return float 纯横向偏差（无前后干扰，直接给PID用）
// */
//float OmniChassis_Setup::CalculateLateralError(const Vector2D& robotPos, const Vector2D& nearestPt, float tLookahead) {
//    // 步骤1：计算原始偏差向量 Δp = 机器人位置 - 最近点（你的定义：Δp = p - p(t')）
//    Vector2D delta_p = robotPos - nearestPt;

//    // 步骤2：获取前视点的切向量（和主函数一致，确保前进方向基准统一）
//    // 主函数里已经调用过一次，但这里再调用一次，保证偏差计算和前进方向完全同步
//    lookaheadTangent = path_.Get_Tangent_Vector(tLookahead);

//    // 步骤3：单位化前视点切向量（确保后续点积计算时，不会被长度缩放干扰）
//    lookaheadTangent.normalize();
//    

//    // 步骤4：定义“横向方向”：垂直于前视点切向量（左转90度，和主函数corrDir方向一致）
//    // 主函数纠偏方向是 corrDir = (-lookaheadTangent.y, lookaheadTangent.x)，这里横向方向和它保持一致
//    Vector2D lateral_dir = Vector2D(-lookaheadTangent.y, lookaheadTangent.x);
//    // 横向方向也归一化：确保点积计算的偏差单位是“米”（无缩放干扰）
//    lateral_dir.normalize();

//    // 步骤5：核心：计算原始偏差Δp在“横向方向”的投影 → 纯横向偏差
//    // 点积公式：delta_p · lateral_dir = |delta_p| * cosθ（θ是Δp和横向方向的夹角）
//    // 作用：过滤前后方向干扰（前后方向与横向垂直，cos90°=0），只留左右偏差
//    float lateral_err = delta_p*lateral_dir;

//    // （可选）调试用：如果发现纠偏方向反了，把偏差乘-1即可
//    // lateral_err *= -1;

//    return lateral_err;
//}