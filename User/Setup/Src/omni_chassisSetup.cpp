#include "omni_chassisSetup.h"
// Path_line path_line_;
// Speedplanner_1D_Param_Config path_param({.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 4.0f, .maxSpeed = 0.5f, .initialSpeed = 0.05f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f});
// Path_line path_line_(path_param);
#if debug_ladar

int last_cout_ladar_data = -1;

#endif

uint32_t chassisstackHighWaterMark = 0;

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;

//	chassisstackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
	
    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();
    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;
    robot_pos_.x += original_point_.x;
    robot_pos_.y += original_point_.y;

    Angle_Twist angle_twist = {0};
    angle_twist.yaw_rate = dyaw;
    angle_twist.yaw_angle = yaw;
    this->updateAngleData(angle_twist);

    switch (chassis_status_)
    {
    case CHASSIS_MANUAL_CONTROL_A:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        if (_tool_Abs(airjoy_data_.right_x) > 0.05f)
            target_chassis_twist_.yaw_rate = airjoy_data_.right_x * 6;
        else
            target_chassis_twist_.yaw_rate = 0.0f;

        target_yaw_ = yaw;

        this->set_Target(target_chassis_twist_);

        break;
    }

    case CHASSIS_MANUAL_CONTROL_B:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 0.6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 0.6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        // 获取当前角度
        float yaw_real_angle = yaw;

        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
        }

        // this->setWorldSpeed(target_chassis_twist_);
        this->set_Target(target_chassis_twist_);
        // this->update();
        break;
    }

    case CHASSIS_LOCK_FORWEAPON:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        float target_yaw_angle = 90.0f;

        float yaw_real_angle = yaw;

        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_angle, yaw_real_angle);
        }

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        // this->setWorldSpeed(target_chassis_twist_);
        this->set_Target(target_chassis_twist_);
        break;
    }

    case CHASSIS_AUTO_CONTROL:
    {
        if (flag == 1)
        {
            flag = 0;
            flag_run = 1;
            if (path_flag == 0)
            {
                Clamping_Bar_Selection_Planning();
				WeaponSage_Start=1;
            }
            else
            {
                KFS_Selection_Planning();
            }
        }

        if (flag_run == 1)
        {
            if (path_line_.Is_End() == true)
            {
                num++;

                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
                // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                planspeed = path_line_.plan(robot_pos_);
                if(path_line_.get_pid_end_flag()==0)
                {
                    Path_correction();
                    speed = planspeed + corrVelocity;// 最终速度 = 规划的前进速度 + 横向纠偏速度
                }
                else
                    speed = planspeed; 
                //                if (num > 2)
                //                {
                //                    debug_uart.printf_DMA("%f,%f,%f,%f,%f,%f\n", robot_pos_.x, robot_pos_.y, speed.magnitude(), speed.x, speed.y, corrVelocity.magnitude());
                //                    num = 0;
                //                }
            }
            else
            {
                if (flag_1 == 0)
                {
                    flag = 0;
                    planspeed.x = 0.0f;
                    planspeed.y = 0.0f;
                    Path_correction();
                    speed = planspeed + corrVelocity;
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
					WeaponSage_END=1;
                }
                else
                {
                    flag = 0;
                    flag_run = 0;
                    speed.x = 0.0f;
                    speed.y = 0.0f;
                    path_line_.plan_reset();
                    path_line_.Reset();
                    planspeed.x = 0.0f;
                    planspeed.y = 0.0f;
                    chassis_status_ = CHASSIS_STOP;
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
					WeaponSage_END=1;
                }
            }
        }
        else
        {
            target_yaw_ = yaw;
            target_chassis_twist_.vx = 0.0f;
            target_chassis_twist_.vy = 0.0f;
            speed.x = 0.0f;
            speed.y = 0.0f;
        }
        if (path_line_.index_ == 1)
        {
            if (path_flag == 0)
            {
        
                target_yaw_ = -90.0f;
//                if (abs(target_yaw_ - yaw) > 1.0f)
//                {
//                    target_chassis_twist_.vx = 0.0f;
//                    target_chassis_twist_.vy = 0.0f;
//                }
            }
        }
        // 获取当前角度
        float yaw_real_angle = yaw;
        // float yaw_real_angle = ladar_data_.yaw;
        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
        }
        // this->set_ControlMode(CURRENT_ZERO_MODE);
        this->set_Target(target_chassis_twist_);
        break;
    }
    case CHASSIS_STOP:
    {

        // this->wheels_[0]->setTargetCurrent(0);
        // this->wheels_[1]->setTargetCurrent(0);
        // this->wheels_[2]->setTargetCurrent(0);
        this->set_ControlMode(CURRENT_ZERO_MODE);
        this->set_Target({0, 0, 0});
        break;
    }

    case CHASSIS_MANUAL_CONTROL_C:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        target_chassis_twist_.yaw_rate = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        this->set_Target(target_chassis_twist_);
        break;
    }
    default:
    {
        break;
    }
    }

    // 接收一次雷达数据打印一次

#if debug_ladar

    if (Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data() > last_cout_ladar_data)
    {
        debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);
        last_cout_ladar_data = Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data();
    }

#endif
    // debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);

    // debug_uart.printf_DMA("%f,%f,%f,%f\r\n",
    //                       target_yaw_,yaw,target_chassis_twist_.yaw_rate,dyaw);

    this->update();

    Point2D fk_speed;
    fk_speed.x = this->getWorldSpeed().vx;
    fk_speed.y = this->getWorldSpeed().vy;

    SpeedFK_Queue.send(fk_speed);
}

/////////////////////////////////    路径纠偏代码   //////////////////////////////////////////////

/**
 * @brief 整合已有接口，获取“最近点坐标”和“对应的t值”
 * @param robotPos 输入：机器人当前实际位置（闭环核心输入）
 * @param tNearest 输出：最近点对应的曲线参数t（0~1），给后续找前视点用
 * @return Vector2D 输出：最近点的坐标（给后续算横向偏差用）
 */
Vector2D OmniChassis_Setup::GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest)
{
    // 第一步：调用你的Get_Nearest_Distance，拿到tNearest（最近点对应的t值）
    // 重点：第二个参数传 &tNearest（tNearest的地址），因为你的函数是“输出参数”（通过指针赋值）
    path_.Get_Nearest_Distance(robotPos, &tNearest);

    // 第二步：用第一步拿到的tNearest，调用你的Get_Point，拿到最近点坐标
    Vector2D nearestPt = path_.Get_Point(tNearest);

    // 第三步：返回最近点坐标，给后续“算横向偏差”用
    return nearestPt;
}

// 函数作用：输入最近点的编号tNearest，输出前视点坐标和它的编号tLookahead
Vector2D OmniChassis_Setup::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
{
    // -------------- 对应第1步：初始化，从最近点开始 --------------
    tLookahead = tNearest;        // 前视点的编号，先从最近点的编号开始（比如t=0.3）
    float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
    float step = 0.01f;           // 每次挪的“小步子”

    // 拿到最近点的坐标（比如(5.2, 6.1)），作为“挪步”的起点
    Vector2D lastPt = path_.Get_Point(tLookahead);

    // -------------- 对应第2步：小步慢挪，直到累计距离够前视距离 --------------
    // 条件：1. 编号t没到终点（<1.0）；2. 累计距离还没到前视距离（<0.4m）
    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
    {
        // 1. 往前挪一小步：t增加0.005（比如0.3→0.305）
        float nextT = tLookahead + step;
        // 防止挪超终点：如果nextT>1.0，就改成1.0（不能超出曲线）
        if (nextT > 1.0f)
        {
            nextT = 1.0f;
        }

        // 2. 拿到这一步挪到的点的坐标（比如t=0.305对应的曲线点(5.22, 6.11)）
        Vector2D nextPt = path_.Get_Point(nextT);

        // 3. 计算这一步走了多远（比如从(5.2,6.1)到(5.22,6.11)，距离≈0.022m）
        float distStep = (nextPt - lastPt).magnitude();

        // 4. 累计距离：把这一步的距离加进去（比如0+0.022=0.022m）
        accumulatedDist += distStep;

        // 5. 更新：准备下一步挪步（把当前点当起点，当前t当下一步的基础）
        tLookahead = nextT; // 编号更新
        lastPt = nextPt;    // 起点更新为(5.22,6.11)
    }

    // -------------- 对应第3步：如果到终点了，直接用终点当前视点 --------------
    if (tLookahead >= 1.0f)
    {
        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
    }

    // -------------- 返回前视点坐标 --------------
    return lastPt;
}

/**
 * @brief 计算横向偏差（带方向：正=偏左，负=偏右，单位：m）
 * @param robotPos 机器人当前位置（主函数的m_robotPos）
 * @param nearestPt 曲线最近点（主函数的nearestPt，即P(t')）
 * @param tLookahead 前视点t值（主函数的tLookahead，用于获取稳定切向量）
 * @return float 纯横向偏差（无前后干扰，直接给PID用）
 */
float OmniChassis_Setup::CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead)
{
    // 步骤1：计算原始偏差向量 Δp = 机器人位置 - 最近点（你的定义：Δp = p - p(t')）
    Vector2D delta_p = robotPos - nearestPt;

    // 步骤2：获取前视点的切向量（和主函数一致，确保前进方向基准统一）
    // 主函数里已经调用过一次，但这里再调用一次，保证偏差计算和前进方向完全同步
    lookaheadTangent = path_.Get_Tangent_Vector(tLookahead);

    // 步骤4：定义“横向方向”：垂直于前视点切向量（左转90度，和主函数corrDir方向一致）
    // 主函数纠偏方向是 corrDir = (-lookaheadTangent.y, lookaheadTangent.x)，这里横向方向和它保持一致
    Vector2D lateral_dir = Vector2D(-lookaheadTangent.y, lookaheadTangent.x);
    // 横向方向也归一化：确保点积计算的偏差单位是“米”（无缩放干扰）
    lateral_dir.normalize();

    // 步骤5：核心：计算原始偏差Δp在“横向方向”的投影 → 纯横向偏差
    // 点积公式：delta_p · lateral_dir = |delta_p| * cosθ（θ是Δp和横向方向的夹角）
    // 作用：过滤前后方向干扰（前后方向与横向垂直，cos90°=0），只留左右偏差
    float lateral_err = delta_p * lateral_dir;

    // （可选）调试用：如果发现纠偏方向反了，把偏差乘-1即可
    // lateral_err *= -1;

    return lateral_err;
}
void OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int cho = 0;

    Point2D temp;
    temp.x = robot_pos_.x;
    temp.y = robot_pos_.y;
    if (robot_pos_.x < -0.5f || robot_pos_.x > 5.2f || robot_pos_.y < -0.5f || robot_pos_.y > 8.3f)
    {
        flag_run = 0;
        cho = 0;
    }
    else if (robot_pos_.y < 2.0f)
    {
        cho = 1;
    }
    else
    {
        //cho = 2;
    }

    if (cho == 1)
    {
        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(temp, KFS, 0, 26);
        int point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.entranceMap, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 )
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        int KFS_next_index=index;
        
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);

        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // 修改车子朝向
        if (abs(path_key_point_[KFS_next_index] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
    else if (cho == 2)
    {

        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(Point2D{temp.x, 1.2f, 0.0f}, KFS, 0, 26);
        temp.x += 0.5f;
        temp.y += 0.5f;
        point_map = MF_AutoCtrler::GetMapNumFromPos(temp);
        // 将车子开到空旷地带的路口
        int point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        // 将车子开到kfs前
        //        point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        //        for (int i = 0; i < point_sum; i++)
        //        {
        //            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
        //            {
        //                path_key_point_[index] = path_point_[i];
        //                index++;
        //            }
        //        }
        // 将车子开到斜坡前
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // 修改车子朝向
        if (abs(path_key_point_[0] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
}

void OmniChassis_Setup::Path_correction(void)
{
    // 获取曲线（带保护）
    BezierCurve &curve = path_line_.get_bezier_curve();

    pathEnd = curve.Get_Point(1.0f);
    // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest
    nearestPt = GetPathNearestPoint(curve, robot_pos_, tNearest);

    // 终点纠偏补丁：如果非常接近终点，直接使用终点位置吸附
    // tNearest > 0.99 表示基本到了终点，或者 Is_End()==false 表示规划已结束
    if (tNearest > 0.99f || path_line_.Is_End() == false)
    {
        Vector2D endPt = curve.Get_End_point();
        // 如果曲线未初始化（例如空曲线），不进行操作
        if (endPt.magnitude() < 0.0001f && curve.Get_Start_point().magnitude() < 0.0001f)
        {
            speed = planspeed; // 保持原有速度（通常是0）
            return;
        }

        Vector2D errorVec = endPt - robot_pos_;

        // 终点吸附增益，可以根据需要调整，相当于位置环 P 参数
        float final_kp = 2.0f;
        corrVelocity = errorVec * final_kp;

        // 限制最大纠偏速度，防止终点抖动
        float max_corr = 0.5f;
        if (corrVelocity.magnitude() > max_corr)
        {
            corrVelocity = corrVelocity.normalize() * max_corr;
        }

//        speed = planspeed + corrVelocity; // 叠加到规划速度上
        return;
    }

    // 2. 找前视点+前进方向：根据最近点和前视距离，寻找前视点及其参数 tLookahead
    lookaheadPt = FindLookaheadPoint(curve, tNearest, tLookahead);
    lookaheadTangent = curve.Get_Tangent_Vector(tLookahead);
    // 3. 计算横向偏差：计算机器人当前位置到路径切线的垂直距离
    lateralError = CalculateLateralError(curve, robot_pos_, nearestPt, tLookahead);
    // 4. 横向偏差PID控制：计算横向纠偏速度大小
    correctspeed = pid_track.pid_calc(0.0f, lateralError);
    Vector2D corrDir(-lookaheadTangent.y, lookaheadTangent.x); // 纠偏方向（垂直前进方向，左右纠偏）
    corrVelocity = corrDir * correctspeed;                     // 合成纠偏速度（方向+大小）

    // baseVelocity = lookaheadTangent * planspeed.magnitude();
    
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_1);
   path_line_.Add_Point(Vector2D{1.8f, 0.8f});
   path_line_.Add_End_Point(Clamping_Bar_Selection_pos_);
    // path_line_.Add_End_Point(Vector2D{3.92f, 1.38f});
}