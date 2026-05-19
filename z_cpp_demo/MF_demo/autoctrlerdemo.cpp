/**
 * @file autoctrlerdemo.cpp
 * @author XieFField
 * @brief 梅花林自动测试程序 (包含 get_GimbalMF_PAPB 移植测试)
 *        里面不少是用哈基米3生成的，把正式代码移植到demo是真好用
 */

#include "demo_AutoCtrler.h"
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;
using namespace MF_AutoCtrler;

// 定义 PI
#ifndef PI
#define PI 3.1415926535f
#endif

// 模拟工具函数
float _tool_Abs(float x) { return std::abs(x); }

typedef enum {
    ROTATE_PATH_SHORTEST,   // 最短路径 (默认)
    ROTATE_PATH_POSITIVE,    // 指定正方向旋转
    ROTATE_PATH_NEGATIVE     // 指定负方向旋转
} Rotate_Strategy_E;

// KFS 高度表 (cm)
const float MF_high[12] = 
{
    40.0f, 20.0f, 40.0f,
    20.0f, 40.0f, 60.0f,
    40.0f, 60.0f, 40.0f,
    20.0f, 40.0f, 20.0f
};

float chassisMoveDir(int8_t startmapNum, int8_t next_mapNum)
{
    if(startmapNum < 1 || startmapNum >30 || next_mapNum < 1 || next_mapNum >30)
    {
        if(startmapNum < 1 || startmapNum >30)
            cout << "Start mapNum " << (int)startmapNum << " is out of range." << endl;

        if(next_mapNum < 1 || next_mapNum >30)
            cout << "Next mapNum " << (int)next_mapNum << " is out of range." << endl;

        return -1.0f; // 无效输入
    }
    
    bool isinMFstart = IsWalkable(startmapNum);
    bool isinMFnext = IsWalkable(next_mapNum);

    if(isinMFstart == false)
    {
        cout << "Start map " << (int)startmapNum << " is in MF." << endl;
    }
    if(isinMFnext == false)
    {
        cout << "Next map " << (int)next_mapNum << " is in MF." << endl;
    }

     

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

// 模拟 ArmSetup 类
class MockArmSetup {
public:
    // 模拟状态变量
    struct {
        Point2D now_armPosition; // 机械臂/底盘位置
        Point2D now_ChassisPosition; // 底盘位置 (包含theta)
        float now_chassis_speed = 1.0f;
        
        int targetKFS[2] = {0, 0};
        Point2D targetKFS_pos[2];
        Direction_E KFS_Movedirection[2];
        PathNode_S path;
        struct { Point2D bestB1, bestBMF1; } pathPos;
        
        int gimbal_calcCount = 0;
        int gimbal_calcHz = 100;
        float arm_width = 0.12f;
        
        // [修改] 修正云台速度为 90度/秒 (约1.57 rad/s)，原公式计算值过大(约16 rad/s)
        struct { float gimbal_max_rad = 90.0f * (PI / 180.0f); } time_set; 
        
        struct { Point2D PA; Point2D PB; } PointPAB[2]; // PA, PB

        Rotate_Strategy_E current_strategy = ROTATE_PATH_SHORTEST; // [新增] 策略
    } auto_ctrl_;

    struct {
        float max_launchHeight_ = 0.4f;
        float max_stretchLength_ = 0.130f;
        float arm_length_ = 0.6f;
        float end_link_length_ = 0.08f;

         float stretch_Ratio_ = 0.08417f;
        float launch_Ratio_ = 0.07221f;
        float rotate_gearRatio_ = 144.878f;
        float pitch_gearRatio_ = 360.0f;

        float min_rotate_angle_ = 0.0f;
        float max_rotate_angle_ = 359.999f;
    } init_data_;

    // 模拟电机状态
    float current_rotate_angle = 0.0f; // 当前云台角度
    float current_launch_height = 0.0f; // 当前升降高度
    bool sucker_is_open = false;       // 吸盘状态

    // 新增标志位用于打印
    bool has_printed_trigger = false;
    bool has_printed_rotate = false;
    bool has_printed_safe = false;     // 新增：Safe时刻
    bool has_printed_finish = false;   // 新增：完成旋转时刻

    // Carrying 阶段标志
    bool has_printed_carrying_start = false;
    bool has_printed_carrying_lift = false;
    bool has_printed_carrying_place = false;
    bool has_printed_carrying_done = false;
    
    // [新增] 更加详细的事件标志位
    bool flag_safe_rotate = false;
    bool flag_rotate_done = false; // [新增] 旋转完成标志
    bool flag_pickup = false;
    bool flag_safe_carry = false;
    bool flag_safe_range = false;
    bool flag_storage_done = false;

    // [新增] 统一打印函数
    void print_event(const char* event_name, float time) {
        printf(">>> [%s] T=%.2f | Pos=(%.3f, %.3f) | Ang=%.1f | H=%.3f | Sucker=%d\n", 
            event_name, 
            time,
            auto_ctrl_.now_armPosition.x, auto_ctrl_.now_armPosition.y, 
            current_rotate_angle, current_launch_height, sucker_is_open);
    }

    // 初始化仿真场景
    void init_simulation(int targetKFS_Index) 
    {
        auto_ctrl_.targetKFS[0] = targetKFS_Index;
        auto_ctrl_.targetKFS_pos[0] = MapNum_RealPos[MF_AutoCtrler::MFNum_TransforMapNum(targetKFS_Index) - 1];
        
        // 【修复关键点】修改计算频率以匹配仿真步长(10ms)
        auto_ctrl_.gimbal_calcHz = 1000; 

        // 假设机器人初始位置在 (0,0)，计算路径
        Point2D startPos = {0,0,0}; 
        auto_ctrl_.path = PathNodeResult_calc(startPos, static_cast<int8_t>(targetKFS_Index), 0);
        
        // 设置路径点世界坐标
        auto_ctrl_.pathPos.bestB1 = MapCenterWorld(auto_ctrl_.path.bestB1);
        auto_ctrl_.pathPos.bestBMF1 = MapCenterWorld(auto_ctrl_.path.bestBMF1);
        
        auto_ctrl_.now_armPosition = auto_ctrl_.pathPos.bestB1; // 直接跳到最佳点的后1.2m处

        // 获取移动方向
        get_MoveDiretion(startPos, static_cast<int8_t>(targetKFS_Index), 0, auto_ctrl_.KFS_Movedirection);
        
        float start_offset = 1.2f;
        switch(auto_ctrl_.KFS_Movedirection[0]) {
            case Positive_X: auto_ctrl_.now_armPosition.x -= start_offset; break;
            case Negative_X: auto_ctrl_.now_armPosition.x += start_offset; break;
            case Positive_Y: auto_ctrl_.now_armPosition.y -= start_offset; break;
            case Negative_Y: auto_ctrl_.now_armPosition.y += start_offset; break;
            default: break;
        }
        
        // 设置底盘Yaw (假设沿移动方向)
        switch(auto_ctrl_.KFS_Movedirection[0]) {
            case Positive_X: auto_ctrl_.now_ChassisPosition.theta = 270.0f; break; 
            case Negative_X: auto_ctrl_.now_ChassisPosition.theta = 90.0f; break;
            case Positive_Y: auto_ctrl_.now_ChassisPosition.theta = 0.0f; break;
            case Negative_Y: auto_ctrl_.now_ChassisPosition.theta = 180.0f; break;
            default: break;
        }
        
        // 设置云台初始角度 (假设初始为0度)
        current_rotate_angle = 0.0f;
        
        // [新增] 设置初始高度：KFS高度 - 20cm
        float kfs_h_cm = MF_high[targetKFS_Index - 1];
        current_launch_height = (kfs_h_cm - 20.0f) / 100.0f; // 转换为米
        if(current_launch_height < 0) current_launch_height = 0;

        // 重置标志位
        has_printed_trigger = false;
        has_printed_rotate = false;
        has_printed_safe = false;
        has_printed_finish = false;
        
        has_printed_carrying_start = false;
        has_printed_carrying_lift = false;
        has_printed_carrying_place = false;
        has_printed_carrying_done = false;

        flag_safe_rotate = false;
        flag_rotate_done = false; // [新增] 复位
        flag_pickup = false;
        flag_safe_carry = false;
        flag_safe_range = false;
        flag_storage_done = false;
        
        // cout << "Sim Init: Target KFS=" << targetKFS_Index << endl;
        // cout << "Path: B1(" << auto_ctrl_.pathPos.bestB1.x << "," << auto_ctrl_.pathPos.bestB1.y << ") -> ";
        // cout << "BMF1(" << auto_ctrl_.pathPos.bestBMF1.x << "," << auto_ctrl_.pathPos.bestBMF1.y << ")" << endl;
        // cout << "Direction: " << auto_ctrl_.KFS_Movedirection[0] << endl;
        // cout << "Gimbal Speed: " << auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI << " deg/s" << endl; // [新增] 打印确认速度
        
        // 1. 开始仿真的时刻
        print_event("Simulation Start", 0.0f);
    }

    // 模拟硬件接口
    void set_RotateAngle(float angle) {
        float diff = angle - current_rotate_angle;
        while(diff > 180) diff -= 360;
        while(diff < -180) diff += 360;
        
        float max_step = auto_ctrl_.time_set.gimbal_max_rad * (180.0f/PI) * 0.01f; // 10ms步长
        if(abs(diff) > max_step) {
            current_rotate_angle += (diff > 0 ? 1 : -1) * max_step;
        } else 
            current_rotate_angle = angle;
        
        while(current_rotate_angle > 360) current_rotate_angle -= 360;
        while(current_rotate_angle < 0) current_rotate_angle += 360;
    }

    void set_LaunchHeight(float height) {
        float max_speed = 0.5f; // m/s
        float step = max_speed * 0.01f;
        if (current_launch_height < height) {
            current_launch_height += step;
            if(current_launch_height > height) current_launch_height = height;
        } else if (current_launch_height > height) {
            current_launch_height -= step;
            if(current_launch_height < height) current_launch_height = height;
        }
    }

    void setSuckerStatus(int status) 
    {
        if(status == 1) 
        {
            if(!sucker_is_open) cout << endl << ">>> [ACTION] Sucker OPENED (吸盘打开)! <<<" << endl;
            sucker_is_open = true;
        }
        else if (status == 0)
        {
            if(sucker_is_open) cout << endl << ">>> [ACTION] Sucker CLOSED (吸盘关闭)! <<<" << endl;
            sucker_is_open = false;
        }
    }

    void setRotateStrategy(Rotate_Strategy_E strategy) 
    {
        auto_ctrl_.current_strategy = strategy;
    }
    
    Rotate_Strategy_E getRotateStrategy() 
    {
        return auto_ctrl_.current_strategy;
    }

    struct Joint_Status_S { float rotateJoint_angle_; float launchJoint_Height_; };
    Joint_Status_S get_currentJointStatus() {
        return {current_rotate_angle, current_launch_height};
    }

    // 核心逻辑：移植自 Arm_Setup.cpp
    bool check_Arm_collision(float px, float py, float pivot_x, float pivot_y, float arm_world_angle_deg, float L_arm, float W_arm)
    {
        float angle_rad = arm_world_angle_deg * (PI / 180.0f);
        float c = cosf(angle_rad);
        float s = sinf(angle_rad);
        Point2D d = { px - pivot_x, py - pivot_y, 0.0f };
        
        // [修正] 正确的坐标变换：将世界坐标投影到机械臂局部坐标系
        // Local X: 沿机械臂轴向 (点乘方向向量 (c, s))
        // Local Y: 垂直机械臂轴向 (点乘法向量 (-s, c))
        Point2D local = {
             d.x * c + d.y * s, // Local X
            -d.x * s + d.y * c, // Local Y
             0.0f
        };
        
        // 矩形碰撞判断: x in [0, L], y in [-W/2, W/2]
        if(local.x >= 0.0f && local.x <= L_arm && _tool_Abs(local.y) <= (W_arm / 2.0f))
            return true; 
        return false; 
    }

    // 核心逻辑：移植自 Arm_Setup.cpp (state_signAlign)
    // [修改] 增加 current_time 参数用于打印
    void state_signAlign(int targetKFS, float current_time)
    {
        Direction_E move_direction;
        Point2D target_pos = {0, 0 ,0}; // [新增]
        
        if(targetKFS == auto_ctrl_.targetKFS[0]) {
            move_direction = auto_ctrl_.KFS_Movedirection[0];
            target_pos = auto_ctrl_.targetKFS_pos[0]; // [新增]
        }
        else return;

        // [新增] 移植 Arm_Setup.cpp 中的启动判断逻辑
        // 只有到达 bestB1 附近才开始计算，防止过早旋转
        switch(move_direction) 
        {
            case Positive_X:
                if(auto_ctrl_.now_armPosition.x < auto_ctrl_.pathPos.bestB1.x - 0.1f) return; 
                break;
            case Negative_X:
                if(auto_ctrl_.now_armPosition.x > auto_ctrl_.pathPos.bestB1.x + 0.1f) return; 
                break;
            case Positive_Y:
                if(auto_ctrl_.now_armPosition.y < auto_ctrl_.pathPos.bestB1.y - 0.1f) return; 
                break;
            case Negative_Y:
                if(auto_ctrl_.now_armPosition.y > auto_ctrl_.pathPos.bestB1.y + 0.1f) return; 
                break;
            default: break;
        }

        auto_ctrl_.gimbal_calcCount++;
        if(auto_ctrl_.gimbal_calcCount < 1000.0f/ static_cast<float>(auto_ctrl_.gimbal_calcHz))
            return; 
        auto_ctrl_.gimbal_calcCount = 0;

        int index = 0;
        get_GimbalMF_PAPB(targetKFS, auto_ctrl_.PointPAB[index].PA, auto_ctrl_.PointPAB[index].PB);
        Point2D PA = auto_ctrl_.PointPAB[index].PA;
        Point2D PB = auto_ctrl_.PointPAB[index].PB;

        float vx = 0.0f, vy = 0.0f;
        switch(move_direction) {
            case Positive_X: vx = auto_ctrl_.now_chassis_speed; break;
            case Negative_X: vx = -auto_ctrl_.now_chassis_speed; break;
            case Positive_Y: vy = auto_ctrl_.now_chassis_speed; break;
            case Negative_Y: vy = -auto_ctrl_.now_chassis_speed; break;
            default: break;
        }

        float current_deg = current_rotate_angle;
        float target_deg = 90.0f;
        float diff = target_deg - current_deg;

        if(diff > 180.0f) diff -= 360.0f;
        else if(diff < -180.0f) diff += 360.0f;

        float T_rot = _tool_Abs(diff) * (PI / 180.0f) / (auto_ctrl_.time_set.gimbal_max_rad * 0.8);
        bool safe = true;

        for(float t = 0.0f; t <= T_rot; t+= 0.05f)
        {
            Point2D pivot;
            pivot.x = auto_ctrl_.now_armPosition.x + vx * t;
            pivot.y = auto_ctrl_.now_armPosition.y + vy * t;
            pivot.theta = 0.0f;

            float step_deg = 0.0f;
            if(diff > 0 ) step_deg = 1.0f * (auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI) * t;
            else step_deg = -1.0f * (auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI) * t;

            if(_tool_Abs(step_deg) > _tool_Abs(diff)) step_deg = diff;
            float gimbal_angle_t  = current_deg + step_deg;
            float world_angle_t = Get_ArmWorldAngle(auto_ctrl_.now_ChassisPosition.theta, gimbal_angle_t);

            if(check_Arm_collision(PA.x, PA.y, pivot.x, pivot.y, world_angle_t, auto_ctrl_.arm_width, auto_ctrl_.arm_width) ||
               check_Arm_collision(PB.x, PB.y, pivot.x, pivot.y, world_angle_t, auto_ctrl_.arm_width, auto_ctrl_.arm_width))
            {
                safe = false;
                break;
            }
        }

        // 选择并锁定策略
        if(diff > 0) auto_ctrl_.current_strategy = ROTATE_PATH_POSITIVE;
        else if (diff < 0) auto_ctrl_.current_strategy = ROTATE_PATH_NEGATIVE;
        else auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
        
        this->setRotateStrategy(auto_ctrl_.current_strategy);

        if(safe)
        {
            // 2. safe to rotate时刻
            if(!flag_safe_rotate) { 
                print_event("Safe to Rotate", current_time); 
                flag_safe_rotate = true; 
            }
            
            this->set_RotateAngle(target_deg);

            // [新增] 完成rotate时刻
            if(!flag_rotate_done && _tool_Abs(current_rotate_angle - target_deg) < 0.1f) {
                print_event("Rotate Complete", current_time);
                flag_rotate_done = true;
            }
            
            // [修改] 吸盘触发逻辑：检查末端坐标是否与 KFS 中心重合
            Point2D kfs_pos = auto_ctrl_.targetKFS_pos[0];
            float dist_err = 0.0f;
            
            // 根据移动方向判断轴向距离
            switch(move_direction) {
                case Positive_X: case Negative_X:
                    dist_err = _tool_Abs(auto_ctrl_.now_armPosition.x - kfs_pos.x);
                    break;
                case Positive_Y: case Negative_Y:
                    dist_err = _tool_Abs(auto_ctrl_.now_armPosition.y - kfs_pos.y);
                    break;
                default: break;
            }
            
            // 死区判断 (例如 2cm)
            if(dist_err < 0.02f) {
                if(!sucker_is_open) {
                    this->setSuckerStatus(1); // SUCK
                    // 4. 对准KFS完成拾取的时刻
                    if(!flag_pickup) {
                        print_event("Picked Up (Aligned)", current_time);
                        flag_pickup = true;
                    }
                }
            }
        }
        else
        {
            this->set_RotateAngle(current_deg); 
        }
    }

    // 核心逻辑：移植自 Arm_Setup.cpp (state_carrying)
    void state_carrying(int targetKFS, float current_time)
    {
        // if(!has_printed_carrying_start) { cout << ">>> [Carrying] Start Carrying Logic (开始搬运逻辑)..." << endl; has_printed_carrying_start = true; }

        Direction_E move_direction;
        if(targetKFS == auto_ctrl_.targetKFS[0]) 
            move_direction = auto_ctrl_.KFS_Movedirection[0];
        else 
            return;

        int index = 0;
        get_GimbalMF_PAPB(targetKFS, auto_ctrl_.PointPAB[index].PA, auto_ctrl_.PointPAB[index].PB);
        Point2D PA = auto_ctrl_.PointPAB[index].PA;
        Point2D PB = auto_ctrl_.PointPAB[index].PB;

        float vx = 0.0f, vy = 0.0f;
        switch(move_direction) 
        {
            case Positive_X: vx = auto_ctrl_.now_chassis_speed; break;
            case Negative_X: vx = -auto_ctrl_.now_chassis_speed; break;
            case Positive_Y: vy = auto_ctrl_.now_chassis_speed; break;
            case Negative_Y: vy = -auto_ctrl_.now_chassis_speed; break;
            default: break;
        }

        float current_deg = current_rotate_angle;
        float target_deg = 0.0f; 

        Rotate_Strategy_E strategy = this->getRotateStrategy();

        float diff = 0.0f;
        float current_mod = fmodf(current_deg, 360.0f);
        if(current_mod < 0) 
            current_mod += 360.0f;
        float target_mod = 0.0f;
        float raw_diff = target_mod - current_mod;

        switch(strategy)
        {
            case ROTATE_PATH_POSITIVE:
                if(raw_diff <= 0.0f) diff = raw_diff + 360.0f; else diff = raw_diff; break;
            case ROTATE_PATH_NEGATIVE:
                if(raw_diff >= 0.0f) diff = raw_diff - 360.0f; else diff = raw_diff; break;
            case ROTATE_PATH_SHORTEST:
                diff = raw_diff;
                if(diff > 180.0f) diff -= 360.0f; else if(diff < -180.0f) diff += 360.0f;
                break;
        }

        float kfs_size = 0.35f; 
        float check_L = init_data_.arm_length_ + kfs_size + init_data_.end_link_length_;
        float check_W = (auto_ctrl_.arm_width > kfs_size) ? auto_ctrl_.arm_width : kfs_size;

        bool safe = true;
        float T_rot = _tool_Abs(diff) * (PI / 180.0f) / (auto_ctrl_.time_set.gimbal_max_rad * 0.8);

        for(float t = 0.0f; t <= T_rot; t+= 0.05f)
        {
            Point2D pivot{
                 .x = auto_ctrl_.now_armPosition.x + vx * t,
                 .y = auto_ctrl_.now_armPosition.y + vy * t,
                 .theta = 0.0f
            };

            float step_deg = 0.0f;
            if(diff > 0 ) step_deg = 1.0f * (auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI) * t;
            else step_deg = -1.0f * (auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI) * t;

            if(_tool_Abs(step_deg) > _tool_Abs(diff)) step_deg = diff;
            if(_tool_Abs(step_deg) > 90.0f) break; // 优化：超过90度视为安全

            float gimbal_angle_t  = current_deg + step_deg;
            float world_angle_t = Get_ArmWorldAngle(auto_ctrl_.now_ChassisPosition.theta, gimbal_angle_t);

            if(check_Arm_collision(PA.x, PA.y, pivot.x, pivot.y, world_angle_t, check_L, check_W) ||
               check_Arm_collision(PB.x, PB.y, pivot.x, pivot.y, world_angle_t, check_L, check_W))
            {
                safe = false;
                break;
            }
        }

        if(safe)
        {
            // 5. safe to carry的時刻
            if(!flag_safe_carry) 
            {
                print_event("Safe to Carry", current_time);
                flag_safe_carry = true;
            }

            this->set_RotateAngle(target_deg);

            float diff_from_KFS = current_deg - 90.0f;
            while(diff_from_KFS > 180.0f) diff_from_KFS -= 360.0f;
            while(diff_from_KFS < -180.0f) diff_from_KFS += 360.0f;

            float storage_height = 0.0f; 
            float safe_height = 0.15f;   

            if(_tool_Abs(diff_from_KFS) > 120.0f)
            {
                 // 6. 到達安全範圍的時刻
                 if(!flag_safe_range) 
                 {
                     if(current_launch_height < safe_height) 
                     {
                         print_event("Safe Range - Lifting", current_time);
                     } 
                     else 
                     {
                         print_event("Safe Range - No Lift", current_time);
                     }
                     flag_safe_range = true;
                 }

                 if(current_launch_height < safe_height)
                 {
                     this->set_LaunchHeight(safe_height);
                 }
            }

            static int place_state = 0;
            static float wait_start_time = 0.0f;

            if(_tool_Abs(diff) > 5.0f) place_state = 0;

            if(_tool_Abs(diff) < 2.0f)
            {
                if(place_state == 0)
                {
                    this->set_LaunchHeight(storage_height);
                    wait_start_time = current_time;
                    place_state = 1;
                    // if(!has_printed_carrying_place) { cout << ">>> [Carrying] Arrived 0 deg. Lowering to storage (到达0度，下降至存储位)..." << endl; has_printed_carrying_place = true; }
                }
                else if(place_state == 1)
                {
                    this->set_LaunchHeight(storage_height); 
                    if(current_time - wait_start_time > 0.2f)
                    {
                        this->setSuckerStatus(0); // STOP
                        place_state = 2;
                    }
                }
                else if(place_state == 2)
                {
                    this->set_LaunchHeight(safe_height);
                    // 7. 完成儲存的時刻
                    if(!flag_storage_done) {
                        print_event("Storage Complete", current_time);
                        flag_storage_done = true;
                    }
                }
            }
        }
        else
        {
            this->set_RotateAngle(current_deg); 
        }
    }


    // 辅助函数：计算 PA PB (动态计算版)
    void get_GimbalMF_PAPB(int target_KFSIndex, Point2D& PA, Point2D& PB) 
    {
        // 1. 获取 KFS 中心坐标
        // [修改] 关键修复：将 MF编号 转换为 Map编号
        int8_t mapNum = MFNum_TransforMapNum((int8_t)target_KFSIndex);
        Point2D KFS_Pos = MapCenterWorld(mapNum);
        
        // 2. 获取参考点 (路径起点 B1)
        Point2D Robot_Pos = auto_ctrl_.pathPos.bestB1; 
        
        // 3. 单元格半宽 (1.2m / 2 = 0.6m)
        float half_cell = 0.6f; 
        
        // 4. 根据运动方向和相对位置确定障碍物面
        Direction_E dir = auto_ctrl_.KFS_Movedirection[0];
        
        // [修改] 逻辑修正：PA 必须是运动方向上先遇到的点 (Near Corner)
        if (dir == Positive_X || dir == Negative_X) 
        {
            // 水平运动，比较 Y 坐标
            if (KFS_Pos.y > Robot_Pos.y) 
            {
                // KFS 在上方 (North)，底盘在下方通过 -> 障碍面是 KFS 下表面
                // PA/PB Y坐标都是 Bottom (y - half)
                // X坐标取决于运动方向
                float common_y = KFS_Pos.y - half_cell;
                if (dir == Positive_X) 
                {
                    PA.x = KFS_Pos.x - half_cell; PA.y = common_y; // Left-Bottom
                    PB.x = KFS_Pos.x + half_cell; PB.y = common_y; // Right-Bottom
                } 
                else 
                { // Negative_X
                    PA.x = KFS_Pos.x + half_cell; PA.y = common_y; // Right-Bottom
                    PB.x = KFS_Pos.x - half_cell; PB.y = common_y; // Left-Bottom
                }
            } 
            else 
            {
                // KFS 在下方 (South) -> 障碍面是 KFS 上表面
                float common_y = KFS_Pos.y + half_cell;
                if (dir == Positive_X) {
                    PA.x = KFS_Pos.x - half_cell; PA.y = common_y; // Left-Top
                    PB.x = KFS_Pos.x + half_cell; PB.y = common_y; // Right-Top
                } else { // Negative_X
                    PA.x = KFS_Pos.x + half_cell; PA.y = common_y; // Right-Top
                    PB.x = KFS_Pos.x - half_cell; PB.y = common_y; // Left-Top
                }
            }
        } 
        else
        {
            // 垂直运动，比较 X 坐标
            if (KFS_Pos.x > Robot_Pos.x) 
            {

                // KFS 在右侧 (East) -> 障碍面是 KFS 左表面
                float common_x = KFS_Pos.x - half_cell;
                if (dir == Positive_Y) {
                    PA.x = common_x; PA.y = KFS_Pos.y - half_cell; // Left-Bottom
                    PB.x = common_x; PB.y = KFS_Pos.y + half_cell; // Left-Top
                } else { // Negative_Y
                    PA.x = common_x; PA.y = KFS_Pos.y + half_cell; // Left-Top
                    PB.x = common_x; PB.y = KFS_Pos.y - half_cell; // Left-Bottom
                }
            } 
            else 
            {
                // KFS 在左侧 (West) -> 障碍面是 KFS 右表面
                float common_x = KFS_Pos.x + half_cell;
                if (dir == Positive_Y) 
                {
                    PA.x = common_x; PA.y = KFS_Pos.y - half_cell; // Right-Bottom
                    PB.x = common_x; PB.y = KFS_Pos.y + half_cell; // Right-Top
                } 
                else 
                { // Negative_Y
                    PA.x = common_x; PA.y = KFS_Pos.y + half_cell; // Right-Top
                    PB.x = common_x; PB.y = KFS_Pos.y - half_cell; // Left-Bottom
                }
            }
        }
    }
};

// ==================================================================================
// 4. 主函数
// ==================================================================================

static float CalcPathInfoCost(Point2D robotPos, const PathInformation_S &path)
{
    if (path.entranceMap == 0 || path.MFroad[0] == 0)
        return 1.0e9f;

    int sE1 = BFS_Steps(path.entranceMap, path.MFroad[0]);
    if (sE1 >= BFS_INF)
        return 1.0e9f;

    float dRobotToE = euclid(robotPos, MapCenterWorld(path.entranceMap));

    if (path.MFroad[1] == 0)
    {
        int s1X = BFS_Steps(path.MFroad[0], path.exitMap);
        if (s1X >= BFS_INF)
            return 1.0e9f;
        return dRobotToE + CELL_M * (float)(sE1 + s1X);
    }

    int s12 = BFS_Steps(path.MFroad[0], path.MFroad[1]);
    int s2X = BFS_Steps(path.MFroad[1], path.exitMap);
    if (s12 >= BFS_INF || s2X >= BFS_INF)
        return 1.0e9f;

    return dRobotToE + CELL_M * (float)(sE1 + s12 + s2X);
}

static float BruteForceBestCost(Point2D robotPos, int8_t MF1, int8_t MF2)
{
    RoadResult_S MF1Road = MFNum_ToCatchRoadResult(MF1);
    RoadResult_S MF2Road = MFNum_ToCatchRoadResult(MF2);
    int8_t roadMF1[2] = {MF1Road.result1, MF1Road.result2};
    int8_t roadMF2[2] = {MF2Road.result1, MF2Road.result2};

    int8_t entrances[30] = {0};
    uint8_t entranceCount = 0;
    int8_t robotMap = GetMapNumFromPos(robotPos);
    bool isRobotInsideMap = (robotMap >= 1 && robotMap <= 30 && IsWalkable(robotMap));

    if (isRobotInsideMap)
    {
        entrances[entranceCount++] = robotMap;
    }
    else
    {
        bool isBelow = (robotPos.y < MapNum_RealPos[0].y);
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
        return 1.0e9f;

    bool hasMF2 = (MF2 != 0 && (roadMF2[0] != 0 || roadMF2[1] != 0));
    float bestCost = 1.0e9f;

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
                int s1X = BFS_Steps(R1, 26);
                if (s1X >= BFS_INF)
                    continue;

                float J = dRobotToE + CELL_M * (float)(sE1 + s1X);
                if (J < bestCost)
                    bestCost = J;
            }
            else
            {
                for (int i2 = 0; i2 < 2; ++i2)
                {
                    int8_t R2 = roadMF2[i2];
                    if (R2 == 0)
                        continue;

                    int s12 = BFS_Steps(R1, R2);
                    int s2X = BFS_Steps(R2, 26);
                    if (s12 >= BFS_INF || s2X >= BFS_INF)
                        continue;

                    float J = dRobotToE + CELL_M * (float)(sE1 + s12 + s2X);
                    if (J < bestCost)
                        bestCost = J;
                }
            }
        }
    }

    return bestCost;
}

static void PrintMustPast(const PathInformation_S &path)
{
    cout << "  mustPastMap: ";
    for (int i = 0; i < 12; ++i)
    {
        if (path.mustPastMap[i] == 0)
            break;
        cout << (int)path.mustPastMap[i] << " ";
    }
    cout << endl;
}

int main(void)
{
    struct TestCase
    {
        const char *name;
        Point2D robotPos;
        int8_t MF1;
        int8_t MF2;
    };

    TestCase tests[] = {
        {"Case-1 单目标 林外下方", {0.2f, 0.8f, 0.0f}, 11, 0},
        {"Case-2 单目标 林外上方", {5.5f, 9.6f, 0.0f}, 10, 0},
        {"Case-3 双目标 林外下方", {1.0f, 0.5f, 0.0f}, 4, 9},
        {"Case-4 双目标 林内通道", {0.6f, 5.0f, 0.0f}, 6, 11}};

    bool allPass = true;

    cout << "=== PathInformation 最优性测试开始 ===" << endl;

    int testCount = (int)(sizeof(tests) / sizeof(tests[0]));
    for (int i = 0; i < testCount; ++i)
    {
        TestCase &tc = tests[i];
        PathInformation_S info = PathInformation_calc(tc.robotPos, tc.MF1, tc.MF2);
        float calcCost = CalcPathInfoCost(tc.robotPos, info);
        float bruteCost = BruteForceBestCost(tc.robotPos, tc.MF1, tc.MF2);

        float err = calcCost - bruteCost;
        if (err < 0.0f)
            err = -err;

        bool calcHasPath = (info.entranceMap != 0 && info.MFroad[0] != 0);
        bool bruteHasPath = (bruteCost < 1.0e8f);
        bool pass = false;

        if (!calcHasPath && !bruteHasPath)
            pass = true;
        else if (calcHasPath && bruteHasPath)
            pass = (err < 1.0e-4f);
        else
            pass = false;
        if (!pass)
            allPass = false;

        cout << "\n[" << tc.name << "]" << endl;
        cout << "  robotPos=(" << tc.robotPos.x << ", " << tc.robotPos.y << "), MF1=" << (int)tc.MF1 << ", MF2=" << (int)tc.MF2 << endl;
        cout << "  entranceMap=" << (int)info.entranceMap << ", MFroad1=" << (int)info.MFroad[0] << ", MFroad2=" << (int)info.MFroad[1] << ", exitMap=" << (int)info.exitMap << endl;
        cout << "  calcCost=" << calcCost << ", bruteBestCost=" << bruteCost << ", absErr=" << err << endl;
        cout << "MFroad1's index is " << (int)info.Index_MFroad[0] << endl;
        if(info.MFroad[1] != 0)
            cout << "MFroad2's index is " << (int)info.Index_MFroad[1] << endl;
        cout << "  result=" << (pass ? "PASS" : "FAIL") << endl;
        PrintMustPast(info);
    }

    cout << "\n=== PathInformation 最优性测试结束: " << (allPass ? "全部PASS" : "存在FAIL") << " ===" << endl;

    int testDir[4][2] =
    {
        {2,5}, {15,30}, {6,26}, {30,26}
    };

    float testDirresult[4];
    for(int i = 0; i < 4; i ++)
    {
        testDirresult[i] = chassisMoveDir(testDir[i][0], testDir[i][1]);
        cout << "\n[Direction Test " << (i+1) << "] From Map " << testDir[i][0] << " to Map " << testDir[i][1] << ": Direction = " << testDirresult[i] << endl;
    }

    return allPass ? 0 : 1;
}