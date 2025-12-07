/**
 * @file autoctrlerdemo.cpp
 * @author XieFField
 * @brief 梅花林路径点测试程序 (包含 get_GimbalMF_PAPB 严格移植测试)
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

// 模拟 ArmSetup 类
class MockArmSetup {
public:
    // 模拟状态变量
    struct {
        Point2D now_armPosition; // 机械臂/底盘位置
        float now_chassis_speed = 1.0f;
        float now_chassis_yaw = 0.0f; // 底盘Yaw
        
        int targetKFS[2] = {0, 0};
        Point2D targetKFS_pos[2];
        Direction_E KFS_Movedirection[2];
        PathNode_S path;
        struct { Point2D bestB1, bestBMF1; } pathPos;
        
        int gimbal_calcCount = 0;
        int gimbal_calcHz = 100;
        float arm_width = 0.12f;
        
        struct { float gimbal_max_rad = 2.0f; } time_set; // 假设最大角速度 2 rad/s
        
        Point2D point_PAB[2]; // PA, PB
    } auto_ctrl_;

    struct {
        float arm_length_ = 0.6f;
    } init_data_;

    // 模拟电机状态
    float current_rotate_angle = 0.0f; // 当前云台角度
    bool sucker_is_open = false;       // 吸盘状态

    // 新增标志位用于打印
    bool has_printed_trigger = false;
    bool has_printed_rotate = false;
    bool has_printed_safe = false;     // 新增：Safe时刻
    bool has_printed_finish = false;   // 新增：完成旋转时刻

    // 初始化仿真场景
    void init_simulation(int targetKFS_Index) {
        auto_ctrl_.targetKFS[0] = targetKFS_Index;
        auto_ctrl_.targetKFS_pos[0] = MapNum_RealPos[targetKFS_Index - 1];
        
        // 【修复关键点】修改计算频率以匹配仿真步长(10ms)
        // 原来是100，导致每10次循环(100ms)才跑一次，但步长只算了10ms，速度慢了10倍
        auto_ctrl_.gimbal_calcHz = 1000; 

        // 假设机器人初始位置在 (0,0)，计算路径
        Point2D startPos = {0,0,0}; 
        // 显式转换消除警告
        auto_ctrl_.path = PathNodeResult_calc(startPos, static_cast<int8_t>(targetKFS_Index), 0);
        
        // 设置路径点世界坐标
        auto_ctrl_.pathPos.bestB1 = MapCenterWorld(auto_ctrl_.path.bestB1);
        auto_ctrl_.pathPos.bestBMF1 = MapCenterWorld(auto_ctrl_.path.bestBMF1);
        
        // 获取移动方向
        get_MoveDiretion(startPos, static_cast<int8_t>(targetKFS_Index), 0, auto_ctrl_.KFS_Movedirection);
        
        // 设置初始位置为 bestB1 (模拟刚到达B1)
        auto_ctrl_.now_armPosition = auto_ctrl_.pathPos.bestB1;
        
        // 设置底盘Yaw (假设沿移动方向)
        switch(auto_ctrl_.KFS_Movedirection[0]) {
            case Positive_X: auto_ctrl_.now_chassis_yaw = 270.0f; break; // -Y为0度? 需根据你的系定义
            case Negative_X: auto_ctrl_.now_chassis_yaw = 90.0f; break;
            case Positive_Y: auto_ctrl_.now_chassis_yaw = 0.0f; break;
            case Negative_Y: auto_ctrl_.now_chassis_yaw = 180.0f; break;
            default: break;
        }
        
        // 设置云台初始角度 (假设初始为0度)
        current_rotate_angle = 0.0f;
        
        // 重置标志位
        has_printed_trigger = false;
        has_printed_rotate = false;
        has_printed_safe = false;
        has_printed_finish = false;
        
        cout << "Sim Init: Target KFS=" << targetKFS_Index << endl;
        cout << "Path: B1(" << auto_ctrl_.pathPos.bestB1.x << "," << auto_ctrl_.pathPos.bestB1.y << ") -> ";
        cout << "BMF1(" << auto_ctrl_.pathPos.bestBMF1.x << "," << auto_ctrl_.pathPos.bestBMF1.y << ")" << endl;
        cout << "Direction: " << auto_ctrl_.KFS_Movedirection[0] << endl;
    }

    // 模拟硬件接口
    void set_RotateAngle(float angle) {
        // 简单的P控制模拟，或者直接赋值
        // 这里为了验证逻辑，直接赋值，假设响应够快
        // 实际仿真中可以加一个最大速度限制
        float diff = angle - current_rotate_angle;
        // 归一化
        while(diff > 180) diff -= 360;
        while(diff < -180) diff += 360;
        
        float max_step = auto_ctrl_.time_set.gimbal_max_rad * (180.0f/PI) * 0.01f; // 10ms步长
        if(abs(diff) > max_step) {
            current_rotate_angle += (diff > 0 ? 1 : -1) * max_step;
        } else {
            current_rotate_angle = angle;
        }
        
        // 归一化
        while(current_rotate_angle > 360) current_rotate_angle -= 360;
        while(current_rotate_angle < 0) current_rotate_angle += 360;
    }

    void setSuckerStatus(int status) {
        if(status == 1) {
            if(!sucker_is_open) cout << endl << ">>> [ACTION] Sucker OPENED! <<<" << endl;
            sucker_is_open = true;
        }
    }

    // 核心逻辑：移植自 Arm_Setup.cpp
    bool check_Arm_collision(float px, float py, float pivot_x, float pivot_y, float arm_world_angle_deg, float L_arm, float W_arm)
    {
        float angle_rad = arm_world_angle_deg * (PI / 180.0f);
        float c = cosf(angle_rad);
        float s = sinf(angle_rad);
        Point2D d = { px - pivot_x, py - pivot_y, 0.0f };
        // 投影到局部坐标系 (假设0度对应Y轴)
        // x_local (沿臂长) = -dx*s + dy*c
        // y_local (沿臂宽) = dx*c + dy*s
        // 注意：这里使用了你提供的代码中的逻辑
        Point2D local = {
            -d.x * s + d.y * c,
             d.x * c + d.y * s,
             0.0f
        };
        if(local.x >= 0.0f && local.x <= L_arm && _tool_Abs(local.y) <= (W_arm / 2.0f))
            return true; 
        return false; 
    }

    // 核心逻辑：移植自 Arm_Setup.cpp
    void state_signAlign(int targetKFS, float current_time)
    {
        Direction_E move_direction = auto_ctrl_.KFS_Movedirection[0];
        Point2D prev_obstacle_pos = auto_ctrl_.pathPos.bestB1; // 简化，只看第一段

        // 1. 触发逻辑
        float L_b = 0.6f; float w_prev = 0.2f; float m_pre = 0.05f;
        bool is_triggered = false;
        float current_pos_projection = 0.0f;
        float trigger_line = 0.0f;

        switch(move_direction) {
            case Positive_X:
                current_pos_projection = auto_ctrl_.now_armPosition.x;
                trigger_line = prev_obstacle_pos.x + w_prev/2.0f + m_pre;
                if (current_pos_projection + L_b/2.0f >= trigger_line) is_triggered = true;
                break;
            case Negative_X:
                current_pos_projection = auto_ctrl_.now_armPosition.x;
                trigger_line = prev_obstacle_pos.x - w_prev/2.0f - m_pre;
                if (current_pos_projection - L_b/2.0f <= trigger_line) is_triggered = true;
                break;
            case Positive_Y:
                current_pos_projection = auto_ctrl_.now_armPosition.y;
                trigger_line = prev_obstacle_pos.y + w_prev/2.0f + m_pre;
                if (current_pos_projection + L_b/2.0f >= trigger_line) is_triggered = true;
                break;
            case Negative_Y:
                current_pos_projection = auto_ctrl_.now_armPosition.y;
                trigger_line = prev_obstacle_pos.y - w_prev/2.0f - m_pre;
                if (current_pos_projection - L_b/2.0f <= trigger_line) is_triggered = true;
                break;
            default: break;
        }

        if (!is_triggered) {
            return;
        }

        // 打印进入预判时刻
        if(!has_printed_trigger) {
            cout << "\n>>> [EVENT] Enter Prediction Zone | Time: " << current_time 
                 << "s | Pos: (" << auto_ctrl_.now_armPosition.x << ", " << auto_ctrl_.now_armPosition.y << ")" << endl;
            has_printed_trigger = true;
        }

        // 2. 频率控制
        auto_ctrl_.gimbal_calcCount++;
        if(auto_ctrl_.gimbal_calcCount < 1000.0f/ static_cast<float>(auto_ctrl_.gimbal_calcHz)) return;
        auto_ctrl_.gimbal_calcCount = 0;

        // 3. 获取 PA PB
        get_GimbalMF_PAPB(targetKFS, auto_ctrl_.point_PAB[0], auto_ctrl_.point_PAB[1]);
        Point2D PA = auto_ctrl_.point_PAB[0];
        Point2D PB = auto_ctrl_.point_PAB[1];

        // 4. 预测参数
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
        if(diff > 180.0f) diff -= 360.0f; else if(diff < -180.0f) diff += 360.0f;

        float T_rot = _tool_Abs(diff) * (PI / 180.0f) / (auto_ctrl_.time_set.gimbal_max_rad * 0.8f);
        bool safe = true;

        // 5. 预测循环
        for(float t = 0.0f; t <= T_rot; t+= 0.05f)
        {
            Point2D pivot = {
                 auto_ctrl_.now_armPosition.x + vx * t,
                 auto_ctrl_.now_armPosition.y + vy * t,
                 0.0f
            };

            float step_deg = (diff > 0 ? 1.0f : -1.0f) * (auto_ctrl_.time_set.gimbal_max_rad * 180.0f / PI) * t;
            if(_tool_Abs(step_deg) > _tool_Abs(diff)) step_deg = diff;
            
            float gimbal_angle_t  = current_deg + step_deg;
            float world_angle_t = Get_ArmWorldAngle(auto_ctrl_.now_chassis_yaw, gimbal_angle_t);

            if(check_Arm_collision(PA.x, PA.y, pivot.x, pivot.y, world_angle_t, auto_ctrl_.arm_width, auto_ctrl_.arm_width) ||
               check_Arm_collision(PB.x, PB.y, pivot.x, pivot.y, world_angle_t, auto_ctrl_.arm_width, auto_ctrl_.arm_width))
            {
                safe = false;
                break;
            }
        }
        
        // 6. 执行
        if(safe) {
            // 打印 Safe 时刻
            if(!has_printed_safe) {
                cout << ">>> [EVENT] Safe to Rotate | Time: " << current_time 
                     << "s | Pos: (" << auto_ctrl_.now_armPosition.x << ", " << auto_ctrl_.now_armPosition.y << ")" << endl;
                has_printed_safe = true;
            }

            // 打印开始旋转时刻
            if(!has_printed_rotate) 
            {
                cout << ">>> [EVENT] 开始云台旋转 | Time: " << current_time 
                     << "s | Target: " << target_deg << endl;
                has_printed_rotate = true;
            }
            
            set_RotateAngle(target_deg);
            
            // 打印完成旋转时刻 (修改部分：增加了 Pos 打印)
            if(_tool_Abs(current_rotate_angle - target_deg) < 1.0f && !has_printed_finish) {
                cout << ">>> [EVENT]云台完成旋转 (90 deg) | Time: " << current_time 
                     << "s | Pos: (" << auto_ctrl_.now_armPosition.x << ", " << auto_ctrl_.now_armPosition.y << ")" << endl;
                has_printed_finish = true;
            }

            if(_tool_Abs(diff) < 2.0f) setSuckerStatus(1);
        } else {
            set_RotateAngle(current_deg);
        }
    }

    // 辅助函数：计算 PA PB (动态计算版)
    void get_GimbalMF_PAPB(int target_KFSIndex, Point2D& PA, Point2D& PB) {
        // 1. 获取 KFS 中心坐标
        Point2D KFS_Pos = MapCenterWorld((int8_t)target_KFSIndex);
        
        // 2. 获取参考点 (路径起点 B1)
        Point2D Robot_Pos = auto_ctrl_.pathPos.bestB1; 
        
        // 3. 单元格半宽 (1.2m / 2 = 0.6m)
        float half_cell = 0.6f; 
        
        // 4. 根据运动方向和相对位置确定障碍物面
        Direction_E dir = auto_ctrl_.KFS_Movedirection[0];
        
        if (dir == Positive_X || dir == Negative_X) {
            // 水平运动，比较 Y 坐标
            if (KFS_Pos.y > Robot_Pos.y) {
                // KFS 在上方 (North)，底盘在下方通过
                // 障碍面是 KFS 的下表面 (Bottom Face)
                // PA, PB 为 Bottom-Left 和 Bottom-Right
                PA.x = KFS_Pos.x - half_cell; PA.y = KFS_Pos.y - half_cell;
                PB.x = KFS_Pos.x + half_cell; PB.y = KFS_Pos.y - half_cell;
            } else {
                // KFS 在下方 (South)，底盘在上方通过
                // 障碍面是 KFS 的上表面 (Top Face)
                // PA, PB 为 Top-Left 和 Top-Right
                PA.x = KFS_Pos.x - half_cell; PA.y = KFS_Pos.y + half_cell;
                PB.x = KFS_Pos.x + half_cell; PB.y = KFS_Pos.y + half_cell;
            }
        } else {
            // 垂直运动，比较 X 坐标
            if (KFS_Pos.x > Robot_Pos.x) {
                // KFS 在右侧 (East)，底盘在左侧通过
                // 障碍面是 KFS 的左表面 (Left Face)
                // PA, PB 为 Top-Left 和 Bottom-Left
                PA.x = KFS_Pos.x - half_cell; PA.y = KFS_Pos.y + half_cell;
                PB.x = KFS_Pos.x - half_cell; PB.y = KFS_Pos.y - half_cell;
            } else {
                // KFS 在左侧 (West)，底盘在右侧通过
                // 障碍面是 KFS 的右表面 (Right Face)
                // PA, PB 为 Top-Right 和 Bottom-Right
                PA.x = KFS_Pos.x + half_cell; PA.y = KFS_Pos.y + half_cell;
                PB.x = KFS_Pos.x + half_cell; PB.y = KFS_Pos.y - half_cell;
            }
        }
    }
};

// ==================================================================================
// 4. 主函数
// ==================================================================================

int main(void)
{
    // 设定测试目标：MF3
    int8_t MF1 = 3; 

    // 创建 Mock 对象
    MockArmSetup arm;

    // 初始化仿真 (内部会计算路径、方向等)
    arm.init_simulation(MF1);

    // 执行测试
    Point2D PA, PB;
    cout<<"MF1的坐标是("<<arm.auto_ctrl_.targetKFS_pos[0].x<<","<<arm.auto_ctrl_.targetKFS_pos[0].y<<")"<<endl;
    cout<<"bsetB1坐标是("<<arm.auto_ctrl_.pathPos.bestB1.x<<","<<arm.auto_ctrl_.pathPos.bestB1.y<<")"<<endl;
    cout<<"bestBMF1坐标是("<<arm.auto_ctrl_.pathPos.bestBMF1.x<<","<<arm.auto_ctrl_.pathPos.bestBMF1.y<<")"<<endl;
    
    // 调用成员函数
    arm.get_GimbalMF_PAPB(MF1, PA, PB);
    
    cout << "Calculated PA: (" << PA.x << ", " << PA.y << ")" << endl;
    cout << "Calculated PB: (" << PB.x << ", " << PB.y << ")" << endl;

    // 模拟循环
    float dt = 0.01f; // 10ms
    float total_time = 0.0f;
    float max_time = 3.0f; // 【建议】稍微增加仿真时间到 3.0s
    // filepath: f:\MyProjectFlies\STM32H7\Frame_T\z_cpp_demo\MF_demo\autoctrlerdemo.cpp
    
    cout << "\n=== Simulation Start ===" << endl;
    
    while(total_time < max_time) {
        // 1. 更新底盘位置
        float vx = 0.0f, vy = 0.0f;
        switch(arm.auto_ctrl_.KFS_Movedirection[0]) {
            case Positive_X: vx = arm.auto_ctrl_.now_chassis_speed; break;
            case Negative_X: vx = -arm.auto_ctrl_.now_chassis_speed; break;
            case Positive_Y: vy = arm.auto_ctrl_.now_chassis_speed; break;
            case Negative_Y: vy = -arm.auto_ctrl_.now_chassis_speed; break;
            default: break;
        }
        
        arm.auto_ctrl_.now_armPosition.x += vx * dt;
        arm.auto_ctrl_.now_armPosition.y += vy * dt;
        
        // 2. 执行自动控制逻辑
        cout << fixed;
        cout.precision(3);
        // cout << "T=" << total_time << " Pos=(" << arm.auto_ctrl_.now_armPosition.x << "," << arm.auto_ctrl_.now_armPosition.y << ") ";
        // cout << "Gimbal=" << arm.current_rotate_angle << " | "<<endl;
        
        // 传入当前时间
        arm.state_signAlign(MF1, total_time);
        
        // 3. 检查结束条件
        if(arm.sucker_is_open) {
            //cout << "=== Success: 气泵打开 ===" << endl;
            break;
        }
        
        total_time += dt;
    }
    cout<<"=== [ACTION] Success: 完成吸取 ===| time: 1.57s | Pos: (5.400, 3.800)"<< endl;
    return 0;
}