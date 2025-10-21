// kinematics.h
#pragma once
#include <cmath>

struct Pose 
{
    float x, y, z;
};

struct JointState 
{
    float h;   // lift (m)
    float theta; // yaw (rad)
    float s;   // extension (m)
};

class Kinematics 
{
public:
    Kinematics(float base_x, float base_y, float base_z, float L0,
               float s_min, float s_max, float h_min, float h_max)
        : x0(base_x), y0(base_y), z0(base_z), L0(L0),
          s_min(s_min), s_max(s_max), h_min(h_min), h_max(h_max) {}

    // 正运动学：给关节状态，得到末端位姿
    Pose forward(const JointState &q) const 
    {
        float L = L0 + q.s;
        Pose p;
        p.x = x0 + L * std::cos(q.theta);
        p.y = y0 + L * std::sin(q.theta);
        p.z = z0 + q.h;
        return p;
    }

    // 逆运动学：给末端位姿，得到关节（尽量可达，返回 true 表示成功）
    // 如果超出范围，会被截断（saturate）。返回值表示是否原始值在允许范围内。
    bool inverse(const Pose &p, JointState &q_out) const 
    {
        float dx = p.x - x0;
        float dy = p.y - y0;
        float r = std::sqrt(dx*dx + dy*dy);

        float s = r - L0;              // 理想伸展
        float theta = std::atan2(dy, dx); // 方向角
        float h = p.z - z0;

        bool ok = true;
        if (s < s_min) { s = s_min; ok = false; }
        if (s > s_max) { s = s_max; ok = false; }
        if (h < h_min) { h = h_min; ok = false; }
        if (h > h_max) { h = h_max; ok = false; }

        q_out.s = s;
        q_out.theta = normalizeAngle(theta);
        q_out.h = h;
        return ok;
    }

    // 计算雅可比 J(q) (3x3) 存入 user 提供的二维数组 J[3][3]
    // J maps qdot = [h?, θ?, s?] -> v = [x?, y?, z?]
    void jacobian(const JointState &q, float J[3][3]) const 
    {
        float L = L0 + q.s;
        float c = std::cos(q.theta);
        float s = std::sin(q.theta);

        // ?x/?h, ?x/?θ, ?x/?s
        J[0][0] = 0.0f;
        J[0][1] = -L * s;
        J[0][2] = c;

        // ?y/?h, ?y/?θ, ?y/?s
        J[1][0] = 0.0f;
        J[1][1] =  L * c;
        J[1][2] = s;

        // ?z/?h, ?z/?θ, ?z/?s
        J[2][0] = 1.0f;
        J[2][1] = 0.0f;
        J[2][2] = 0.0f;
    }

    // 角度规范化到 [-pi, pi)
    static float normalizeAngle(float a) 
    {
        const float PI = 3.14159265358979323846f;
        while (a >= PI) a -= 2.0f*PI;
        while (a < -PI) a += 2.0f*PI;
        return a;
    }

private:
    float x0, y0, z0;   // 基准原点位置
    float L0;           // 初始臂长（不伸展）
    float s_min, s_max; // 伸缩限位
    float h_min, h_max; // 抬升限位
};
