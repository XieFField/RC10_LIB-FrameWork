import numpy as np
import matplotlib.pyplot as plt

# 定义初始变量

x_0 = 0.1
x_g = 5.5
v_max = 3.0
v_0 = 0.5
v_g = 0
a_max_accel = 60.0     # 加速段最大加速度
a_max_decel = 1.0    # 减速段最大加速度（现在可以分开设置）
j_max = 20.0     # 最大加加速度（jerk）
count = 0      # 计数器，用于记录调整次数

print("加速度参数设置:")
print(f"加速段最大加速度: {a_max_accel}")
print(f"减速段最大加速度: {a_max_decel}")

# ========== 计算加速段参数 ==========
# 判断是否能达到最大加速度
if (v_max - v_0) * j_max < a_max_accel ** 2:   # 不能达到最大加速度的情况
    if v_0 > v_max:  # 如果初始速度已经大于最大速度
        Tj1 = 0      # 加加速时间为0
        Ta = 0       # 总加速时间为0
        alima = 0    # 实际最大加速度为0
    else:            # 正常情况下的计算
        Tj1 = np.sqrt((v_max - v_0) / j_max)  # 加加速时间
        Ta = 2 * Tj1                          # 总加速时间（对称的加加速和减加速）
        alima = Tj1 * j_max                   # 实际能达到的最大加速度
else:                               # 能达到最大加速度的情况
    Tj1 = a_max_accel / j_max             # 加加速时间
    Ta = Tj1 + (v_max - v_0) / a_max_accel  # 总加速时间
    alima = a_max_accel                   # 实际最大加速度等于设定最大加速度

# ========== 计算减速段参数 ==========
# 判断是否能达到最大减速度
if (v_max - v_g) * j_max < a_max_decel ** 2:   # 不能达到最大减速度的情况
    Tj2 = np.sqrt((v_max - v_g) / j_max)  # 加减速时间
    Td = 2 * Tj2                         # 总减速时间
    alimd = Tj2 * j_max                  # 实际最大减速度
else:                              # 能达到最大减速度的情况
    Tj2 = a_max_decel / j_max            # 加减速时间
    Td = Tj2 + (v_max - v_g) / a_max_decel  # 总减速时间
    alimd = a_max_decel                  # 实际最大减速度等于设定最大减速度

# ========== 计算匀速段时间 ==========
Tv = (x_g - x_0) / v_max - Ta / 2 * (1 + v_0 / v_max) - Td / 2 * (1 + v_g / v_max)

# ========== 处理不存在匀速阶段的情况 ==========
if Tv > 0:        # 存在匀速阶段
    vlim = v_max   # 限制速度等于最大速度
    T = Tv + Ta + Td  # 总时间
else:             # 不存在匀速阶段
    Tv = 0        # 匀速时间为0
    amax_accel_org = a_max_accel  # 保存原始加速段最大加速度值
    amax_decel_org = a_max_decel  # 保存原始减速段最大加速度值
    
    # 计算delta值，用于求解时间参数
    # 由于现在有两个不同的加速度，需要分别计算加速段和减速段
    # 这里使用平均加速度来近似计算
    a_avg = (a_max_accel + a_max_decel) / 2
    delta = (a_avg ** 4) / (j_max ** 2) + 2 * (v_0 ** 2 + v_g ** 2) + a_avg * (4 * (x_g - x_0) - 2 * a_avg / j_max * (v_0 + v_g))
    
    # 初始时间参数计算（使用平均加速度）
    Tj1 = a_max_accel / j_max
    Ta = (a_avg ** 2 / j_max - 2 * v_0 + np.sqrt(delta)) / (2 * a_avg)
    Tj2 = a_max_decel / j_max
    Td = (a_avg ** 2 / j_max - 2 * v_g + np.sqrt(delta)) / (2 * a_avg)
    vlim = v_0 + (Ta - Tj1) * alima  # 计算实际达到的最大速度

    # 逐渐减少加速度，直到找到可行的解
    while Ta < 2 * Tj1 or Td < 2 * Tj2:
        count += 1
        # 同时减少加速段和减速段的加速度，保持比例关系
        reduction_factor = 0.9  # 每次减少10%
        a_max_accel = max(a_max_accel * reduction_factor, 0.1)  # 保持最小加速度
        a_max_decel = max(a_max_decel * reduction_factor, 0.1)  # 保持最小加速度
        
        # 重新计算加速段参数
        if (v_max - v_0) * j_max < a_max_accel ** 2:
            Tj1 = np.sqrt((v_max - v_0) / j_max)
            Ta = 2 * Tj1
            alima = Tj1 * j_max
        else:
            Tj1 = a_max_accel / j_max
            Ta = Tj1 + (v_max - v_0) / a_max_accel
            alima = a_max_accel
        
        # 重新计算减速段参数
        if (v_max - v_g) * j_max < a_max_decel ** 2:
            Tj2 = np.sqrt((v_max - v_g) / j_max)
            Td = 2 * Tj2
            alimd = Tj2 * j_max
        else:
            Tj2 = a_max_decel / j_max
            Td = Tj2 + (v_max - v_g) / a_max_decel
            alimd = a_max_decel
        
        # 重新计算平均加速度和delta值
        a_avg = (a_max_accel + a_max_decel) / 2
        if a_avg > 0:
            delta = (a_avg ** 4) / (j_max ** 2) + 2 * (v_0 ** 2 + v_g ** 2) + a_avg * (4 * (x_g - x_0) - 2 * a_avg / j_max * (v_0 + v_g))
        else:
            delta = (a_avg ** 4) / (j_max ** 2) + 2 * (v_0 ** 2 + v_g ** 2) - a_avg * (4 * (x_g - x_0) - 2 * a_avg / j_max * (v_0 + v_g))
        
        # 重新计算时间参数
        Ta = (a_avg ** 2 / j_max - 2 * v_0 + np.sqrt(delta)) / (2 * a_avg)
        Td = (a_avg ** 2 / j_max - 2 * v_g + np.sqrt(delta)) / (2 * a_avg)
        vlim = v_0 + (Ta - Tj1) * alima  # 重新计算实际最大速度
        
        # 防止无限循环
        if count > 100:
            print("警告：经过100次迭代仍未找到可行解")
            break

    print("调整后信息:")
    print("TJ1:", Tj1)
    print("Ta:", Ta)
    print("TJ2:", Tj2)
    print("Td:", Td)
    print("调整后的加速段最大加速度:", a_max_accel)
    print("调整后的减速段最大加速度:", a_max_decel)
    print("调整次数:", count)

    # 处理加速或减速时间为负的情况
    if Ta < 0 or Td < 0:
        if v_0 > v_g:  # 初始速度大于目标速度，主要是减速
            Ta = 0
            Tj1 = 0
            alima = 0
            Td = 2 * (x_g - x_0) / (v_g + v_0)
            Tj2 = (j_max * (x_g - x_0) - np.sqrt(j_max * (j_max * (x_g - x_0) ** 2 + (v_g + v_0) ** 2 * (v_g - v_0)))) / (j_max * (v_g + v_0))
            alimd = -j_max * Tj2
            vlim = v_g - (Td - Tj2) * alimd
            alimd = -alimd
        else:  # 主要是加速
            Td = 0
            Tj2 = 0
            Ta = 2 * (x_g - x_0) / (v_g + v_0)
            Tj1 = (j_max * (x_g - x_0) - np.sqrt(j_max * (j_max * (x_g - x_0) ** 2 - (v_g + v_0) ** 2 * (v_g - v_0)))) / (j_max * (v_g + v_0))
            alima = j_max * Tj1
            vlim = v_0 + (Ta - Tj1) * alima

    print("最终时间参数:")
    print("Tj1:", Tj1)
    print("Tj2:", Tj2)
    print("Ta:", Ta)
    print("Td:", Td)
    print("加速度参数:")
    print("alima:", alima)
    print("alimd:", alimd)
    
    T = Tv + Ta + Td  # 计算总时间

# ========== 计算各阶段路程 ==========
# 阶段1: 加加速阶段 (0 <= t < Tj1)
distance_stage1 = v_0 * Tj1 + j_max * Tj1 ** 3 / 6

# 阶段2: 匀加速阶段 (Tj1 <= t < (Ta - Tj1))
T2_duration = Ta - 2 * Tj1
v_start_stage2 = v_0 + j_max * Tj1 ** 2 / 2
distance_stage2 = v_start_stage2 * T2_duration + 0.5 * alima * T2_duration ** 2

# 阶段3: 减加速阶段 ((Ta - Tj1) <= t < Ta)
distance_stage3 = vlim * Tj1 - j_max * Tj1 ** 3 / 6

# 阶段4: 匀速阶段 (Ta <= t < (Ta + Tv))
distance_stage4 = vlim * Tv

# 阶段5: 加减速阶段 ((T - Td) <= t < (T - Td + Tj2))
distance_stage5 = vlim * Tj2 - j_max * Tj2 ** 3 / 6

# 阶段6: 匀减速阶段 ((T - Td + Tj2) <= t < (T - Tj2))
T6_duration = Td - 2 * Tj2
v_start_stage6 = vlim - j_max * Tj2 ** 2 / 2
distance_stage6 = v_start_stage6 * T6_duration - 0.5 * alimd * T6_duration ** 2

# 阶段7: 减减速阶段 ((T - Tj2) <= t < T)
distance_stage7 = v_g * Tj2 + j_max * Tj2 ** 3 / 6

# 计算总路程用于验证
total_distance_calculated = (distance_stage1 + distance_stage2 + distance_stage3 + 
                           distance_stage4 + distance_stage5 + distance_stage6 + distance_stage7)

# ========== 生成轨迹数据 ==========
p = []   # 位置序列
vc = []  # 速度序列
ac = []  # 加速度序列
jc = []  # 加加速度序列

# 遍历时间序列，计算每个时间点的运动状态
for t in np.arange(0, T, 0.001):
    # 阶段1: 加加速阶段 (0 <= t < Tj1)
    if 0 <= t < Tj1:
        x = x_0 + v_0 * t + j_max * t ** 3 / 6
        p.append(x)
        v = v_0 + j_max * t ** 2 / 2
        vc.append(v)
        a = j_max * t
        ac.append(a)
        jc.append(j_max)
        
    # 阶段2: 匀加速阶段 (Tj1 <= t < (Ta - Tj1))
    elif Tj1 <= t < (Ta - Tj1):
        x = x_0 + v_0 * t + alima / 6 * (3 * t ** 2 - 3 * Tj1 * t + Tj1 ** 2)
        p.append(x)
        v = v_0 + alima * (t - Tj1 / 2)
        vc.append(v)
        a = alima
        ac.append(a)
        jc.append(0)
        
    # 阶段3: 减加速阶段 ((Ta - Tj1) <= t < Ta)
    elif (Ta - Tj1) <= t < Ta:
        x = x_0 + (vlim + v_0) * Ta / 2 - vlim * (Ta - t) + j_max * (Ta - t) ** 3 / 6
        p.append(x)
        v = vlim - j_max * (Ta - t) ** 2 / 2
        vc.append(v)
        a = j_max * (Ta - t)
        ac.append(a)
        jc.append(-j_max)
        
    # 阶段4: 匀速阶段 (Ta <= t < (Ta + Tv))
    elif Ta <= t < (Ta + Tv):
        x = x_0 + (vlim + v_0) * Ta / 2 + vlim * (t - Ta)
        p.append(x)
        v = vlim
        vc.append(v)
        a = 0
        ac.append(0)
        jc.append(0)
        
    # 阶段5: 加减速阶段 ((T - Td) <= t < (T - Td + Tj2))
    elif (T - Td) <= t < (T - Td + Tj2):
        x = x_g - (vlim + v_g) * Td / 2 + vlim * (t - T + Td) - j_max * (t - T + Td) ** 3 / 6
        p.append(x)
        v = vlim - j_max * (t - T + Td) ** 2 / 2
        vc.append(v)
        a = -j_max * (t - T + Td)
        ac.append(a)
        jc.append(-j_max)
        
    # 阶段6: 匀减速阶段 ((T - Td + Tj2) <= t < (T - Tj2))
    elif (T - Td + Tj2) <= t < (T - Tj2):
        x = x_g - (vlim + v_g) * Td / 2 + vlim * (t - T + Td) - alimd / 6 * (3 * (t - T + Td) ** 2 - 3 * Tj2 * (t - T + Td) + Tj2 ** 2)
        p.append(x)
        v = vlim - alimd * (t - T + Td - Tj2 / 2)
        vc.append(v)
        a = -alimd
        ac.append(a)
        jc.append(0)
        
    # 阶段7: 减减速阶段 ((T - Tj2) <= t < T)
    elif (T - Tj2) <= t < T:
        x = x_g - v_g * (T - t) - j_max * (T - t) ** 3 / 6
        p.append(x)
        v = v_g + j_max * (T - t) ** 2 / 2
        vc.append(v)
        a = -j_max * (T - t)
        ac.append(a)
        jc.append(j_max)

# 创建时间序列用于绘图
t = np.arange(0, T, 0.001)

# ========== 打印各阶段路程信息 ==========
print("\n" + "="*50)
print("各阶段路程分析")
print("="*50)
print(f"阶段1 (加加速): {distance_stage1:.4f} 单位")
print(f"阶段2 (匀加速): {distance_stage2:.4f} 单位") 
print(f"阶段3 (减加速): {distance_stage3:.4f} 单位")
print(f"阶段4 (匀速)  : {distance_stage4:.4f} 单位")
print(f"阶段5 (加减速): {distance_stage5:.4f} 单位")
print(f"阶段6 (匀减速): {distance_stage6:.4f} 单位")
print(f"阶段7 (减减速): {distance_stage7:.4f} 单位")
print("-"*50)
print(f"各阶段路程总和: {total_distance_calculated:.4f} 单位")
print(f"目标总路程    : {x_g - x_0:.4f} 单位")
print(f"路程误差      : {abs(total_distance_calculated - (x_g - x_0)):.6f} 单位")

# 验证：从生成的位置数据计算总路程
if len(p) > 0:
    actual_total_distance = p[-1] - p[0]
    print(f"实际生成路程  : {actual_total_distance:.4f} 单位")
    print(f"实际路程误差  : {abs(actual_total_distance - (x_g - x_0)):.6f} 单位")

# 读取并打印位置序列的最后一个值
if len(p) > 0:
    last_position = p[-1]
    print(f"\n位置序列最后一个值: {last_position:.6f}")
else:
    print("\n位置序列为空")

# ========== 绘制运动曲线 ==========
plt.figure(figsize=(12, 8))

# 位置曲线
plt.subplot(4, 1, 1)
plt.plot(t, p)
plt.ylabel('Position')
plt.legend(['Position'])

# 速度曲线
plt.subplot(4, 1, 2)
plt.plot(t, vc)
plt.ylabel('Velocity')
plt.legend(['Velocity'])

# 加速度曲线
plt.subplot(4, 1, 3)
plt.plot(t, ac)
plt.ylabel('Acceleration')
plt.xlabel('Time')
plt.legend(['Acceleration'])

# 加加速度曲线
plt.subplot(4, 1, 4)
plt.plot(t, jc)
plt.ylabel('Jerk')
plt.xlabel('Time')
plt.legend(['Jerk'])

# 显示图形
plt.tight_layout()
plt.show()



""" int count = 0;                        // 调整次数计数器
    Tv = 0;                               // 匀速时间为0
    float amax_accel_org, amax_decel_org; // 保存原始加速和减速最大加速度
    amax_accel_org = m_maxAcc_;           // 保存原始加速段最大加速度值
    amax_decel_org = m_maxDec_;           // 保存原始减速段最大加速度值

    // 需要分别调整加速段和减速段的最大加速度
    // 这里简化处理，按比例同时调整两个加速度
    float scale_factor = 0.9f; // 调整比例

    // 逐渐减少加速度，直到找到可行的解
    while (true)
    {
        // 重新计算加速段参数
        if ((m_maxSpeed_ - m_initialSpeed_) * m_maxJerk_ < m_maxAcc_ * m_maxAcc_)
        {
            arm_sqrt_f32((m_maxSpeed_ - m_initialSpeed_) / m_maxJerk_, &Tj1);
            Ta = 2 * Tj1;
            alima = Tj1 * m_maxJerk_;
        }

        else
        {
            Tj1 = m_maxAcc_ / m_maxJerk_;
            Ta = Tj1 + (m_maxSpeed_ - m_initialSpeed_) / m_maxAcc_;
            alima = m_maxAcc_;
        }

        // 重新计算减速段参数
        if ((m_maxSpeed_ - m_finalSpeed_) * m_maxJerk_ < m_maxDec_ * m_maxDec_)
        {
            arm_sqrt_f32((m_maxSpeed_ - m_finalSpeed_) / m_maxJerk_, &Tj2);
            Td = 2 * Tj2;
            alimd = Tj2 * m_maxJerk_;
        }
        else
        {
            Tj2 = m_maxDec_ / m_maxJerk_;
            Td = Tj2 + (m_maxSpeed_ - m_finalSpeed_) / m_maxDec_;
            alimd = m_maxDec_;
        }
        vlim = m_finalSpeed_ + (Ta - Tj1) * alima;
        // 计算总路程
        float total_distance = (m_initialSpeed_ + vlim) * Ta / 2 + vlim * Tv + (vlim + m_finalSpeed_) * Td / 2;

        // 检查是否满足距离约束且时间参数有效
        if (abs(total_distance - (m_targetPos_ - m_startPos_)) < 0.01 && Ta >= 2 * Tj1 && Td >= 2 * Tj2 && Ta >= 0 && Td >= 0)
        {
            break;
        }

        // 调整加速度
        m_maxAcc_ *= scale_factor;
        m_maxDec_ *= scale_factor;
        count += 1;

        if (count > 100) // 防止无限循环
        {
            m_maxAcc_ = 0.0f;        // 最大加速度
            m_maxDec_ = 0.0f;        // 最大减速度
            m_maxJerk_ = 0.0f;       // 最大加加速度
            m_maxSpeed_ = 0.0f;      // 最大速度
            m_initialSpeed_ = 0.0f;  // 起始速度
            m_finalSpeed_ = 0.0f;    // 目标速度
            m_startPos_ = 0.0f;      // 起始位置
            m_targetPos_ = 0.0f;     // 目标位置
            m_totalDistance_ = 0.0f; // 总路程
            m_deadzone_ = 0.0f;      // 死区范围
            err_ = 1;
            break;
        }
    }
    vlim = m_initialSpeed_ + (Ta - Tj1) * alima; // 计算实际达到的最大速度
    T = Tv + Ta + Td;
     // 计算总时间
} """
 