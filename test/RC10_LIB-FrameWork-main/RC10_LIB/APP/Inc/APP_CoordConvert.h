/**
 * @file APP_CoordConvert.cpp
 * @author Zhan Hongli (Original), XieFField (Refactored)
 * @brief 使用 CMSIS-DSP (arm_math.h) 优化的齐次坐标变换库。
 * @version 2.0
 * @brief
 * 该库提供了2D和3D的齐次变换功能，例如将激光雷达的坐标系转换到机器人底盘的中心坐标系。
 * 所有矩阵运算均使用ARM官方的CMSIS-DSP库进行加速，以获得最佳性能。
 * 
 * @note 所有角度参数的单位均为 **弧度 (radians)**。
 */
#ifndef __APP_COORDCONVERT_H
#define __APP_COORDCONVERT_H

#include "arm_math.h"
#include "APP_tool.h" // 包含 Point2D 和 Point3D 结构体定义



/**
 * @brief 使用 CMSIS-DSP 的2D齐次变换类。
 * @details
 * 用于处理二维平面上的平移和旋转。
 * 例如：将一个安装在机器人上、有一定偏移和旋转角度的传感器的坐标，转换到机器人的中心坐标系。
 */
class HomogeneousTransform2D {
public:
    /**
     * @brief 默认构造函数，创建一个单位变换（无平移、无旋转）。
     */
    HomogeneousTransform2D();

    /**
     * @brief 构造函数，根据给定的位姿创建一个变换。
     * @param pose 包含平移和旋转信息的2D点。
     */
    HomogeneousTransform2D(const Point2D& pose);

    /**
     * @brief 设置一个全新的变换，覆盖当前值。
     * @param pose 包含平移和旋转信息的2D点。
     */
    void setTransform(const Point2D& pose);

    /**
     * @brief 仅设置变换的平移部分，旋转部分保持不变。
     * @param translation 包含x, y平移量的2D点（将忽略其theta成员）。
     */
    void setTranslation(const Point2D& translation);

    /**
     * @brief 仅设置变换的旋转部分，平移部分保持不变。
     * @param theta_rad 绕Z轴的旋转角度（单位：弧度）。
     */
    void setRotation(float theta_rad);

    /**
     * @brief 将此变换应用于一个2D点。
     * @param point 要变换的原始点。
     * @return Point2D 变换后的新点。
     */
    Point2D apply(const Point2D& point) const;

    /**
     * @brief 矩阵乘法：将当前变换与另一个变换相乘。
     * @details
     * 这相当于将两个变换叠加。例如 T_A_to_C = T_A_to_B.multiply(T_B_to_C)。
     * @param other 要乘在右边的变换矩阵。
     * @return HomogeneousTransform2D 两个变换相乘后的结果。
     */
    HomogeneousTransform2D multiply(const HomogeneousTransform2D& other) const;

    /**
     * @brief 计算当前变换的逆变换。
     * @details
     * 如果一个变换是从A坐标系到B坐标系 (T_A_to_B)，那么它的逆变换就是从B坐标系到A坐标系 (T_B_to_A)。
     * @return HomogeneousTransform2D 逆变换矩阵。
     */
    HomogeneousTransform2D inverse() const;

private:
    float32_t m_data[9]; // 3x3 矩阵数据 (行主序)
    arm_matrix_instance_f32 m_matrix; // CMSIS-DSP 矩阵实例
};

/**
 * @brief 使用 CMSIS-DSP 的3D齐次变换类。
 * @details
 * 用于处理三维空间中的平移和旋转。
 */
class HomogeneousTransform3D {
public:
    /**
     * @brief 默认构造函数，创建一个单位变换。
     */
    HomogeneousTransform3D();

    /**
     * @brief 构造函数，根据给定的位姿创建一个变换。
     * @param pose 包含平移和旋转信息的3D点。
     */
    HomogeneousTransform3D(const Point3D& pose);

    /**
     * @brief 设置一个全新的变换，覆盖当前值。
     * @param pose 包含平移和旋转信息的3D点。
     */
    void setTransform(const Point3D& pose);

    /**
     * @brief 仅设置变换的平移部分，旋转部分保持不变。
     * @param translation 包含x, y, z平移量的3D点（将忽略其旋转成员）。
     */
    void setTranslation(const Point3D& translation);

    /**
     * @brief 仅设置变换的旋转部分（基于欧拉角），平移部分保持不变。
     * @param roll_rad  绕X轴的旋转（横滚角，单位：弧度）。
     * @param pitch_rad 绕Y轴的旋转（俯仰角，单位：弧度）。
     * @param yaw_rad   绕Z轴的旋转（偏航角，单位：弧度）。
     */
    void setRotation(float roll_rad, float pitch_rad, float yaw_rad);

    /**
     * @brief 将此变换应用于一个3D点。
     * @param point 要变换的原始点。
     * @return Point3D 变换后的新点。
     */
    Point3D apply(const Point3D& point) const;

    /**
     * @brief 矩阵乘法：将当前变换与另一个变换相乘。
     */
    HomogeneousTransform3D multiply(const HomogeneousTransform3D& other) const;

    /**
     * @brief 计算当前变换的逆变换。
     */
    HomogeneousTransform3D inverse() const;

private:
    float32_t m_data[16]; // 4x4 矩阵数据 (行主序)
    arm_matrix_instance_f32 m_matrix; // CMSIS-DSP 矩阵实例
};



#endif // __APP_COORDCONVERT_H
