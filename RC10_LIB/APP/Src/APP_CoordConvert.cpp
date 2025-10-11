
#include "APP_CoordConvert.h"



/* ==================================================================================
 *                            2D齐次变换 - 实现
 * ==================================================================================*/

// 构造函数：初始化矩阵实例并设为单位矩阵
HomogeneousTransform2D::HomogeneousTransform2D() : m_matrix{3, 3, m_data} 
{
    arm_set_identity_f32(&m_matrix); //此函数在APP_tool里面
}

// 构造函数：初始化矩阵实例并设置初始变换
HomogeneousTransform2D::HomogeneousTransform2D(const Point2D& pose) : m_matrix{3, 3, m_data} 
{
    setTransform(pose);
}


// 设置变换矩阵
void HomogeneousTransform2D::setTransform(const Point2D& pose) 
{
    float32_t c, s;
    arm_sin_cos_f32(pose.theta, &s, &c); 
    
    // 按照行主序填充矩阵数据
    m_data[0] = c;  m_data[1] = -s; m_data[2] = pose.x;
    m_data[3] = s;  m_data[4] = c;  m_data[5] = pose.y;
    m_data[6] = 0;  m_data[7] = 0;  m_data[8] = 1;
}

// 仅设置平移部分
void HomogeneousTransform2D::setTranslation(const Point2D& translation) 
{
    m_data[2] = translation.x;
    m_data[5] = translation.y;
}

// 仅设置旋转部分
void HomogeneousTransform2D::setRotation(float theta_rad) 
{
    float32_t tx = m_data[2];
    float32_t ty = m_data[5];
    float32_t c, s;
    arm_sin_cos_f32(theta_rad, &s, &c);
    m_data[0] = c; m_data[1] = -s;
    m_data[3] = s; m_data[4] = c;
    // 保持平移部分不变
    m_data[2] = tx;
    m_data[5] = ty;
}

// 应用变换到2D点
Point2D HomogeneousTransform2D::apply(const Point2D& point) const 
{
    // 将2D点扩展为齐次坐标 [x, y, 1]'
    float32_t p_in[3] = {point.x, point.y, 1.0f};
    float32_t p_out[3];
    arm_matrix_instance_f32 p_in_mat = {3, 1, p_in};
    arm_matrix_instance_f32 p_out_mat = {3, 1, p_out};

    // 执行矩阵乘法: p_out = M * p_in
    arm_mat_mult_f32(&m_matrix, &p_in_mat, &p_out_mat);
    
    return Point2D(p_out[0], p_out[1]);
}

// 矩阵乘法
HomogeneousTransform2D HomogeneousTransform2D::multiply(const HomogeneousTransform2D& other) const 
{
    HomogeneousTransform2D result;
    // 执行矩阵乘法: result = this * other
    arm_mat_mult_f32(&m_matrix, &other.m_matrix, &result.m_matrix);
    return result;
}

// 计算逆变换（针对刚体变换的优化方法）
HomogeneousTransform2D HomogeneousTransform2D::inverse() const 
{
    HomogeneousTransform2D result;
    // 刚体变换的逆，旋转部分等于原旋转矩阵的转置
    // R' = R^T
    result.m_data[0] = m_data[0]; result.m_data[1] = m_data[3];
    result.m_data[3] = m_data[1]; result.m_data[4] = m_data[4];

    // 平移部分等于 -R^T * t
    float32_t tx = m_data[2];
    float32_t ty = m_data[5];
    result.m_data[2] = -(result.m_data[0] * tx + result.m_data[1] * ty);
    result.m_data[5] = -(result.m_data[3] * tx + result.m_data[4] * ty);
    
    // 填充齐次坐标部分
    result.m_data[6] = 0; result.m_data[7] = 0; result.m_data[8] = 1;
    return result;
}


/* ==================================================================================
 *                            3D齐次变换 - 实现
 * ==================================================================================*/

// 构造函数：初始化矩阵实例并设为单位矩阵
HomogeneousTransform3D::HomogeneousTransform3D() : m_matrix{4, 4, m_data} 
{
    arm_set_identity_f32(&m_matrix); //此函数在APP_tool里面 
}

HomogeneousTransform3D::HomogeneousTransform3D(const Point3D& pose) : m_matrix{4, 4, m_data} 
{
    setTransform(pose);
}



void HomogeneousTransform3D::setTransform(const Point3D& pose) 
{
    setRotation(pose.roll, pose.pitch, pose.yaw);
    setTranslation(pose);
}

void HomogeneousTransform3D::setTranslation(const Point3D& translation) 
{
    m_data[3] = translation.x;
    m_data[7] = translation.y;
    m_data[11] = translation.z;
}

// 根据欧拉角设置旋转矩阵
void HomogeneousTransform3D::setRotation(float roll_rad, float pitch_rad, float yaw_rad) 
{
    float32_t sr, cr, sp, cp, sy, cy;
    arm_sin_cos_f32(roll_rad, &sr, &cr);
    arm_sin_cos_f32(pitch_rad, &sp, &cp);
    arm_sin_cos_f32(yaw_rad, &sy, &cy);

    float32_t tx = m_data[3];
    float32_t ty = m_data[7];
    float32_t tz = m_data[11];

    // 按照 Z-Y-X 欧拉角顺序构建旋转矩阵
    m_data[0] = cy * cp;
    m_data[1] = cy * sp * sr - sy * cr;
    m_data[2] = cy * sp * cr + sy * sr;

    m_data[4] = sy * cp;
    m_data[5] = sy * sp * sr + cy * cr;
    m_data[6] = sy * sp * cr - cy * sr;

    m_data[3] = tx;
    m_data[7] = ty;
    m_data[11] = tz;

    m_data[8]  = -sp;
    m_data[9]  = cp * sr;
    m_data[10] = cp * cr;
    
    // 确保齐次坐标部分正确
    m_data[12] = 0; m_data[13] = 0; m_data[14] = 0; m_data[15] = 1;
}

// 应用变换到3D点
Point3D HomogeneousTransform3D::apply(const Point3D& point) const 
{
    // 将3D点扩展为齐次坐标 [x, y, z, 1]'
    float32_t p_in[4] = {point.x, point.y, point.z, 1.0f};
    float32_t p_out[4];
    arm_matrix_instance_f32 p_in_mat = {4, 1, p_in};
    arm_matrix_instance_f32 p_out_mat = {4, 1, p_out};

    // 执行矩阵乘法: p_out = M * p_in
    arm_mat_mult_f32(&m_matrix, &p_in_mat, &p_out_mat);
    
    return Point3D(p_out[0], p_out[1], p_out[2]);
}

// 矩阵乘法
HomogeneousTransform3D HomogeneousTransform3D::multiply(const HomogeneousTransform3D& other) const 
{
    HomogeneousTransform3D result;
    arm_mat_mult_f32(&m_matrix, &other.m_matrix, &result.m_matrix);
    return result;
}

// 计算逆变换（针对刚体变换的优化方法）
HomogeneousTransform3D HomogeneousTransform3D::inverse() const 
{
    HomogeneousTransform3D result;
    
    // 创建一个临时的3x3矩阵用于旋转部分的计算
    float32_t rot_data[9];
    arm_matrix_instance_f32 rot = {3, 3, rot_data};

    // 提取原旋转矩阵的转置（即逆）
    rot_data[0] = m_data[0]; rot_data[1] = m_data[4]; rot_data[2] = m_data[8];
    rot_data[3] = m_data[1]; rot_data[4] = m_data[5]; rot_data[5] = m_data[9];
    rot_data[6] = m_data[2]; rot_data[7] = m_data[6]; rot_data[8] = m_data[10];
    
    // 将转置后的旋转矩阵复制到结果中
    result.m_data[0] = rot_data[0]; result.m_data[1] = rot_data[1]; result.m_data[2] = rot_data[2];
    result.m_data[4] = rot_data[3]; result.m_data[5] = rot_data[4]; result.m_data[6] = rot_data[5];
    result.m_data[8] = rot_data[6]; result.m_data[9] = rot_data[7]; result.m_data[10] = rot_data[8];

    // 计算新的平移部分: t' = -R^T * t
    float32_t t_in[3] = {m_data[3], m_data[7], m_data[11]};
    float32_t t_out[3];
    arm_matrix_instance_f32 t_in_mat = {3, 1, t_in};
    arm_matrix_instance_f32 t_out_mat = {3, 1, t_out};
    
    arm_mat_mult_f32(&rot, &t_in_mat, &t_out_mat);

    result.m_data[3] = -t_out[0];
    result.m_data[7] = -t_out[1];
    result.m_data[11] = -t_out[2];

    // 设置齐次坐标部分
    result.m_data[12] = 0; result.m_data[13] = 0; result.m_data[14] = 0; result.m_data[15] = 1;

    return result;
}


