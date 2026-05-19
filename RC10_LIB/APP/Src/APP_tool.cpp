/**
 * @file    APP_tool.cpp
 * @brief   Some tools for the project
 * @author  XieFField
 * @version 1.0
 */

#include "APP_tool.h"

/**
 * @brief Perform binary search on a sorted array
 */
int binarySearch(const uint32_t arr[], uint8_t count, uint32_t key)
{
    int low = 0, high = count - 1;
    while (low <= high)
    {
        int mid = (low + high) >> 1;
        if (arr[mid] == key)
            return mid;
        if (arr[mid] < key)
            low = mid + 1;
        else
            high = mid - 1;
    }
    return -1;
}

/**
 * @brief  将矩阵设置为单位矩阵
 * @param[in,out] M   指向矩阵实例
 * @note   要求矩阵是方阵 (numRows == numCols)
 */
void arm_set_identity_f32(arm_matrix_instance_f32 *M)
{
    if (M->numRows != M->numCols) 
        return; // 不是方阵，直接返回
    

    uint32_t n = M->numRows;
    uint32_t size = n * n;

    // 先清零 uint64_t
    arm_fill_f32(0.0f, M->pData, size);

    // 设置对角线为1
    for (uint32_t i = 0; i < n; i++) 
        M->pData[i * n + i] = 1.0f;
    
}

//斜坡函数
void ramp(float target, float& current, float max_change_rate, float dt)
{
    float error = target - current;
    float max_change = max_change_rate * dt;
    current += constrain(error, -max_change, max_change);
}

//弧度转换为角度函数
float rad_to_deg(float rad)
{
    return rad / PI * 180.0f;
}

//角度转换为弧度函数
float deg_to_rad(float deg)
{
    return deg / 180.0f * PI;
 }


//归一化角度到[0,360)区间
float normalize_deg_0_360(float a)
{
    float r = fmodf(a, 360.0f);
    while (r < 0.0f) r += 360.0f;
    if (r == 0.0f) r = 0.0f; // 特判处理 -0.0f 为 +0.0f
    return r;            // [0,360)
}

//归一化角度到[-180,180)区间
float normalize_deg_pm180(float a)
{
    float r = fmodf(a + 180.0f, 360.0f);
    if (r < 0.0f) r += 360.0f;
    if (r == 0.0f) r = 0.0f; // 特判处理 -0.0f 为 +0.0f
    return r - 180.0f;   // [-180,180)
}

// 将 val_deg 映射到“最接近 ref_deg(0..360)”的等价角，并返回 0..360
float wrap_to_nearest_0_360(float ref_deg_0_360, float val_deg_any)
{
    float ref = normalize_deg_0_360(ref_deg_0_360);
    float delta = normalize_deg_pm180(val_deg_any - ref);  // 差值用 ±180 归一化

    return normalize_deg_0_360(ref + delta);               // 最终保持 0..360
}

// 将 val_deg（任意/0..360）映射到“最接近 ref_deg_cont（连续角）”的等效角，返回连续角（可超出0..360）
float wrap_to_nearest_cont(float ref_deg_cont, float val_deg_any)
{
    float base = normalize_deg_0_360(val_deg_any);
    float k = roundf((ref_deg_cont - base) / 360.0f);

    if(k == 0.0f) k = 0.0f; // 特判处理 -0.0f 为 +0.0f
    return base + 360.0f * k; // 连续角
}