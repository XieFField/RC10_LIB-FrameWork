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