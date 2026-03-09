#ifndef __MODULE_CAMERA_H
#define __MODULE_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "usart.h"
#include <stdint.h>
#include "BSP_USB_UART_Driver.h"
#include "APP_DebugTool.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// 摄像头数据结构体 (保持与上位机 PnPData 内存布局一致)
#pragma pack(1)
struct Camera_Data_t {
    float x;    // [0-3]
    float y;    // [4-7]
    float yaw;  // [8-11]
};
#pragma pack()

class Module_Camera : public UART_ {
public:
    /**
     * @brief 获取单例实例
     * @param uart_handle 串口句柄 (如 &huart6)
     * @return Module_Camera* 
     */
    static Module_Camera* GetInstance(UART_HandleTypeDef *uart_handle);

    /**
     * @brief 初始化串口
     */
    void InitUART();

    /**
     * @brief 串口中断回调函数 (状态机解析)
     */
    void Callback_Fuc(uint8_t *buf, uint16_t len) override;

    /**
     * @brief 获取最新摄像头数据
     */
    Camera_Data_t GetCameraData();

    /**
     * @brief 检查摄像头是否在线
     * @return true 在线 (最近500ms有收到合法数据)
     */
    bool IsConnected();

private:
    // 私有构造函数，实现单例模版
    Module_Camera(uint16_t rx_buffer_size, uint8_t *rx_buffer, UART_HandleTypeDef *uart_handle);
    
    // 禁用拷贝
    Module_Camera(const Module_Camera&) = delete;
    Module_Camera& operator=(const Module_Camera&) = delete;

    // 协议常量
    static const uint8_t FRAME_HEAD_0 = 0xAA;
    static const uint8_t FRAME_HEAD_1 = 0xBB;
    static const uint8_t FRAME_TAIL_0 = 0xCC;
    static const uint8_t FRAME_TAIL_1 = 0xDD;
    static const uint8_t DATA_LEN = 12; // 3个float

    // 解析状态机
    enum RxState {
        WAITING_FOR_HEAD_0,
        WAITING_FOR_HEAD_1,
        WAITING_FOR_DATA,
        WAITING_FOR_TAIL_0,
        WAITING_FOR_TAIL_1
    };

    // 模仿 Module_Position: 使用 UART_* 类型
    UART_* uart_instance_;
    bool uart_initialized_;
    
    RxState rx_state = WAITING_FOR_HEAD_0;
    uint8_t data_buffer[12]; // 暂存数据
    uint8_t data_index = 0;

    Camera_Data_t current_data_ = {0.0f, 0.0f, 0.0f};
    bool is_data_valid = false;
    uint32_t last_update_time_ = 0;
};

#endif // __cplusplus
#endif // __MODULE_CAMERA_H