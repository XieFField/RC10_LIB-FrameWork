#include "Module_Camera.h"
#include <cstring>

Module_Camera* Module_Camera::GetInstance(UART_HandleTypeDef *uart_handle) 
{
    // 定义静态接收缓冲区
    static uint8_t static_rx_buffer[64] = {0};
    static Module_Camera instance(64, static_rx_buffer, uart_handle);
    return &instance;
}

Module_Camera::Module_Camera(uint16_t rx_buffer_size, uint8_t *rx_buffer, UART_HandleTypeDef *uart_handle) 
    : UART_(rx_buffer_size, rx_buffer, uart_handle),
      uart_instance_(nullptr),
      uart_initialized_(false)
{
}

void Module_Camera::InitUART() 
{
    if (uart_initialized_) return;
    
    // 模仿 Module_Position: 使用作用域解析符调用基类静态方法
    UART_HandleTypeDef *uart_handle = Module_Camera::UART_::GetUartHandle();
    
    // 使用 InstanceManager 工具
    uart_instance_ = InstanceManager::GetInstanceByUartHandle(uart_handle);
    
    if (uart_instance_ != nullptr) {
        uart_instance_->UART_Init();
        uart_initialized_ = true;
    }
}

void Module_Camera::Callback_Fuc(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        
        switch (rx_state) {
            case WAITING_FOR_HEAD_0:
                if (byte == FRAME_HEAD_0) { // 0xAA
                    rx_state = WAITING_FOR_HEAD_1;
                }
                break;

            case WAITING_FOR_HEAD_1:
                if (byte == FRAME_HEAD_1) { // 0xBB
                    rx_state = WAITING_FOR_DATA;
                    data_index = 0;
                } else {
                    if (byte == FRAME_HEAD_0) rx_state = WAITING_FOR_HEAD_1;
                    else rx_state = WAITING_FOR_HEAD_0;
                }
                break;

            case WAITING_FOR_DATA:
                data_buffer[data_index++] = byte;
                if (data_index >= DATA_LEN) {
                    rx_state = WAITING_FOR_TAIL_0;
                }
                break;

            case WAITING_FOR_TAIL_0:
                if (byte == FRAME_TAIL_0) { // 0xCC
                    rx_state = WAITING_FOR_TAIL_1;
                } else {
                    rx_state = WAITING_FOR_HEAD_0; 
                }
                break;

            case WAITING_FOR_TAIL_1:
                if (byte == FRAME_TAIL_1) { // 0xDD
                    memcpy(&current_data_, data_buffer, sizeof(Camera_Data_t));
                    last_update_time_ = HAL_GetTick();
                    is_data_valid = true;
                    frame_seq_++;
                    rx_state = WAITING_FOR_HEAD_0;
                } else {
                    rx_state = WAITING_FOR_HEAD_0;
                }
                break;

            default:
                rx_state = WAITING_FOR_HEAD_0;
                break;
        }
    }
}

Camera_Data_t Module_Camera::GetCameraData() {
    return current_data_;
}

uint32_t Module_Camera::GetFrameSeq() const {
    return frame_seq_;
}

bool Module_Camera::IsConnected() {
    return (HAL_GetTick() - last_update_time_) < 500;
}