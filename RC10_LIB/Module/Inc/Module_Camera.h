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

// ����ͷ���ݽṹ�� (��������λ�� PnPData �ڴ沼��һ��)
#pragma pack(1)
struct Camera_Data_t {
    float x;    // [0-3]
    float y;    // [4-7]
    float z;    // [8-11]
    float yaw;  // [12-15]
};
#pragma pack()

class Module_Camera : public UART_ {
public:
    /**
     * @brief ��ȡ����ʵ��
     * @param uart_handle ���ھ��? (�� &huart6)
     * @return Module_Camera* 
     */
    static Module_Camera* GetInstance(UART_HandleTypeDef *uart_handle);

    /**
     * @brief ��ʼ������
     */
    void InitUART();

    /**
     * @brief �����жϻص����� (״̬������)
     */
    void Callback_Fuc(uint8_t *buf, uint16_t len) override;

    /**
     * @brief ��ȡ��������ͷ����
     */
    Camera_Data_t GetCameraData();

    /**
     * @brief �������ͷ�Ƿ�����?
     * @return true ���� (���?500ms���յ��Ϸ�����)
     */
    bool IsConnected();

private:
    // ˽�й��캯����ʵ�ֵ���ģ��
    Module_Camera(uint16_t rx_buffer_size, uint8_t *rx_buffer, UART_HandleTypeDef *uart_handle);
    
    // ���ÿ���
    Module_Camera(const Module_Camera&) = delete;
    Module_Camera& operator=(const Module_Camera&) = delete;

    // Э�鳣��
    static const uint8_t FRAME_HEAD_0 = 0xAA;
    static const uint8_t FRAME_HEAD_1 = 0xBB;
    static const uint8_t FRAME_TAIL_0 = 0xCC;
    static const uint8_t FRAME_TAIL_1 = 0xDD;
    static const uint8_t DATA_LEN = 16; // 4��float: x/y/z/yaw

    // ����״̬��
    enum RxState {
        WAITING_FOR_HEAD_0,
        WAITING_FOR_HEAD_1,
        WAITING_FOR_DATA,
        WAITING_FOR_TAIL_0,
        WAITING_FOR_TAIL_1
    };

    // ģ�� Module_Position: ʹ�� UART_* ����
    UART_* uart_instance_;
    bool uart_initialized_;
    
    RxState rx_state = WAITING_FOR_HEAD_0;
    uint8_t data_buffer[16]; // �ݴ�����
    uint8_t data_index = 0;

    Camera_Data_t current_data_ = {0.0f, 0.0f, 0.0f, 0.0f};
    bool is_data_valid = false;
    uint32_t last_update_time_ = 0;
};

#endif // __cplusplus
#endif // __MODULE_CAMERA_H