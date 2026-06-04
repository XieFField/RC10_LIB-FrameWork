#pragma once

#include "Module_communication.h"
#include "RC_gpio_exti.h"
#include "BSP_USB_UART_Driver.h"
#include "stdint.h"

namespace tim { class Tim; }

#ifdef __cplusplus

namespace communication {

typedef struct{
    float left_x;   //摇杆
    float left_y;
    float right_x;
    float right_y;
    uint8_t page;   //显示屏页面

    uint8_t SWA;   uint8_t SWB; //拨杆
    uint8_t SWC;   uint8_t SWD;
    uint8_t SWE;   uint8_t SWF;

    uint8_t LB;   uint8_t RB;  //按键
    uint8_t LT;   uint8_t RT;

    uint8_t d_pad_up;   uint8_t d_pad_down;   
    uint8_t d_pad_left; uint8_t d_pad_right; //十字键
}RC10_AirJoy_Data_S;

/* KFS 梅花桩位置数据结构体 */
typedef struct {
    uint8_t r1_kfs[3];   // KFS1 三个位置
    uint8_t r2_kfs[4];   // KFS2 四个位置
    uint8_t fake_kfs;    // KFSf 位置
    uint8_t color;       // 我方颜色
} KFS_DATA_S;

class Lora_communication : public Communication, public gpio::GpioExti {
public:
    static Lora_communication* GetInstance();

    void Init();
    void Task_Process();        //  public
    void Tim_It_Process();      //  public

    void update_airjoy_data(RC10_AirJoy_Data_S * data)
    {
        if(!data) return;

        data->left_x = airjoy_data_.left_x;
        data->left_y = airjoy_data_.left_y;
        data->right_x = airjoy_data_.right_x;
        data->right_y = airjoy_data_.right_y;
        data->page = airjoy_data_.page;

        data->SWA = airjoy_data_.SWA;
        data->SWB = airjoy_data_.SWB;
        data->SWC = airjoy_data_.SWC;
        data->SWD = airjoy_data_.SWD;
        data->SWE = airjoy_data_.SWE;
        data->SWF = airjoy_data_.SWF;

        data->LB = airjoy_data_.LB;
        data->RB = airjoy_data_.RB;
        data->LT = airjoy_data_.LT;
        data->RT = airjoy_data_.RT;

        data->d_pad_up = airjoy_data_.d_pad_up;
        data->d_pad_down = airjoy_data_.d_pad_down;
        data->d_pad_left = airjoy_data_.d_pad_left;
        data->d_pad_right = airjoy_data_.d_pad_right;
    }

    void send_robot_pos(float x, float y, float yaw);

    void send_claw_status(bool claw1, bool claw2, bool claw3);

    void send_sucker_status(bool sucker1, bool sucker2);

    void send_auto_status(bool auto_status);

    void send_command(int8_t cmd);

    // KFS 数据对外查询接口
    const KFS_DATA_S& GetKFSData() const { return kfs_data_; }

protected:
    virtual void Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) override;
    void EXTI_Prosess();        //  protectedֻ All_EXTI_Prosess

private:
    Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
         GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
          GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
           tim::Tim* timer);
    ~Lora_communication();

    void flush_pending_frame();

    UART_HandleTypeDef* lora_tx_huart;
    UART_HandleTypeDef* lora_rx_huart;
    GPIO_TypeDef* lora_aux_port;
    uint16_t lora_aux_pin;
    uint32_t timer_tick_count;
    
    uint8_t tx_ring_buffer[RING_BUF_SIZE];
    uint8_t rx_ring_buffer[RING_BUF_SIZE];
    alignas(32) uint8_t tx_dma_buffer[DMA_BUF_SIZE];
    alignas(32) uint8_t rx_dma_buffer[DMA_BUF_SIZE];

    UART_ bsp_rx;
    tim::Tim* attached_timer;

    uint16_t pending_x_raw_;
    uint16_t pending_y_raw_;
    uint16_t pending_yaw_raw_;
    uint8_t pending_claw_status_;
    uint8_t pending_sucker_status_;
    uint8_t pending_mode_;
    uint8_t pending_command_;
    bool pending_tx_dirty_;
    bool auto_mode_;

    uint16_t key_pressed_count_;
    uint16_t key_down_count_;
    uint16_t key_last_status_;

    KFS_DATA_S kfs_data_;

    static Lora_communication* s_instance;
    static void RxCallback(uint8_t* buf, uint16_t len);

    RC10_AirJoy_Data_S airjoy_data_; // 存储解析后的遥控器数据
};

}

#endif