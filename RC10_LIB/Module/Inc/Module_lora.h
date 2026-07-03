#pragma once

#include "Module_communication.h"
#include "RC_gpio_exti.h"
#include "BSP_USB_UART_Driver.h"
#include "stdint.h"

//测试通过 本机tx_error_cnt=1
#ifndef LORA_TEST_FORCE_TX_DMA_FAIL_ONCE
// 置 1：第一次启动 LoRa TX DMA 时模拟返回 HAL_BUSY。
// 现象：tx_error_cnt 增加 1，tx_busy 被清零，后续 AUX 触发仍可继续发送。
#define LORA_TEST_FORCE_TX_DMA_FAIL_ONCE 0
#endif

namespace tim { class Tim; }

#ifdef __cplusplus

namespace communication {

typedef struct{
    // ---- 摇杆原始值（0~1023）----
    uint16_t joystick1;   // 摇杆1 原始值
    uint16_t joystick2;   // 摇杆2 原始值
    uint16_t joystick3;   // 摇杆3 原始值
    uint16_t joystick4;   // 摇杆4 原始值
    // ---- 摇杆归一化值（-1.0 ~ 1.0）----
    float left_x;   //摇杆
    float left_y;
    float right_y;
    float right_x;
    // ---- 按键位图与页面 ----
    uint16_t key;         // 16位按键原始位图
    uint8_t page;         // 显示屏页面
    // ---- 按键统计 ----
    uint16_t key_pressed_count;   // 当前帧中被按下的按键个数 (0~16)
    uint16_t key_down_count;      // 累计检测到的按键按下次数（上升沿计数）
    uint16_t key_last_status;     // 上一帧的按键状态F

    // ---- 拨杆 ----
    uint8_t SWA;   uint8_t SWB; //拨杆
    uint8_t SWC;   uint8_t SWD;
    uint8_t SWE;   uint8_t SWF;

    // ---- 按键 ----
    uint8_t LB;   uint8_t RB;  //按键
    uint8_t LT;   uint8_t RT;

    // ---- 十字键 ----
    uint8_t d_pad_up;   uint8_t d_pad_down;   
    uint8_t d_pad_left; uint8_t d_pad_right; //十字键

    // ---- KFS 梅花桩接收数据 ----
    uint8_t KFS1_1, KFS1_2, KFS1_3;
    uint8_t KFS2_1, KFS2_2, KFS2_3, KFS2_4;
    uint8_t KFSf_1;
    uint8_t color;       // 我方颜色

    // ---- 串口屏命令帧 ----
    uint8_t recv_command_command;
    uint8_t recv_command_cnt;        // 当前命令的累计次数（单命令）
    uint8_t recv_command_total_cnt;  // 全部 12 个命令 (0~11) 的计数器总和
    uint8_t link_lost;

    // ---- 通信统计 -------
    uint16_t rx_drop_cnt;
    uint16_t tx_drop_cnt;
    uint16_t tx_error_cnt;
    uint16_t tx_timeout_recovery_cnt;
    uint16_t rx_crc_error_cnt;

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

    void Init();                //  初始化
    void Task_Process();        //  更新 airjoy_data 和 kfs_data 中的数据
    void Tim_It_Process();      //  定时器中断处理，负责定时触发发送

//-----------------------------R1 数据查询接口---------------------------------------
    const KFS_DATA_S& GetKFSData() const { return kfs_data; }

    void update_airjoy_data(RC10_AirJoy_Data_S * data);
    bool is_link_lost() const { return link_lost; }
    uint16_t get_rx_drop_cnt() const { return GetRxDropCnt(); }
    uint16_t get_tx_drop_cnt() const { return GetTxDropCnt(); }
    uint16_t get_tx_error_cnt() const { return GetTxErrorCnt(); }
    uint16_t get_tx_timeout_recovery_cnt() const { return GetTxTimeoutRecoveryCnt(); }
    uint16_t get_rx_crc_error_cnt() const { return GetRxCrcErrorCnt(); }

    void send_robot_pos(float x, float y, float yaw);

    void send_claw_status(bool claw1, bool claw2, bool claw3);

    void send_sucker_status(bool sucker);

    void send_robot_kfs_keepplace(uint8_t keepplace){send_kfs_keepplace = keepplace;}

    void set_robot_KFS_want_place(uint8_t want1, uint8_t want2, uint8_t want3);

    void send_robot_spear(bool spear1,bool spear2, bool spear3){send_spear = spear3|(spear2<<1)|(spear1<<2);}
    
    void send_robot_mode(uint8_t mode){send_mode = mode;}

    // void send_auto_status(bool auto_status);

    // void send_command(int8_t cmd);
//-----------------------------R1 数据查询接口---------------------------------------


//-----------------------------老接口（不建议使用）-----------------------------------
    void SetSendAxisData(int16_t x, int16_t y, int16_t z) {
        send_x = x; send_y = y; send_z = z;
    }

    void SetSendStatus(uint8_t gripper_status, uint8_t suction_cup_status, uint8_t automatic_status) {
        send_gripper_status = gripper_status;
        send_suction_cup_status = suction_cup_status;
        send_automatic_status = automatic_status;
    }

    void SetSendMode(uint8_t mode) { send_mode = mode; }

    void SetSendCommand(uint8_t command1, uint8_t command2) {
        send_chosen_command = command1; send_chosen_command_cnt = command2;
    }

    void SetSendWantKFSData(uint8_t KFS_want_place1, uint8_t KFS_want_place2) {
        send_kfs_want_place1 = KFS_want_place1;
        send_kfs_want_place2 = KFS_want_place2;
    }

    void SetSendSpearData(uint8_t spear) {
        send_spear = spear;
    }

    void SetSendKeepKFSData(uint8_t KFS_Keepplace) {
        send_kfs_keepplace = KFS_Keepplace;
    }
//-----------------------------老接口（不建议使用）-----------------------------------


protected:
    virtual HAL_StatusTypeDef Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) override;
    void EXTI_Prosess();        //  protectedֻ All_EXTI_Prosess
    static void RxCallback(uint8_t* buf, uint16_t len);

private:
    Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
         GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
          GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
           tim::Tim* timer);
    ~Lora_communication();


    UART_HandleTypeDef* lora_tx_huart;
    UART_HandleTypeDef* lora_rx_huart;
    GPIO_TypeDef* lora_aux_port;
    uint16_t lora_aux_pin;
    uint32_t timer_tick_count;
    uint32_t last_joystick_rx_tick;
    uint32_t last_joystick_frame_count;
    bool link_lost;
    
    uint8_t tx_ring_buffer[RING_BUF_SIZE];
    uint8_t rx_ring_buffer[RING_BUF_SIZE];
    alignas(32) uint8_t tx_dma_buffer[DMA_BUF_SIZE];
    alignas(32) uint8_t rx_dma_buffer[DMA_BUF_SIZE];

    UART_ bsp_rx;
    tim::Tim* attached_timer;


    static Lora_communication* s_instance;
    uint16_t send_x, send_y, send_z;
    uint8_t send_gripper_status;
    uint8_t send_suction_cup_status;
    uint8_t send_automatic_status;
    uint8_t send_mode;
    uint8_t send_chosen_command, send_chosen_command_cnt;
    uint8_t send_kfs_want_place1, send_kfs_want_place2;
    uint8_t send_spear;
    uint8_t send_kfs_keepplace;
    RC10_AirJoy_Data_S airjoy_data; // 存储解析后的遥控器数据
    KFS_DATA_S kfs_data;



};

}

#endif
