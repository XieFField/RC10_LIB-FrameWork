#pragma once

#include "Module_communication.h"
#include "RC_gpio_exti.h"
#include "BSP_USB_UART_Driver.h"
#include "stdint.h"

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
    uint16_t key_last_status;     // 上一帧的按键状态

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
    uint8_t recv_command_load1;
    uint8_t recv_command_load2;

    // ---- KFS 发送目标 ----
    uint8_t KFS_want1;
    uint8_t KFS_want2;

    // ---- 武器头等 ----
    uint8_t spear;

    // ---- 底盘轴控（发送用，有符号）----
    int16_t Axis_x;
    int16_t Axis_y;
    int16_t Axis_yaw;
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

        data->joystick1 = airjoy_data_.joystick1;
        data->joystick2 = airjoy_data_.joystick2;
        data->joystick3 = airjoy_data_.joystick3;
        data->joystick4 = airjoy_data_.joystick4;

        data->key  = airjoy_data_.key;
        data->page = airjoy_data_.page;

        data->left_x = airjoy_data_.left_x;
        data->left_y = airjoy_data_.left_y;
        data->right_y = airjoy_data_.right_x;
        data->right_x = airjoy_data_.right_y;

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

        data->key_pressed_count = airjoy_data_.key_pressed_count;
        data->key_down_count    = airjoy_data_.key_down_count;
        data->key_last_status   = airjoy_data_.key_last_status;

        data->KFS1_1 = airjoy_data_.KFS1_1;
        data->KFS1_2 = airjoy_data_.KFS1_2;
        data->KFS1_3 = airjoy_data_.KFS1_3;
        data->KFS2_1 = airjoy_data_.KFS2_1;
        data->KFS2_2 = airjoy_data_.KFS2_2;
        data->KFS2_3 = airjoy_data_.KFS2_3;
        data->KFS2_4 = airjoy_data_.KFS2_4;
        data->KFSf_1 = airjoy_data_.KFSf_1;
        data->color  = airjoy_data_.color;

        data->recv_command_command = airjoy_data_.recv_command_command;
        data->recv_command_load1  = airjoy_data_.recv_command_load1;
        data->recv_command_load2  = airjoy_data_.recv_command_load2;

        data->KFS_want1 = airjoy_data_.KFS_want1;
        data->KFS_want2 = airjoy_data_.KFS_want2;
        data->spear     = airjoy_data_.spear;

        data->Axis_x   = airjoy_data_.Axis_x;
        data->Axis_y   = airjoy_data_.Axis_y;
        data->Axis_yaw = airjoy_data_.Axis_yaw;
    }

    void send_robot_pos(float x, float y, float yaw);

    void send_claw_status(bool claw1, bool claw2, bool claw3);

    void send_sucker_status(bool sucker1, bool sucker2);

    void send_auto_status(bool auto_status);

    void send_command(int8_t cmd);


    /**
     * @brief 获取接收到的命令帧数据（串口屏转发）
     * @param command 存放命令的变量
     * @param load1 存放负载1的变量（累计次数）
     * @param load2 存放负载2的变量（保留扩展）
     */
    void GetChosenCommandAndCnt(uint8_t& command, uint8_t& load1, uint8_t& load2) {
        Communication::GetRecvCommandFrameData(command, load1, load2);
    }
    void SetSendAxisData(uint16_t x, uint16_t y, uint16_t z) {
        send_x = x; send_y = y; send_z = z;
        pending_x_raw_ = x; pending_y_raw_ = y; pending_yaw_raw_ = z;
    }

    void SetSendStatus(uint8_t gripper_status, uint8_t suction_cup_status, uint8_t automatic_status) {
        send_gripper_status = gripper_status;
        send_suction_cup_status = suction_cup_status;
        send_automatic_status = automatic_status;
    }

    void SetSendMode(uint8_t mode) { send_mode = mode; }

    void SetSendCommand(uint8_t command1, uint8_t command2) {
        chosen_command = command1; chosen_command_cnt = command2;
    }

    // ---- 发送数据 setter 接口（外部调用修改发送参数）----
    void SetSendWantKFSData(uint8_t KFS_want_place1, uint8_t KFS_want_place2) {
        send_kfs_want_place1_ = KFS_want_place1;
        send_kfs_want_place2_ = KFS_want_place2;
    }

    void SetSendSpearData(uint8_t spear) {
        send_spear_ = spear;
    }

    void SetSendKeepKFSData(uint8_t KFS_Keepplace) {
        send_kfs_keepplace_ = KFS_Keepplace;
    }
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

    // 待发送的 KFS 相关数据（填入 XYZ 帧扩展字段）
    uint8_t send_kfs_want_place1_;
    uint8_t send_kfs_want_place2_;
    uint8_t send_spear_;
    uint8_t send_kfs_keepplace_;

    // 命令帧相关（串口屏转发）
    uint8_t recv_command_command_;
    uint8_t recv_command_load1_;
    uint8_t recv_command_load2_;  // 保留扩展
    uint8_t chosen_command_cnt_;  // 发送帧 command2（累计次数）

    uint16_t key_pressed_count_;
    uint16_t key_down_count_;
    uint16_t key_last_status_;

    KFS_DATA_S kfs_data_;

    static Lora_communication* s_instance;
    static void RxCallback(uint8_t* buf, uint16_t len);
    uint16_t send_x, send_y, send_z;
    uint8_t send_gripper_status;
    uint8_t send_suction_cup_status;
    uint8_t send_automatic_status;
    uint8_t send_mode;
    uint8_t chosen_command, chosen_command_cnt, recv_command_load2;
    uint8_t send_kfs_want_place1, send_kfs_want_place2;
    uint8_t send_spear;
    uint8_t send_kfs_keepplace;
    RC10_AirJoy_Data_S airjoy_data_; // 存储解析后的遥控器数据
};

}

#endif