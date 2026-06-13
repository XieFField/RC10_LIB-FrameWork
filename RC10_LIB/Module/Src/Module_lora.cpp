#include "Module_lora.h"
#include "usart.h"
#include "main.h"
#include <cmath>
uint8_t KFS_want1;
uint8_t KFS_want2;
uint8_t spear;
int16_t Axis_x;
int16_t Axis_y;
int16_t Axis_yaw;
namespace {
static inline float NormalizeAxis(uint16_t raw, float center, float span, float deadzone = 0.05f)
{
    float value = (static_cast<float>(raw) - center) / span;
    if (std::fabs(value) < deadzone) {
        value = 0.0f;
    }
    return value;
}

static inline uint8_t DecodeSwitchLevel(uint16_t raw, uint8_t shift)
{
    uint8_t value = static_cast<uint8_t>((raw >> shift) & 0x03U);
    if (value > 2U) {
        value = 2U;
    }
    return value;
}

static inline uint8_t DecodeSwitchBit(uint16_t raw, uint8_t shift)
{
    return static_cast<uint8_t>((raw >> shift) & 0x01U);
}

static inline uint16_t PackSigned16(float value, float scale)
{
    int32_t scaled = static_cast<int32_t>(std::round(value * scale));
    if (scaled > INT16_MAX) {
        scaled = INT16_MAX;
    } else if (scaled < INT16_MIN) {
        scaled = INT16_MIN;
    }
    return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}
}

namespace communication {

/* ========== 静态成员定义 ========== */
Lora_communication* Lora_communication::s_instance = nullptr;

/* ========== 单例获取 ========== */
Lora_communication* Lora_communication::GetInstance()
{
    static Lora_communication instance(&huart5, &huart6,
                                       GPIOB, GPIO_PIN_10,
                                       GPIOB, GPIO_PIN_11,
                                       nullptr);
    if (s_instance == nullptr) {
        s_instance = &instance;
    }
    return &instance;
}

/* ========== 构造 / 析构 ========== */
Lora_communication::Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
     GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
      GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
       tim::Tim* timer)
    : Communication(tx_huart, rx_huart,
        tx_ring_buffer, tx_dma_buffer, rx_ring_buffer, rx_dma_buffer,
        tx_aux_gpio_port, tx_aux_gpio_pin, rx_aux_gpio_port, rx_aux_gpio_pin),
      GpioExti(tx_aux_gpio_pin),
      bsp_rx(DMA_BUF_SIZE, rx_dma_buffer, rx_huart),
      attached_timer(timer),
      pending_x_raw_(0),
      pending_y_raw_(0),
      pending_yaw_raw_(0),
      pending_claw_status_(0),
      pending_sucker_status_(0),
      pending_mode_(0x01),
    pending_command_(0),
      pending_tx_dirty_(false),
      auto_mode_(false),
      send_kfs_want_place1_(0),
      send_kfs_want_place2_(0),
      send_spear_(0),
      send_kfs_keepplace_(0),
      recv_command_command_(1),
      recv_command_load1_(0),
      recv_command_load2_(0),
      chosen_command_cnt_(0),
      key_pressed_count_(0),
      key_down_count_(0),
      key_last_status_(0),
      airjoy_data_()
{
    lora_tx_huart = tx_huart;
    lora_rx_huart = rx_huart;
    lora_aux_port = tx_aux_gpio_port;
    lora_aux_pin = tx_aux_gpio_pin;
    
    s_instance = this;
    // 初始化 KFS 结构体为 0
    kfs_data_ = {};
    send_x = 0; send_y = 0; send_z = 0;
    send_gripper_status = 0;
    send_suction_cup_status = 0;
    send_automatic_status = 0;
    send_mode = 0;
    chosen_command = 1; chosen_command_cnt = 0;
    send_kfs_want_place1 = 0; send_kfs_want_place2 = 0;
    send_spear = 0;
    send_kfs_keepplace = 0;
}

Lora_communication::~Lora_communication() {
}

/* ========== 初始化 ========== */
void Lora_communication::Init() {

    bsp_rx.SetCallback(RxCallback);
    bsp_rx.UART_Init();
}

/* ========== 静态回调 ========== */
void Lora_communication::RxCallback(uint8_t* buf, uint16_t len) {
    if (s_instance) {
        s_instance->Comm_RxDMAToRxBuffer(s_instance->lora_rx_huart, len);
    }
}

/* ========== 虚函数实现 ========== */
void Lora_communication::Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) {
    if (huart != nullptr && huart == lora_tx_huart) {
        HAL_UART_Transmit_DMA(huart, data, size);
    }
}

void Lora_communication::Task_Process() {
    if (Comm_Task_Loop()) {
        SetSendAxisData(Axis_x, Axis_y, Axis_yaw);
        SetSendWantKFSData(KFS_want1, KFS_want2);
        SetSendSpearData(spear);

        uint16_t joystick[4];
        // 新的 Communication 接口：分别获取摇杆和按键/设置数据
        GetRecvJoystickData(joystick);
        uint16_t key = GetRecvAllKeyData();

        kfs_data_.color = GetColor();

        kfs_data_.r1_kfs[0] = GetRecvFKFS1Data(1);
        kfs_data_.r1_kfs[1] = GetRecvFKFS1Data(2);
        kfs_data_.r1_kfs[2] = GetRecvFKFS1Data(3);

        kfs_data_.r2_kfs[0] = GetRecvFKFS2Data(1);
        kfs_data_.r2_kfs[1] = GetRecvFKFS2Data(2);
        kfs_data_.r2_kfs[2] = GetRecvFKFS2Data(3);
        kfs_data_.r2_kfs[3] = GetRecvFKFS2Data(4);

        kfs_data_.fake_kfs = GetRecvFKFSfData(1);

        // 读取串口屏转发的命令帧（0xAA 0x77），并将命令/累计次数存入发送变量
        GetChosenCommandAndCnt(recv_command_command_, recv_command_load1_, recv_command_load2_);
        chosen_command_cnt_ = recv_command_load1_;  // 累计次数作为 command2 发送

        airjoy_data_.page = GetPage();

        airjoy_data_.left_x  = NormalizeAxis(joystick[0], 512.0f, 512.0f);
        airjoy_data_.left_y  = NormalizeAxis(joystick[1], 512.0f, 512.0f);
        airjoy_data_.right_x = NormalizeAxis(joystick[2], 512.0f, 512.0f);
        airjoy_data_.right_y = NormalizeAxis(joystick[3], 512.0f, 512.0f);

        airjoy_data_.SWA = DecodeSwitchLevel(key, 0);
        airjoy_data_.SWB = DecodeSwitchBit(key, 2);
        airjoy_data_.SWC = DecodeSwitchBit(key, 3);
        airjoy_data_.SWD = DecodeSwitchBit(key, 4);
        airjoy_data_.SWE = DecodeSwitchBit(key, 5);
        airjoy_data_.SWF = DecodeSwitchLevel(key, 6);

        airjoy_data_.d_pad_up    = static_cast<uint8_t>((key >> 8) & 0x01U);
        airjoy_data_.d_pad_down  = static_cast<uint8_t>((key >> 9) & 0x01U);
        airjoy_data_.d_pad_left  = static_cast<uint8_t>((key >> 10) & 0x01U);
        airjoy_data_.d_pad_right = static_cast<uint8_t>((key >> 11) & 0x01U);

        airjoy_data_.LB = static_cast<uint8_t>((key >> 12) & 0x01U);
        airjoy_data_.RB = static_cast<uint8_t>((key >> 13) & 0x01U);
        airjoy_data_.LT = static_cast<uint8_t>((key >> 14) & 0x01U);
        airjoy_data_.RT = static_cast<uint8_t>((key >> 15) & 0x01U);

        uint16_t key_status = key;
        for (uint8_t i = 0; i < 16; ++i) {
            if (key_status & (1U << i)) {
                key_pressed_count_++;
            }
        }

        uint16_t rising_edges = static_cast<uint16_t>(key_status & (~key_last_status_));
        for (uint8_t i = 0; i < 16; ++i) {
            if (rising_edges & (1U << i)) {
                key_down_count_++;
            }
        }
        key_last_status_ = key_status;

        // ====== 同步全部数据到 airjoy_data_，方便 debug 查看 rc_data ======
        airjoy_data_.joystick1 = joystick[0];
        airjoy_data_.joystick2 = joystick[1];
        airjoy_data_.joystick3 = joystick[2];
        airjoy_data_.joystick4 = joystick[3];
        airjoy_data_.key  = key;
        airjoy_data_.page = airjoy_data_.page; // 上面已赋值

        airjoy_data_.key_pressed_count = key_pressed_count_;
        airjoy_data_.key_down_count    = key_down_count_;
        airjoy_data_.key_last_status   = key_last_status_;

        airjoy_data_.KFS1_1 = kfs_data_.r1_kfs[0];
        airjoy_data_.KFS1_2 = kfs_data_.r1_kfs[1];
        airjoy_data_.KFS1_3 = kfs_data_.r1_kfs[2];
        airjoy_data_.KFS2_1 = kfs_data_.r2_kfs[0];
        airjoy_data_.KFS2_2 = kfs_data_.r2_kfs[1];
        airjoy_data_.KFS2_3 = kfs_data_.r2_kfs[2];
        airjoy_data_.KFS2_4 = kfs_data_.r2_kfs[3];
        airjoy_data_.KFSf_1 = kfs_data_.fake_kfs;
        airjoy_data_.color  = kfs_data_.color;

        airjoy_data_.recv_command_command = recv_command_command_;
        airjoy_data_.recv_command_load1  = recv_command_load1_;
        airjoy_data_.recv_command_load2  = recv_command_load2_;
    }
}

void Lora_communication::flush_pending_frame()
{
//    if (!pending_tx_dirty_) {
//        return;
//    }

//    Comm_SendAxisDataToTxBuffer(
//        pending_x_raw_,
//        pending_y_raw_,
//        pending_yaw_raw_,
//        pending_claw_status_,
//        pending_sucker_status_,
//        auto_mode_,
//        pending_mode_,
//        pending_command_,
//        chosen_command_cnt_,
//        send_kfs_want_place1_,
//        send_kfs_want_place2_,
//        send_spear_,
//        send_kfs_keepplace_);

//    pending_tx_dirty_ = false;
}

void Lora_communication::send_robot_pos(float x, float y, float yaw)
{
    pending_x_raw_ = PackSigned16(x, 100.0f);
    pending_y_raw_ = PackSigned16(y, 100.0f);
    pending_yaw_raw_ = PackSigned16(yaw, 100.0f);
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

void Lora_communication::send_claw_status(bool claw1, bool claw2, bool claw3)
{
    pending_claw_status_ = static_cast<uint8_t>((claw1 ? 0x01U : 0x00U) |
                                               (claw2 ? 0x02U : 0x00U) |
                                               (claw3 ? 0x04U : 0x00U));
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

void Lora_communication::send_sucker_status(bool sucker1, bool sucker2)
{
    pending_sucker_status_ = static_cast<uint8_t>((sucker1 ? 0x01U : 0x00U) |
                                                 (sucker2 ? 0x02U : 0x00U));
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

void Lora_communication::send_auto_status(bool auto_status)
{
    auto_mode_ = auto_status;
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

void Lora_communication::send_command(int8_t cmd)
{
    pending_command_ = static_cast<uint8_t>(cmd);
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

/* ========== 定时器中断 ========== */
void Lora_communication::Tim_It_Process() {
    timer_tick_count++;
    if (timer_tick_count >= 2) { // 计数达到 1ms 
        timer_tick_count = 0;
        GetChosenCommandAndCnt(chosen_command, chosen_command_cnt, recv_command_load2);
        Comm_SendAxisDataToTxBuffer(send_x, send_y, send_z,
            send_gripper_status, send_suction_cup_status, send_automatic_status,
            send_mode, chosen_command, chosen_command_cnt,
            send_kfs_want_place1, send_kfs_want_place2, send_spear, send_kfs_keepplace);
    }
}

/* ========== GPIO 中断处理 ========== */
void Lora_communication::EXTI_Prosess() {
    Comm_TxBufferToTxDMA(lora_tx_huart);
}

} // namespace communication