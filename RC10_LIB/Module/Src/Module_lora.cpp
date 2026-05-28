#include "Module_lora.h"
#include "usart.h"
#include "main.h"
#include <cmath>

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    communication::Lora_communication::All_EXTI_Prosess(GPIO_Pin);
}

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
Lora_communication* Lora_communication::gpio_exti_list[MAX_GPIO_EXTI_NUM] = {nullptr};

/* ========== 单例获取 ========== */
Lora_communication* Lora_communication::GetInstance()
{
    static Lora_communication instance(&huart5, &huart6,
                                       Lora_IO1_GPIO_Port, Lora_IO1_Pin,
                                       Lora_IO2_GPIO_Port, Lora_IO2_Pin,
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
      key_pressed_count_(0),
      key_down_count_(0),
      key_last_status_(0),
      airjoy_data_()
{
    lora_tx_huart = tx_huart;
    lora_rx_huart = rx_huart;
    lora_aux_port = tx_aux_gpio_port;
    lora_aux_pin = tx_aux_gpio_pin;

    uint8_t pin = __builtin_ctz(tx_aux_gpio_pin);
    if (pin < MAX_GPIO_EXTI_NUM) {
        gpio_exti_list[pin] = this;
    }
    
    s_instance = this;
}

Lora_communication::~Lora_communication() {
}

/* ========== 初始化 ========== */
void Lora_communication::Init() {
    lora_tx_huart = &huart5;
    lora_rx_huart = &huart6;
    lora_aux_port = Lora_IO1_GPIO_Port;
    lora_aux_pin = Lora_IO1_Pin;

    uint8_t pin = __builtin_ctz(lora_aux_pin);
    if (pin < MAX_GPIO_EXTI_NUM) {
        gpio_exti_list[pin] = this;
    }

    bsp_rx.SetCallback(RxCallback);
    bsp_rx.UART_Init();
}

/* ========== 静态回调 ========== */
void Lora_communication::RxCallback(uint8_t* buf, uint16_t len) {
    if (s_instance) {
        s_instance->Comm_RxDMAToRxBuffer(s_instance->lora_rx_huart, len);
    }
}

/* ========== EXTI 分发 ========== */
void Lora_communication::All_EXTI_Prosess(uint16_t gpio_pin_) {
    uint8_t pin = __builtin_ctz(gpio_pin_);
    if (pin < MAX_GPIO_EXTI_NUM) {
        if (gpio_exti_list[pin] != nullptr) {
            gpio_exti_list[pin]->EXTI_Prosess();
        }
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
        uint16_t joystick[4];
        uint16_t key;
        GetRecvData(joystick, key);
        
        uint8_t command, load1, load2;
        GetSettingData(command, load1, load2);
        (void)command;
        (void)load1;
        (void)load2;

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

        uint16_t key_status = GetKeyStatus();
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
    }
}

void Lora_communication::flush_pending_frame()
{
    if (!pending_tx_dirty_) {
        return;
    }

    Comm_SendAxisDataToTxBuffer(
        pending_x_raw_,
        pending_y_raw_,
        pending_yaw_raw_,
        pending_claw_status_,
        pending_sucker_status_,
        auto_mode_,
        pending_mode_,
        pending_command_,
        0x00);

    pending_tx_dirty_ = false;
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

void Lora_communication::send_command(uint8_t cmd)
{
    pending_command_ = cmd;
    pending_tx_dirty_ = true;
    flush_pending_frame();
}

/* ========== 定时器中断 ========== */
void Lora_communication::Tim_It_Process() {
    timer_tick_count++;
    if (timer_tick_count >= 20) {
        timer_tick_count = 0;
        if (pending_tx_dirty_) {
            flush_pending_frame();
        }
    }
}

/* ========== GPIO 中断处理 ========== */
void Lora_communication::EXTI_Prosess() {
    Comm_TxBufferToTxDMA(lora_tx_huart);
}

} // namespace communication