#include "Module_CrsfReceiver.h"
#include <cstring>
#include <cmath>
#include "core_cm7.h"

/* -------------  Cache 维护宏/函数  ------------- */
// SCB cache ops 要求地址与大小基于 cache line 对齐(32字节)
static inline void dcache_clean_range(void* addr, uint32_t len) 
{
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache not enabled
    const uint32_t line = 32u;
    uint32_t start = (uint32_t)addr & ~(line - 1u);
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));
    SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

static inline void dcache_invalidate_range(void* addr, uint32_t len) 
{
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache not enabled
    const uint32_t line = 32u;
    uint32_t start = (uint32_t)addr & ~(line - 1u);
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

/* -------------  外部句柄  ------------- */
extern UART_HandleTypeDef huart7;          

// CrsfReceiver* instance_ = nullptr;
CrsfReceiver* CrsfReceiver::instance_ = nullptr;

/* ======================================================= 
 *   以下实现与 UART7 绑定
 * ====================================================== */

void CrsfReceiver::StaticUartCallback(uint8_t *buf, uint16_t len)
{
    if (instance_) 
		instance_->appendFromISR(buf, len);
}

/* 构造 / 析构 */
// Module_CrsfReceiver.cpp 中的构造函数

CrsfReceiver::CrsfReceiver(UART_HandleTypeDef* huart)
    : packet_byte_index_(0),
      new_data_available_(1),
      emergency_stop_triggered_(1),
      rx_state_(STATE_WAIT_ADDR),
      crc_(CRSF_CRC_POLY),
      rx_ring_head_(0),
      rx_ring_tail_(0),
      rx_buffer_{0},  // ← 先清零缓冲区
      UART_(256,rx_buffer_,huart)
{
    instance_ = this;
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
        channels_[i] = RM_POCKET_CHANNEL_MID;

    memset(channels_payload_, 0, sizeof(channels_payload_));
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    payload_ptr_ = nullptr;

}

CrsfReceiver* CrsfReceiver::GetInstance(UART_HandleTypeDef *huart)
{
    // if (instance_ == nullptr) {
    //     instance_ = new CrsfReceiver(huart);
    // }
    // return instance_;

    static CrsfReceiver instance(huart);
    return &instance;
}


void CrsfReceiver::Callback_Fuc(uint8_t *buf, uint16_t len)
{
	if (instance_) 
            instance_->appendFromISR(buf, len);
}
//CrsfReceiver::~CrsfReceiver()
//{
//    uart_driver_.SetCallback(nullptr);
//    HAL_UART_AbortReceive(uart_driver_.GetUartHandle());
//    instance_ = instance_ = nullptr
	
//}

// 测试接口：临时关闭 D-Cache（调试用）
//void CrsfReceiver::setDisableDCacheForTest(bool disable)
//{
//    if (disable) {
//        if (!dcache_test_disabled_) {
//            SCB_DisableDCache();
//            dcache_test_disabled_ = true;
//        }
//    } else {
//        if (dcache_test_disabled_) {
//            SCB_EnableDCache();
//            dcache_test_disabled_ = false;
//        }
//    }
//}

// 初始化时检查 UART/DMA 设置是否正确（RX DMA 存在且为循环模式）
//void CrsfReceiver::checkDmaConfig()
//{
//    UART_HandleTypeDef* h = uart_driver_.GetUartHandle();
//    if (h && h->hdmarx) {
//        dma_config_ok_ = (h->hdmarx->Init.Mode == DMA_CIRCULAR);
//    } else {
//        dma_config_ok_ = false;
//    }
//}

/* ----------------  CRC8  ---------------- */
GENERIC_CRC8::GENERIC_CRC8(uint8_t poly)
{
    for (uint16_t i = 0; i < 256; ++i) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; ++j)
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        crc8tab[i] = crc;
    }
}
uint8_t GENERIC_CRC8::calc(const uint8_t* data, uint16_t len, uint8_t crc)
{
    while (len--) crc = crc8tab[crc ^ *data++];
    return crc;
}

/* ----------------  状态机  ---------------- */
void CrsfReceiver::handleByte(uint8_t byte)
{
    switch (rx_state_) {
    case STATE_WAIT_ADDR:
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_BROADCAST)
            rx_state_ = STATE_WAIT_SIZE;
        break;
    case STATE_WAIT_SIZE:
        if (byte == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 2))
            rx_state_ = STATE_WAIT_TYPE;
        else
            rx_state_ = STATE_WAIT_ADDR;
        break;
    case STATE_WAIT_TYPE:
        if (byte == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            packet_byte_index_ = 0;
            payload_ptr_ = channels_payload_;
            rx_state_ = STATE_WAIT_PAYLOAD;
        } else
            rx_state_ = STATE_WAIT_ADDR;
        break;
    case STATE_WAIT_PAYLOAD:
        if (packet_byte_index_ < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) {
            payload_ptr_[packet_byte_index_++] = byte;
            if (packet_byte_index_ >= CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
                rx_state_ = STATE_WAIT_CRC;
        } else
            rx_state_ = STATE_WAIT_ADDR;
        break;
    case STATE_WAIT_CRC:
        rx_state_ = STATE_PACKET_COMPLETE;
        processRcChannels();
        rx_state_ = STATE_WAIT_ADDR;
        break;
    default:
        rx_state_ = STATE_WAIT_ADDR;
    }
}

/* ----------------  解包 + 映射 + 开关  ---------------- */
void CrsfReceiver::unpackChannels(const uint8_t* payload, int channels[CRSF_NUM_CHANNELS])
{
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        uint32_t bitpos = i * 11;
        uint32_t bytepos = bitpos / 8;
        uint32_t shift   = bitpos % 8;
        uint32_t b0 = (bytepos < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos] : 0;
        uint32_t b1 = (bytepos + 1 < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos + 1] : 0;
        uint32_t b2 = (bytepos + 2 < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos + 2] : 0;
        uint32_t value = (b0 | (b1 << 8) | (b2 << 16)) >> shift;
        value &= 0x7FFu;
        if (value < RM_POCKET_CHANNEL_MIN || value > RM_POCKET_CHANNEL_MAX)
            channels[i] = RM_POCKET_CHANNEL_MID;
        else
            channels[i] = (int)value;
    }
}
void CrsfReceiver::computeMappedValues()
{
   telemetry_data_.left_y = (float)(channels_[2] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    telemetry_data_.left_x = (float)(channels_[3] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    telemetry_data_.right_x = (float)(channels_[0] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    telemetry_data_.right_y = (float)(channels_[1] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    if (fabsf(telemetry_data_.left_y) < stick_deadzone_) telemetry_data_.left_y = 0.0f;
    if (fabsf(telemetry_data_.left_x) < stick_deadzone_) telemetry_data_.left_x = 0.0f;
    if (fabsf(telemetry_data_.right_x) < stick_deadzone_)   telemetry_data_.right_x = 0.0f;
    if (fabsf(telemetry_data_.right_y) < stick_deadzone_) telemetry_data_.right_y = 0.0f;
    if (throttle_curve_ != 1.0f && telemetry_data_.left_y != 0.0f)
        telemetry_data_.left_y = copysignf(powf(fabsf(telemetry_data_.left_y), throttle_curve_), telemetry_data_.left_y);
    if (steering_curve_ != 1.0f && telemetry_data_.left_x != 0.0f)
        telemetry_data_.left_x = copysignf(powf(fabsf(telemetry_data_.left_x), steering_curve_), telemetry_data_.left_x);
    debug_throttle = telemetry_data_.left_y;
    debug_steering = telemetry_data_.left_x;
}
void CrsfReceiver::updateSwitchesAndButtons()
{
    const int BTN_ON = 1500, SW_LOW = 400, SW_HIGH = 1500;
    telemetry_data_.SWA = (channels_[4] > BTN_ON) ? 1 : 0;
   telemetry_data_.SWB = (channels_[5] <= 191) ? 0 : ((channels_[5] >= 1792) ? 2 : 1);
		telemetry_data_.SWC = (channels_[6] <= 191) ? 0 : ((channels_[6] >= 1792) ? 2 : 1);
    if (channels_[7] < SW_LOW)       telemetry_data_.SWD = 0;
    else if (channels_[7] > SW_HIGH) telemetry_data_.SWD = 1;
    else                             telemetry_data_.SWD = 1;
    if (channels_[8] < SW_LOW)       telemetry_data_.botton_click = 0;
    else if (channels_[8] > SW_HIGH) telemetry_data_.botton_click = 1;
    else                             telemetry_data_.botton_click = 1;
    telemetry_data_.scroll_wheel    = (channels_[9]  > BTN_ON) ? 1 : 0;
    telemetry_data_.btn_l2    = (channels_[10] > BTN_ON) ? 1 : 0;
    telemetry_data_.btn_r1    = (channels_[11] > BTN_ON) ? 1 : 0;
    telemetry_data_.btn_r2    = (channels_[12] > BTN_ON) ? 1 : 0;
    telemetry_data_.btn_menu  = (channels_[13] > BTN_ON) ? 1 : 0;
    telemetry_data_.btn_enter = (channels_[14] > BTN_ON) ? 1 : 0;
}

/* ----------------  紧急停止  ---------------- */
void CrsfReceiver::processRcChannels()
{
    unpackChannels(channels_payload_, channels_);
    computeMappedValues();
    updateSwitchesAndButtons();
    static uint32_t last_chk = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_chk > 50) {
        if (telemetry_data_.btn_l2 == 1 && last_emergency_btn_ == 0)
            emergency_stop_triggered_ = true;
        last_emergency_btn_ = telemetry_data_.btn_l2;
        last_chk = now;
    }
    new_data_available_ = true;
}

/* ----------------  用户接口  ---------------- */
void CrsfReceiver::getControlData(RmPocketData_t* data)
{
    if (!data) 
        return;


    data->left_y       = telemetry_data_.left_y;
    data->left_x       = telemetry_data_.left_x;



    data->right_x      = telemetry_data_.right_y;
    data->right_y      = telemetry_data_.right_x;
    data->SWD      = telemetry_data_.SWD;
    data->botton_click     = telemetry_data_.botton_click;
    data->SWA        = telemetry_data_.SWA;
    data->SWB        = telemetry_data_.SWB;
    data->SWC        = telemetry_data_.SWC;

    data->scroll_wheel       = telemetry_data_.scroll_wheel;
    data->btn_l2       = telemetry_data_.btn_l2;

    data->btn_r1       = telemetry_data_.btn_r1;
    data->btn_r2       = telemetry_data_.btn_r2;

    data->btn_menu     = telemetry_data_.btn_menu;
    data->btn_enter    = telemetry_data_.btn_enter;

    data->emergency_stop = emergency_stop_triggered_ ? 1 : 0;
    emergency_stop_triggered_ = false;
    data->trigger_flag = 0;
    debug_mode = telemetry_data_.SWD;
}

/* ----------------  遥测发送  ---------------- */
static volatile bool tx_done = true;
void CrsfReceiver::sendTelemetryData(const RmPocketData_t* data)
{
    if (!data || !tx_done) return;
    uint32_t now = HAL_GetTick();
    if (now - last_battery_send_ >= telemetry_battery_interval_) {
        uint16_t volt = (uint16_t)(data->battery_voltage * 100.0f);
        uint16_t curr = (uint16_t)(data->battery_current * 100.0f);
        uint8_t* p = tx_buffer_;
        p[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
        p[1] = 10;
        p[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
        p[3] = volt & 0xFF;
        p[4] = volt >> 8;
        p[5] = curr & 0xFF;
        p[6] = curr >> 8;
        p[7] = data->battery_capacity & 0xFF;
        p[8] = (data->battery_capacity >> 8) & 0xFF;
        p[9] = (data->battery_capacity >> 16) & 0xFF;
        p[10] = data->battery_percent;
        p[11] = crc_.calc(&p[2], 9);     // CRC
        tx_done = false;
        dcache_clean_range(tx_buffer_, 13);              // ← 刷 Cache (对齐)
        HAL_UART_Transmit_DMA(&huart7, tx_buffer_, 13); // UART7！
        last_battery_send_ = now;
    }
}
void CrsfReceiver::send_uint8(uint8_t sub_type, uint8_t value)
{
    if (!tx_done) return;  // 等待上次发送完成
    
    uint8_t* p = tx_buffer_;
    
    // 帧结构：[地址][长度][类型][子类型][数据][CRC]
    p[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;  // 0xEA
    p[1] = 4;  // 长度 = payload(2) + 2 = 4 (payload=subtype(1)+data(1))
    p[2] = CRSF_FRAMETYPE_CUSTOM_TELEMETRY; // 0x0C
    p[3] = sub_type;    // 子类型（0x00-0xFF，对应Lua的0C00-0CFF）
    p[4] = value;       // 数据字节
    
    // CRC：从类型字段(p[2])开始，包含类型(1)+子类型(1)+数据(1) = 3字节
    p[5] = crc_.calc(&p[2], 3);
    
    tx_done = false;
    dcache_clean_range(tx_buffer_, 6);  // 6字节总长度
    HAL_UART_Transmit_DMA(&huart7, tx_buffer_, 6);
}

void CrsfReceiver::send_uint16(uint8_t sub_type, uint16_t value)
{
    if (!tx_done) return;
    
    uint8_t* p = tx_buffer_;
    
    // 帧结构：[地址][长度][类型][子类型][低字节][高字节][CRC]
    p[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;  // 0xEA
    p[1] = 5;  // 长度 = payload(3) + 2 = 5 (payload=subtype(1)+data(2))
    p[2] = CRSF_FRAMETYPE_CUSTOM_TELEMETRY; // 0x0C
    p[3] = sub_type;                    // 子类型
    p[4] = value & 0xFF;                // 低字节（小端序）
    p[5] = (value >> 8) & 0xFF;         // 高字节
    
    // CRC：从类型字段开始，包含类型(1)+子类型(1)+数据(2) = 4字节
    p[6] = crc_.calc(&p[2], 4);
    
    tx_done = false;
    dcache_clean_range(tx_buffer_, 7);  // 7字节总长度
    HAL_UART_Transmit_DMA(&huart7, tx_buffer_, 7);
}

void CrsfReceiver::send_robot(uint16_t x, uint16_t y, uint16_t yaw)
{
    send_uint16(0x10, x);      // 子类型 0x10 = X坐标

    send_uint16(0x11, y);      // 子类型 0x11 = Y坐标
    
    send_uint16(0x12, yaw);    // 子类型 0x12 = Yaw角度
}
void CrsfReceiver::send_kfs(uint8_t x, uint8_t y)
{
    
    send_uint8(0x20, x);  // 子类型 0x20 =x坐标
	
    send_uint8(0x21, y);  // 子类型 0x21 =y坐标
}
void CrsfReceiver::send_Spear(uint8_t data)
{
    send_uint8(0x30, data);   // 子类型 0x30 = 矛杆数据
}
void CrsfReceiver::send_controlmode(uint8_t mode)
{
   
    send_uint8(0x40, mode);   // 子类型 0x40 = 控制模式
}
/* ----------------  主循环  ---------------- */
void CrsfReceiver::process()
{
    consumeRingBuffer();
}

/* ----------------  ISR 拷贝  ---------------- */
void CrsfReceiver::appendFromISR(const uint8_t *buf, uint16_t len)
{
    if (!buf || !len) return;
    if (len > RX_RING_SIZE) len = RX_RING_SIZE;
    uint16_t head = rx_ring_head_ % RX_RING_SIZE;
    uint16_t tail = rx_ring_tail_ % RX_RING_SIZE;
    uint16_t free_space = (tail + RX_RING_SIZE - head - 1) % RX_RING_SIZE;
    if (free_space == 0) return;
    uint16_t to_copy = (len <= free_space) ? len : free_space;
    uint16_t chunk = RX_RING_SIZE - head;
    if (chunk > to_copy) chunk = to_copy;
    // 确保 CPU 读取 buf 前失效 DCache，这样 memcpy 能拿到 DMA 写入的数据
    dcache_invalidate_range((void*)buf, to_copy);
    memcpy(&rx_ring_[head], buf, chunk);
    head = (head + chunk) % RX_RING_SIZE;
    uint16_t rem = to_copy - chunk;
    if (rem) {
        memcpy(&rx_ring_[head], buf + chunk, rem);
        head = (head + rem) % RX_RING_SIZE;
    }
    // 在更新 head 前确保数据内存屏障，避免乱序
    __asm volatile ("dmb 0xB" ::: "memory");
    rx_ring_head_ = head;
}
void CrsfReceiver::processBatchData(uint8_t *buf, uint16_t len)
{
    if (!buf || !len) return;
    for (uint16_t i = 0; i < len; ++i) handleByte(buf[i]);
}

/* ----------------  消费环形缓冲  ---------------- */
void CrsfReceiver::consumeRingBuffer()
{
    dcache_invalidate_range(rx_ring_, RX_RING_SIZE);   // ← 扔旧 Cache (对齐)
    uint16_t head = rx_ring_head_ % RX_RING_SIZE;
    uint16_t tail = rx_ring_tail_ % RX_RING_SIZE;
    if (head == tail) return;
    uint16_t chunk = (head > tail) ? (head - tail) : (RX_RING_SIZE - tail);
    if (chunk) {
        processBatchData(&rx_ring_[tail], chunk);
        tail = (tail + chunk) % RX_RING_SIZE;
    }
    if (tail != head) {
        uint16_t chunk2 = (head > tail) ? (head - tail) : 0;
        if (chunk2) {
            processBatchData(&rx_ring_[tail], chunk2);
            tail = (tail + chunk2) % RX_RING_SIZE;
        }
    }
    rx_ring_tail_ = tail;
}

/* ----------------  全局 C 链接，指向 UART7  ---------------- */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart7) tx_done = true;  
}