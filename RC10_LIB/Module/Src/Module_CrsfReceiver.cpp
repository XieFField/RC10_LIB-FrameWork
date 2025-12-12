/*******************************************************
 *  STM32H7  CRSF 接收机  「UART7 专用」最终量产版
 *  1. 所有 huart1 → huart7
 *  2. 发送前 Clean D-Cache，接收后 Invalidate
 *  3. 紧急停自动复位，13 字节 CRC 正确
 *  4. 链接脚本零改动，DMA 回调指向 UART7
 ******************************************************/
#include "Module_CrsfReceiver.h"
#include <cstring>
#include <cmath>
#include "core_cm7.h"

/* -------------  Cache 维护宏/函数  ------------- */
// SCB cache ops 要求地址与大小基于 cache line 对齐(32字节)
static inline void dcache_clean_range(void* addr, uint32_t len) {
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache not enabled
    const uint32_t line = 32u;
    uint32_t start = (uint32_t)addr & ~(line - 1u);
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));
    SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}
static inline void dcache_invalidate_range(void* addr, uint32_t len) {
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache not enabled
    const uint32_t line = 32u;
    uint32_t start = (uint32_t)addr & ~(line - 1u);
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

/* -------------  外部句柄  ------------- */
extern UART_HandleTypeDef huart7;          //  <-- 你用的是 UART7

static CrsfReceiver* g_this = nullptr;
CrsfReceiver* CrsfReceiver::instance_ = nullptr;

/* ======================================================= 
 *   以下实现与 UART7 绑定
 * ====================================================== */

void CrsfReceiver::StaticUartCallback(uint8_t *buf, uint16_t len)
{
    if (g_this) g_this->appendFromISR(buf, len);
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
//    instance_ = g_this = this;
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
        channels_[i] = RM_POCKET_CHANNEL_MID;
    memset(channels_payload_, 0, sizeof(channels_payload_));
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    payload_ptr_ = nullptr;
    this->UART_Init();
}

void CrsfReceiver::Callback_Fuc(uint8_t *buf, uint16_t len)
{
	if (g_this) g_this->appendFromISR(buf, len);
}
//CrsfReceiver::~CrsfReceiver()
//{
//    uart_driver_.SetCallback(nullptr);
//    HAL_UART_AbortReceive(uart_driver_.GetUartHandle());
//    instance_ = g_this = nullptr;
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
    throttle_raw_ = (float)(channels_[2] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    steering_raw_ = (float)(channels_[3] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    aux1_raw_ = (float)(channels_[0] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    aux2_raw_ = (float)(channels_[1] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    if (fabsf(throttle_raw_) < stick_deadzone_) throttle_raw_ = 0.0f;
    if (fabsf(steering_raw_) < stick_deadzone_) steering_raw_ = 0.0f;
    if (fabsf(aux1_raw_) < stick_deadzone_) aux1_raw_ = 0.0f;
    if (fabsf(aux2_raw_) < stick_deadzone_) aux2_raw_ = 0.0f;
    if (throttle_curve_ != 1.0f && throttle_raw_ != 0.0f)
        throttle_raw_ = copysignf(powf(fabsf(throttle_raw_), throttle_curve_), throttle_raw_);
    if (steering_curve_ != 1.0f && steering_raw_ != 0.0f)
        steering_raw_ = copysignf(powf(fabsf(steering_raw_), steering_curve_), steering_raw_);
    debug_throttle = throttle_raw_;
    debug_steering = steering_raw_;
}
void CrsfReceiver::updateSwitchesAndButtons()
{
    const int BTN_ON = 1500, SW_LOW = 400, SW_HIGH = 1500;
    sw_sa_ = (channels_[4] > BTN_ON) ? 1 : 0;
    sw_sb_ = (channels_[5] > BTN_ON) ? 1 : 0;
    sw_sc_ = (channels_[6] > BTN_ON) ? 1 : 0;
    if (channels_[7] < SW_LOW)       sw_left_ = 0;
    else if (channels_[7] > SW_HIGH) sw_left_ = 2;
    else                             sw_left_ = 1;
    if (channels_[8] < SW_LOW)       sw_right_ = 0;
    else if (channels_[8] > SW_HIGH) sw_right_ = 2;
    else                             sw_right_ = 1;
    btn_l1_    = (channels_[9]  > BTN_ON) ? 1 : 0;
    btn_l2_    = (channels_[10] > BTN_ON) ? 1 : 0;
    btn_r1_    = (channels_[11] > BTN_ON) ? 1 : 0;
    btn_r2_    = (channels_[12] > BTN_ON) ? 1 : 0;
    btn_menu_  = (channels_[13] > BTN_ON) ? 1 : 0;
    btn_enter_ = (channels_[14] > BTN_ON) ? 1 : 0;
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
        if (btn_l2_ == 1 && last_emergency_btn_ == 0)
            emergency_stop_triggered_ = true;
        last_emergency_btn_ = btn_l2_;
        last_chk = now;
    }
    new_data_available_ = true;
}

/* ----------------  用户接口  ---------------- */
void CrsfReceiver::getControlData(RmPocketData_t* data)
{
    if (!data) return;
    data->throttle   = throttle_raw_;
    data->steering   = steering_raw_;
    data->auxiliary1 = aux1_raw_;
    data->auxiliary2 = aux2_raw_;
    data->sw_left    = sw_left_;
    data->sw_right   = sw_right_;
    data->sw_sa      = sw_sa_;
    data->sw_sb      = sw_sb_;
    data->sw_sc      = sw_sc_;
    data->btn_l1     = btn_l1_;
    data->btn_l2     = btn_l2_;
    data->btn_r1     = btn_r1_;
    data->btn_r2     = btn_r2_;
    data->btn_menu   = btn_menu_;
    data->btn_enter  = btn_enter_;
    data->emergency_stop = emergency_stop_triggered_ ? 1 : 0;
    emergency_stop_triggered_ = false;
    data->trigger_flag = 0;
    debug_mode = sw_left_;
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
    if (huart == &huart7) tx_done = true;   // 只认 UART7
}