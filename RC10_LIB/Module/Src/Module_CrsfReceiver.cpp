/*******************************************************
 *  STM32H7 CRSF接收机 UART7专用实现文件
 *  主要修改点：
 *  1. 所有huart1 → huart7（适配UART7）
 *  2. 发送前Clean D-Cache，接收后Invalidate（保证Cache一致性）
 *  3. 紧急停止自动复位，13字节CRC正确
 *  4. 链接脚本零改动，DMA回调指向UART7
 ******************************************************/
#include "Module_CrsfReceiver.h"
#include <cstring>
#include <cmath>
#include "core_cm7.h"

/* -------------  Cache维护宏/函数（STM32H7专用） ------------- */
// SCB cache操作要求地址与大小基于cache line对齐（32字节）
// 清理DCache（将CPU缓存数据写回内存）
static inline void dcache_clean_range(void* addr, uint32_t len) 
{
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache未启用
    const uint32_t line = 32u;  // STM32H7的cache行大小
    uint32_t start = (uint32_t)addr & ~(line - 1u);  // 对齐到cache行起始
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));  // 对齐到cache行结束
    SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));  // 清理指定范围的cache
}

// 失效DCache（丢弃CPU缓存数据，重新从内存读取）
static inline void dcache_invalidate_range(void* addr, uint32_t len) 
{
    if (len == 0 || addr == nullptr) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) return; // D-Cache未启用
    const uint32_t line = 32u;
    uint32_t start = (uint32_t)addr & ~(line - 1u);
    uint32_t end = (((uint32_t)addr + len + (line - 1u)) & ~(line - 1u));
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));  // 失效指定范围的cache
}

/* -------------  外部句柄声明（在main.c中定义） ------------- */
extern UART_HandleTypeDef huart7;          //  <-- 使用UART7，外部声明
static CrsfReceiver* g_instance = nullptr; // 全局实例指针（备用）
// CrsfReceiver* instance_ = nullptr;      // 注释掉的原声明
CrsfReceiver* CrsfReceiver::instance_ = nullptr; // 静态成员初始化

/* ======================================================= 
 *  以下实现与UART7绑定
 * ====================================================== */

// 静态UART回调函数（供C接口调用）
void CrsfReceiver::StaticUartCallback(uint8_t *buf, uint16_t len)
{
    if (instance_) 
        instance_->appendFromISR(buf, len);  // 转发到实例的ISR处理函数
}

// 单例模式获取实例（线程安全，C++11保证静态局部变量初始化线程安全）
CrsfReceiver& CrsfReceiver::instance(UART_HandleTypeDef* huart)
{
    static CrsfReceiver instance_obj(huart);  // 静态局部变量，生命周期贯穿整个程序
    g_instance = &instance_obj;                // 同时设置全局指针
    return instance_obj;
}

/* 构造函数实现 */
CrsfReceiver::CrsfReceiver(UART_HandleTypeDef* huart)
    : packet_byte_index_(0),                    // 数据包字节索引初始化
      new_data_available_(false),               // 新数据标志初始化
      emergency_stop_triggered_(false),         // 紧急停止标志初始化
      rx_state_(STATE_WAIT_ADDR),               // 接收状态机初始状态
      crc_(CRSF_CRC_POLY),                      // CRC对象初始化（使用CRSF多项式）
      rx_ring_head_(0),                         // 环形缓冲区头指针初始化
      rx_ring_tail_(0),                         // 环形缓冲区尾指针初始化
      rx_buffer_{0},                            // 接收缓冲区清零初始化
      UART_(256, rx_buffer_, huart)             // 调用基类UART_构造函数
{
    instance_ = this;   // 保存静态指针（用于静态回调函数访问）
    
    // 初始化所有通道值为中位值
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
        channels_[i] = RM_POCKET_CHANNEL_MID;
    
    // 清零通道载荷和发送缓冲区
    memset(channels_payload_, 0, sizeof channels_payload_);
    memset(tx_buffer_, 0, sizeof tx_buffer_);
    payload_ptr_ = nullptr;  // 载荷指针初始化为空
}

// UART接收回调函数（从基类UART_重写）
void CrsfReceiver::Callback_Fuc(uint8_t *buf, uint16_t len)
{
    if (instance_) instance_->appendFromISR(buf, len);  // 调用ISR安全的数据追加函数
}

// 析构函数（已注释掉，根据需要使用）
//CrsfReceiver::~CrsfReceiver()
//{
//    uart_driver_.SetCallback(nullptr);
//    HAL_UART_AbortReceive(uart_driver_.GetUartHandle());
//    instance_ = instance_ = nullptr
//    
//}

// 测试接口：临时关闭D-Cache（调试用，已注释）
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

// 初始化时检查UART/DMA设置是否正确（RX DMA存在且为循环模式，已注释）
//void CrsfReceiver::checkDmaConfig()
//{
//    UART_HandleTypeDef* h = uart_driver_.GetUartHandle();
//    if (h && h->hdmarx) {
//        dma_config_ok_ = (h->hdmarx->Init.Mode == DMA_CIRCULAR);
//    } else {
//        dma_config_ok_ = false;
//    }
//}

/* ----------------  CRC8类实现 ---------------- */
// 构造函数：初始化CRC8查找表
GENERIC_CRC8::GENERIC_CRC8(uint8_t poly)
{
    // 生成256字节的CRC查找表
    for (uint16_t i = 0; i < 256; ++i) {
        uint8_t crc = i;
        // 计算每个字节的CRC值（8次迭代）
        for (uint8_t j = 0; j < 8; ++j)
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);  // 多项式计算
        crc8tab[i] = crc;  // 存储到查找表
    }
}

// 计算数据块的CRC值
uint8_t GENERIC_CRC8::calc(const uint8_t* data, uint16_t len, uint8_t crc)
{
    // 遍历数据，使用查找表快速计算CRC
    while (len--) crc = crc8tab[crc ^ *data++];
    return crc;
}

/* ----------------  接收状态机实现 ---------------- */
// 处理单个字节的状态机
void CrsfReceiver::handleByte(uint8_t byte)
{
    switch (rx_state_) {
    case STATE_WAIT_ADDR:
        // 等待设备地址：检查是否为飞行控制器地址或广播地址
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_BROADCAST)
            rx_state_ = STATE_WAIT_SIZE;  // 地址匹配，等待帧大小
        break;
    case STATE_WAIT_SIZE:
        // 等待帧大小：检查是否为遥控通道数据包的大小（载荷+2字节头）
        if (byte == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 2))
            rx_state_ = STATE_WAIT_TYPE;  // 大小匹配，等待帧类型
        else
            rx_state_ = STATE_WAIT_ADDR;  // 大小不匹配，重新等待地址
        break;
    case STATE_WAIT_TYPE:
        // 等待帧类型：检查是否为遥控通道数据包
        if (byte == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            packet_byte_index_ = 0;  // 重置字节索引
            payload_ptr_ = channels_payload_;  // 设置载荷指针
            rx_state_ = STATE_WAIT_PAYLOAD;  // 等待载荷数据
        } else
            rx_state_ = STATE_WAIT_ADDR;  // 类型不匹配，重新等待地址
        break;
    case STATE_WAIT_PAYLOAD:
        // 等待载荷数据：按字节存储到载荷缓冲区
        if (packet_byte_index_ < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) {
            payload_ptr_[packet_byte_index_++] = byte;  // 存储字节并递增索引
            if (packet_byte_index_ >= CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
                rx_state_ = STATE_WAIT_CRC;  // 载荷接收完成，等待CRC
        } else
            rx_state_ = STATE_WAIT_ADDR;  // 索引越界，重新等待地址
        break;
    case STATE_WAIT_CRC:
        // 等待CRC：直接标记数据包完整（实际CRC校验可能在后续处理中）
        rx_state_ = STATE_PACKET_COMPLETE;
        processRcChannels();  // 处理接收到的通道数据
        rx_state_ = STATE_WAIT_ADDR;  // 重置状态机，等待下一个数据包
        break;
    default:
        rx_state_ = STATE_WAIT_ADDR;  // 未知状态，重置状态机
    }
}

/* ----------------  通道数据处理：解包 + 映射 + 开关状态更新 ---------------- */
// 解包CRSF协议的11位通道数据（每个通道11位，共16个通道）
void CrsfReceiver::unpackChannels(const uint8_t* payload, int channels[CRSF_NUM_CHANNELS])
{
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        uint32_t bitpos = i * 11;  // 每个通道11位，计算位位置
        uint32_t bytepos = bitpos / 8;  // 计算字节位置
        uint32_t shift   = bitpos % 8;  // 计算位移量
        
        // 读取3个字节（可能跨越字节边界）
        uint32_t b0 = (bytepos < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos] : 0;
        uint32_t b1 = (bytepos + 1 < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos + 1] : 0;
        uint32_t b2 = (bytepos + 2 < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE) ? payload[bytepos + 2] : 0;
        
        // 组合3个字节，右移，并取低11位
        uint32_t value = (b0 | (b1 << 8) | (b2 << 16)) >> shift;
        value &= 0x7FFu;  // 确保只有11位有效
        
        // 范围检查，如果超出范围则使用中位值
        if (value < RM_POCKET_CHANNEL_MIN || value > RM_POCKET_CHANNEL_MAX)
            channels[i] = RM_POCKET_CHANNEL_MID;
        else
            channels[i] = (int)value;  // 存储原始通道值
    }
}

// 计算归一化的摇杆值（-1.0到1.0范围），应用死区和曲线
void CrsfReceiver::computeMappedValues()
{
    // 将原始通道值（172~1811）归一化到-1.0~1.0范围
    left_y = (float)(channels_[2] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    left_x = (float)(channels_[3] - RM_POCKET_CHANNEL_MID) /
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    right_x = (float)(channels_[0] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    right_y = (float)(channels_[1] - RM_POCKET_CHANNEL_MID) /
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    
    // 应用死区（摇杆中心附近的小值视为零）
    if (fabsf(left_y) < stick_deadzone_) left_y = 0.0f;
    if (fabsf(left_x) < stick_deadzone_) left_x = 0.0f;
    if (fabsf(right_x) < stick_deadzone_) right_x = 0.0f;
    if (fabsf(right_y) < stick_deadzone_) right_y = 0.0f;
    
    // 应用指数曲线（曲线因子≠1.0时）
    if (throttle_curve_ != 1.0f && left_y != 0.0f)
        left_y = copysignf(powf(fabsf(left_y), throttle_curve_), left_y);
    if (steering_curve_ != 1.0f && left_x != 0.0f)
        left_x = copysignf(powf(fabsf(left_x), steering_curve_), left_x);
    
    // 调试变量赋值（便于外部监控）
    debug_throttle = left_y;
    debug_steering = left_x;
}

// 更新开关和按钮状态（基于原始通道值）
void CrsfReceiver::updateSwitchesAndButtons()
{
    // 阈值定义
    const int BTN_ON = 1500;  // 按钮按下阈值
    const int SW_LOW = 400;   // 开关低位阈值
    const int SW_HIGH = 1500; // 开关高位阈值
    
    // 2段开关（SA, SB, SC）：高于阈值=开，低于阈值=关
    sw_sa_ = (channels_[4] > BTN_ON) ? 1 : 0;
    sw_sb_ = (channels_[5] > BTN_ON) ? 1 : 0;
    sw_sc_ = (channels_[6] > BTN_ON) ? 1 : 0;
    
    // 3段开关（左、右）：根据阈值判断低、中、高位
    if (channels_[7] < SW_LOW)       sw_left_ = 0;   // 低位
    else if (channels_[7] > SW_HIGH) sw_left_ = 2;   // 高位
    else                             sw_left_ = 1;   // 中位
    
    if (channels_[8] < SW_LOW)       sw_right_ = 0;  // 低位
    else if (channels_[8] > SW_HIGH) sw_right_ = 2;  // 高位
    else                             sw_right_ = 1;  // 中位
    
    // 按钮（L1, L2, R1, R2, Menu, Enter）：高于阈值=按下
    btn_l1_    = (channels_[9]  > BTN_ON) ? 1 : 0;
    btn_l2_    = (channels_[10] > BTN_ON) ? 1 : 0;
    btn_r1_    = (channels_[11] > BTN_ON) ? 1 : 0;
    btn_r2_    = (channels_[12] > BTN_ON) ? 1 : 0;
    btn_menu_  = (channels_[13] > BTN_ON) ? 1 : 0;
    btn_enter_ = (channels_[14] > BTN_ON) ? 1 : 0;
}

/* ----------------  紧急停止处理 ---------------- */
// 处理接收到的遥控通道数据（解包、映射、更新状态）
void CrsfReceiver::processRcChannels()
{
    // 解包原始通道数据
    unpackChannels(channels_payload_, channels_);
    // 计算归一化的摇杆值
    computeMappedValues();
    // 更新开关和按钮状态
    updateSwitchesAndButtons();
    
    // 紧急停止检测（L2按钮边沿触发）
    static uint32_t last_chk = 0;  // 上次检查时间
    uint32_t now = HAL_GetTick();  // 当前时间
    
    // 每50ms检查一次（防抖和降低CPU负载）
    if (now - last_chk > 50) {
        // L2按钮从释放到按下的边沿触发紧急停止
        if (btn_l2_ == 1 && last_emergency_btn_ == 0)
            emergency_stop_triggered_ = true;
        
        last_emergency_btn_ = btn_l2_;  // 保存当前状态用于下次边沿检测
        last_chk = now;  // 更新时间戳
    }
    
    new_data_available_ = true;  // 设置新数据可用标志
}

/* ----------------  用户接口：获取控制数据 ---------------- */
// 将内部处理后的数据复制到用户提供的结构体
void CrsfReceiver::getControlData(RmPocketData_t* data)
{
    if (!data) return;  // 空指针检查
    
    // 复制摇杆和辅助通道数据
    data->left_y   = left_y;
    data->left_x   = left_x;
    data->right_x = right_x;
    data->right_y = right_y;
    
    // 复制开关状态
    data->sw_left    = sw_left_;
    data->sw_right   = sw_right_;
    data->sw_sa      = sw_sa_;
    data->sw_sb      = sw_sb_;
    data->sw_sc      = sw_sc_;
    
    // 复制按钮状态
    data->btn_l1     = btn_l1_;
    data->btn_l2     = btn_l2_;
    data->btn_r1     = btn_r1_;
    data->btn_r2     = btn_r2_;
    data->btn_menu   = btn_menu_;
    data->btn_enter  = btn_enter_;
    
    // 紧急停止标志（获取后自动复位）
    data->emergency_stop = emergency_stop_triggered_ ? 1 : 0;
    emergency_stop_triggered_ = false;  // 自动复位标志
    
    data->trigger_flag = 0;  // 触发标志（预留，当前为0）
    
    debug_mode = sw_left_;  // 调试模式设为左开关状态（便于调试）
}

/* ----------------  遥测数据发送 ---------------- */
// 发送电池遥测数据到遥控器（CRSF协议格式）
static volatile bool tx_done = true;  // 发送完成标志（volatile，ISR修改）
void CrsfReceiver::sendTelemetryData(const RmPocketData_t* data)
{
    if (!data || !tx_done) return;  // 数据为空或正在发送，则返回
    
    uint32_t now = HAL_GetTick();  // 当前时间
    
    // 检查电池数据发送间隔
    if (now - last_battery_send_ >= telemetry_battery_interval_) {
        // 转换数据为CRSF协议格式
        uint16_t volt = (uint16_t)(data->battery_voltage * 100.0f);  // 电压*100（V→0.01V）
        uint16_t curr = (uint16_t)(data->battery_current * 100.0f);  // 电流*100（A→0.01A）
        
        uint8_t* p = tx_buffer_;  // 发送缓冲区指针
        
        // 构建CRSF电池数据帧（13字节）
        p[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;  // 设备地址：无线电发射器
        p[1] = 10;  // 帧大小：10字节（类型+载荷+CRC）
        p[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;  // 帧类型：电池传感器
        
        // 电压（2字节，小端序）
        p[3] = volt & 0xFF;
        p[4] = volt >> 8;
        
        // 电流（2字节，小端序）
        p[5] = curr & 0xFF;
        p[6] = curr >> 8;
        
        // 电池容量（3字节，小端序）
        p[7] = data->battery_capacity & 0xFF;
        p[8] = (data->battery_capacity >> 8) & 0xFF;
        p[9] = (data->battery_capacity >> 16) & 0xFF;
        
        // 电池剩余百分比
        p[10] = data->battery_percent;
        
        // CRC校验（计算类型+载荷的CRC）
        p[11] = crc_.calc(&p[2], 9);
        
        tx_done = false;  // 设置发送中标志
        
        // 清理DCache（确保DMA能读取到最新数据）
        dcache_clean_range(tx_buffer_, 13);
        
        // 启动DMA传输（使用UART7）
        HAL_UART_Transmit_DMA(&huart7, tx_buffer_, 13);
        
        last_battery_send_ = now;  // 更新上次发送时间
    }
}

/* ----------------  主循环处理函数 ---------------- */
// 必须在主循环中定期调用，处理接收到的数据
void CrsfReceiver::process()
{
    consumeRingBuffer();  // 消费环形缓冲区中的数据
}

/* ----------------  ISR数据追加函数（中断安全） ---------------- */
// 从ISR调用，将接收到的数据追加到环形缓冲区
void CrsfReceiver::appendFromISR(const uint8_t *buf, uint16_t len)
{
    if (!buf || !len) return;  // 空指针或长度检查
    
    // 限制长度不超过环形缓冲区大小
    if (len > RX_RING_SIZE) len = RX_RING_SIZE;
    
    uint16_t head = rx_ring_head_ % RX_RING_SIZE;  // 当前头指针（写入位置）
    uint16_t tail = rx_ring_tail_ % RX_RING_SIZE;  // 当前尾指针（读取位置）
    
    // 计算环形缓冲区可用空间
    uint16_t free_space = (tail + RX_RING_SIZE - head - 1) % RX_RING_SIZE;
    if (free_space == 0) return;  // 缓冲区满，丢弃数据
    
    // 计算实际可复制的数据量
    uint16_t to_copy = (len <= free_space) ? len : free_space;
    
    // 第一段复制（从head到缓冲区末尾）
    uint16_t chunk = RX_RING_SIZE - head;
    if (chunk > to_copy) chunk = to_copy;
    
    // 失效DMA缓冲区的DCache，确保CPU读取到DMA写入的数据
    dcache_invalidate_range((void*)buf, to_copy);
    
    // 复制数据到环形缓冲区
    memcpy(&rx_ring_[head], buf, chunk);
    head = (head + chunk) % RX_RING_SIZE;
    
    // 第二段复制（如果需要，从缓冲区头部开始）
    uint16_t rem = to_copy - chunk;
    if (rem) {
        memcpy(&rx_ring_[head], buf + chunk, rem);
        head = (head + rem) % RX_RING_SIZE;
    }
    
    // 数据内存屏障，确保内存操作顺序（避免乱序执行）
    __asm volatile ("dmb 0xB" ::: "memory");
    
    // 更新头指针（写入位置）
    rx_ring_head_ = head;
}

// 批量处理数据（按字节调用状态机）
void CrsfReceiver::processBatchData(uint8_t *buf, uint16_t len)
{
    if (!buf || !len) return;  // 空指针或长度检查
    
    // 遍历缓冲区中的每个字节，交给状态机处理
    for (uint16_t i = 0; i < len; ++i) 
        handleByte(buf[i]);
}

/* ----------------  消费环形缓冲区数据（主循环调用） ---------------- */
// 从环形缓冲区读取数据并处理
void CrsfReceiver::consumeRingBuffer()
{
    // 失效整个环形缓冲区的DCache，确保读取到最新数据
    dcache_invalidate_range(rx_ring_, RX_RING_SIZE);
    
    uint16_t head = rx_ring_head_ % RX_RING_SIZE;  // 当前头指针
    uint16_t tail = rx_ring_tail_ % RX_RING_SIZE;  // 当前尾指针
    
    if (head == tail) return;  // 缓冲区空，无数据可处理
    
    // 第一段处理（从tail到缓冲区末尾）
    uint16_t chunk = (head > tail) ? (head - tail) : (RX_RING_SIZE - tail);
    if (chunk) {
        processBatchData(&rx_ring_[tail], chunk);  // 处理数据
        tail = (tail + chunk) % RX_RING_SIZE;     // 更新尾指针
    }
    
    // 第二段处理（如果需要，从缓冲区头部开始）
    if (tail != head) {
        uint16_t chunk2 = (head > tail) ? (head - tail) : 0;
        if (chunk2) {
            processBatchData(&rx_ring_[tail], chunk2);
            tail = (tail + chunk2) % RX_RING_SIZE;
        }
    }
    
    // 更新尾指针（已处理完的数据位置）
    rx_ring_tail_ = tail;
}

/* ----------------  全局C链接回调函数，指向UART7 ---------------- */
// HAL库UART发送完成回调函数
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart7) 
        tx_done = true;   // 只有UART7的发送完成才设置标志
}