#include "Module_CrsfReceiver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

CrsfReceiver* CrsfReceiver::instance_ = nullptr;

// 静态UART回调函数
void CrsfReceiver::StaticUartCallback(uint8_t *buf, uint16_t len)
{
    if (instance_ != nullptr)
    {
        // 为避免在ISR/回调中做大量计算，改为快速拷贝到环形缓冲，由主循环消费
        instance_->appendFromISR(buf, len);
    }
}

// 构造函数
CrsfReceiver::CrsfReceiver(UART_HandleTypeDef *huart) : 
    packet_byte_index_(0),
    new_data_available_(false),
    emergency_stop_triggered_(false),
    rx_state_(STATE_WAIT_ADDR),
    crc_(CRSF_CRC_POLY),
    uart_driver_(256, rx_buffer_, huart)
{
    instance_ = this;
    
    // 初始化通道值为中间值
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        channels_[i] = RM_POCKET_CHANNEL_MID;
    }
    
    // 初始化UART驱动（静态成员）
    uart_driver_.SetCallback(StaticUartCallback);
    uart_driver_.UART_Init();
    
    // 设置默认目标地址为遥控器
    // RadioMaster-POCKET接收地址通常是0xEA（遥控器发射机）
}

CrsfReceiver::~CrsfReceiver()
{
    // 如果需要，可在此处停止UART/DMA（例如调用uart_driver_.UART_DeInit()）
    // 在析构前先禁用回调，防止在析构过程中被中断/DMA回调访问
    uart_driver_.SetCallback(nullptr);
    // 尝试中止正在进行的接收（如果底层HAL支持）
    UART_HandleTypeDef *hu = uart_driver_.GetUartHandle();
    if (hu != nullptr)
    {
        // 最小化副作用：尝试中止接收（若HAL提供）
        HAL_UART_AbortReceive(hu);
    }
    instance_ = nullptr;
}

// CRC8类实现（保持不变）
GENERIC_CRC8::GENERIC_CRC8(uint8_t poly) : crcpoly(poly)
{
    uint8_t crc;
    for (uint16_t i = 0; i < crclen; i++)
    {
        crc = i;
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? crcpoly : 0);
        }
        crc8tab[i] = crc & 0xFF;
    }
}

uint8_t GENERIC_CRC8::calc(const uint8_t data)
{
    return calc(&data, 1, 0);
}

uint8_t GENERIC_CRC8::calc(const uint8_t *data, uint16_t len, uint8_t crc)
{
    while (len--)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

// 处理批量数据
void CrsfReceiver::processBatchData(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        handleByte(buf[i]);
    }
}

// 快速将回调收到的数据拷贝到环形缓冲（用于ISR上下文）
void CrsfReceiver::appendFromISR(const uint8_t *buf, uint16_t len)
{
    // 计算可写入的最大字节数
    uint16_t head = rx_ring_head_;
    uint16_t tail = rx_ring_tail_;
    uint16_t free_space = (tail + RX_RING_SIZE - head - 1) % RX_RING_SIZE;
    if (free_space == 0) return; // 环形缓冲已满，丢弃

    uint16_t to_copy = (len <= free_space) ? len : free_space;

    // 先拷贝第一段
    uint16_t chunk = RX_RING_SIZE - head;
    if (chunk > to_copy) chunk = to_copy;
    if (chunk)
    {
        memcpy(&rx_ring_[head], buf, chunk);
        head = (head + chunk) % RX_RING_SIZE;
    }

    // 拷贝剩余（如果有）
    uint16_t rem = to_copy - chunk;
    if (rem)
    {
        memcpy(&rx_ring_[head], buf + chunk, rem);
        head = (head + rem) % RX_RING_SIZE;
    }

    // 更新写指针（尽量原子地更新单个volatile变量）
    rx_ring_head_ = head;
}

// 在主循环中调用以消费环形缓冲数据（会调用现有的processBatchData）
void CrsfReceiver::consumeRingBuffer()
{
    // 如果空则返回
    uint16_t head = rx_ring_head_;
    uint16_t tail = rx_ring_tail_;
    if (head == tail) return;

    // 处理连续段1：从tail到缓冲末尾或head
    uint16_t chunk = (head > tail) ? (head - tail) : (RX_RING_SIZE - tail);
    if (chunk)
    {
        processBatchData(&rx_ring_[tail], chunk);
        tail = (tail + chunk) % RX_RING_SIZE;
    }

    // 如果还有数据并且head wrapped到前面，再处理
    if (tail != head)
    {
        uint16_t chunk2 = (head > tail) ? (head - tail) : 0;
        if (chunk2)
        {
            processBatchData(&rx_ring_[tail], chunk2);
            tail = (tail + chunk2) % RX_RING_SIZE;
        }
    }

    rx_ring_tail_ = tail;
}

// 获取原始通道值
int CrsfReceiver::getRawChannel(uint8_t ch) const
{
    if (ch >= 1 && ch <= CRSF_NUM_CHANNELS)
    {
        return channels_[ch - 1];
    }
    return RM_POCKET_CHANNEL_MID;
}

// 处理单个字节（状态机）
void CrsfReceiver::handleByte(uint8_t byte)
{
    switch (rx_state_)
    {
    case STATE_WAIT_ADDR:
        // RadioMaster-POCKET发送的数据通常地址为0xC8（飞控地址）或广播地址
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_BROADCAST)
        {
            rx_state_ = STATE_WAIT_SIZE;
        }
        break;
        
    case STATE_WAIT_SIZE:
        if (byte == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + 2)) // 22+2
        {
            rx_state_ = STATE_WAIT_TYPE;
        }
        else
        {
            rx_state_ = STATE_WAIT_ADDR; // 长度不匹配，重置
        }
        break;
        
    case STATE_WAIT_TYPE:
        if (byte == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            packet_byte_index_ = 0;
            payload_ptr_ = channels_payload_;
            rx_state_ = STATE_WAIT_PAYLOAD;
        }
        else
        {
            rx_state_ = STATE_WAIT_ADDR; // 不是RC通道包，重置
        }
        break;
        
    case STATE_WAIT_PAYLOAD:
        payload_ptr_[packet_byte_index_++] = byte;
        if (packet_byte_index_ >= CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
        {
            rx_state_ = STATE_WAIT_CRC;
        }
        break;
        
    case STATE_WAIT_CRC:
        // 暂时忽略CRC校验（实际应用应该校验）
        rx_state_ = STATE_PACKET_COMPLETE;
        processRcChannels();
        rx_state_ = STATE_WAIT_ADDR;
        break;
        
    default:
        rx_state_ = STATE_WAIT_ADDR;
        break;
    }
}

// 处理RC通道数据
void CrsfReceiver::processRcChannels()
{
    // 解包通道
    unpackChannels(channels_payload_, channels_);
    
    // 计算映射值
    computeMappedValues();
    
    // 更新开关和按钮状态
    updateSwitchesAndButtons();
    
    // 检查紧急停止（通常绑定到某个按钮或开关）
    // 例如：如果SA开关快速拨动两次，触发紧急停止
    static uint32_t last_emergency_check = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_emergency_check > 50) // 每50ms检查一次
    {
        // 检查L2按钮是否按下（作为紧急停止）
        if (btn_l2_ == 1 && last_emergency_btn_ == 0)
        {
            emergency_stop_triggered_ = true;
        }
        last_emergency_btn_ = btn_l2_;
        last_emergency_check = now;
    }
    
    new_data_available_ = true;
}

// 解包通道数据
void CrsfReceiver::unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS])
{
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        uint32_t bitpos = i * 11u;
        uint32_t bytepos = bitpos / 8u;
        uint32_t shift = bitpos % 8u;
        
        uint32_t value = (uint32_t)payload[bytepos] |
                         ((uint32_t)payload[bytepos + 1] << 8) |
                         ((uint32_t)payload[bytepos + 2] << 16);
        
        value >>= shift;
        value &= 0x7FFu;
        channels[i] = (int)value;
    }
}

// 计算映射值（针对RadioMaster-POCKET优化）
void CrsfReceiver::computeMappedValues()
{
    // RadioMaster-POCKET通道映射建议：
    // CH1: 右摇杆水平
    // CH2: 右摇杆垂直
    // CH3: 左摇杆垂直（油门）
    // CH4: 左摇杆水平（转向）
    // CH5: SA开关
    // CH6: SB开关
    // CH7: SC开关
    // CH8: 左三段开关
    // CH9: 右三段开关
    // CH10: L1按钮
    // CH11: L2按钮
    // CH12: R1按钮
    // CH13: R2按钮
    // CH14: 菜单按钮
    // CH15: 确认按钮
    // CH16: 滚轮或其他
    
    // 左摇杆垂直 -> 油门（通道3）
    throttle_raw_ = (float)(channels_[2] - RM_POCKET_CHANNEL_MID) / 
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    
    // 左摇杆水平 -> 转向（通道4）
    steering_raw_ = (float)(channels_[3] - RM_POCKET_CHANNEL_MID) / 
                    (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    
    // 右摇杆水平 -> 辅助1（通道1）
    aux1_raw_ = (float)(channels_[0] - RM_POCKET_CHANNEL_MID) / 
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    
    // 右摇杆垂直 -> 辅助2（通道2）
    aux2_raw_ = (float)(channels_[1] - RM_POCKET_CHANNEL_MID) / 
                (float)(RM_POCKET_CHANNEL_MAX - RM_POCKET_CHANNEL_MID);
    
    // 应用死区
    if (fabsf(throttle_raw_) < stick_deadzone_) throttle_raw_ = 0.0f;
    if (fabsf(steering_raw_) < stick_deadzone_) steering_raw_ = 0.0f;
    if (fabsf(aux1_raw_) < stick_deadzone_) aux1_raw_ = 0.0f;
    if (fabsf(aux2_raw_) < stick_deadzone_) aux2_raw_ = 0.0f;
    
    // 应用曲线（可选）
    if (throttle_curve_ != 1.0f && throttle_raw_ != 0.0f)
    {
        throttle_raw_ = copysignf(powf(fabsf(throttle_raw_), throttle_curve_), throttle_raw_);
    }
    
    if (steering_curve_ != 1.0f && steering_raw_ != 0.0f)
    {
        steering_raw_ = copysignf(powf(fabsf(steering_raw_), steering_curve_), steering_raw_);
    }
    
    // 保存调试值
    debug_throttle = throttle_raw_;
    debug_steering = steering_raw_;
}

// 更新开关和按钮状态
void CrsfReceiver::updateSwitchesAndButtons()
{
    // 三段开关阈值
    const int SWITCH_LOW_THRESH = 400;
    const int SWITCH_HIGH_THRESH = 1500;
    
    // 按钮阈值
    const int BUTTON_OFF_THRESH = 500;
    const int BUTTON_ON_THRESH = 1500;
    
    // SA开关（通道5）- 2段
    sw_sa_ = (channels_[4] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // SB开关（通道6）- 2段
    sw_sb_ = (channels_[5] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // SC开关（通道7）- 2段
    sw_sc_ = (channels_[6] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // 左三段开关（通道8）
    if (channels_[7] < SWITCH_LOW_THRESH)
        sw_left_ = 0;
    else if (channels_[7] > SWITCH_HIGH_THRESH)
        sw_left_ = 2;
    else
        sw_left_ = 1;
    
    // 右三段开关（通道9）
    if (channels_[8] < SWITCH_LOW_THRESH)
        sw_right_ = 0;
    else if (channels_[8] > SWITCH_HIGH_THRESH)
        sw_right_ = 2;
    else
        sw_right_ = 1;
    
    // L1按钮（通道10）
    btn_l1_ = (channels_[9] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // L2按钮（通道11）
    btn_l2_ = (channels_[10] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // R1按钮（通道12）
    btn_r1_ = (channels_[11] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // R2按钮（通道13）
    btn_r2_ = (channels_[12] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // 菜单按钮（通道14）
    btn_menu_ = (channels_[13] > BUTTON_ON_THRESH) ? 1 : 0;
    
    // 确认按钮（通道15）
    btn_enter_ = (channels_[14] > BUTTON_ON_THRESH) ? 1 : 0;
}

// 获取控制数据（用户接口）
void CrsfReceiver::getControlData(RmPocketData_t *data)
{
    if (data == nullptr) return;
    
    // 摇杆控制
    data->throttle = throttle_raw_;      // 前进后退
    data->steering = steering_raw_;      // 左右转向
    data->auxiliary1 = aux1_raw_;        // 辅助控制1
    data->auxiliary2 = aux2_raw_;        // 辅助控制2
    
    // 开关状态
    data->sw_left = sw_left_;            // 左三段开关
    data->sw_right = sw_right_;          // 右三段开关
    data->sw_sa = sw_sa_;                // SA开关
    data->sw_sb = sw_sb_;                // SB开关
    data->sw_sc = sw_sc_;                // SC开关
    
    // 按钮状态
    data->btn_l1 = btn_l1_;
    data->btn_l2 = btn_l2_;
    data->btn_r1 = btn_r1_;
    data->btn_r2 = btn_r2_;
    data->btn_menu = btn_menu_;
    data->btn_enter = btn_enter_;
    
    // 紧急停止标志
    data->emergency_stop = emergency_stop_triggered_ ? 1 : 0;
    
    // 触发标志（保持向后兼容）
    data->trigger_flag = 0;
    
    // 调试模式
    debug_mode = sw_left_;  // 使用左三段开关作为调试模式选择
}

// 发送遥测数据到RadioMaster-POCKET
void CrsfReceiver::sendTelemetryData(const RmPocketData_t *data)
{
    if (data == nullptr) return;
    
    uint32_t now = HAL_GetTick();
    
    // 发送电池数据（定期发送）
    if (now - last_battery_send_ >= telemetry_battery_interval_)
    {
        // 电池电压（V * 100）
        uint16_t voltage_scaled = (uint16_t)(data->battery_voltage * 100.0f);
        // 电池电流（A * 100）
        uint16_t current_scaled = (uint16_t)(data->battery_current * 100.0f);
        
        tx_buffer_[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;  // 发送到遥控器
        tx_buffer_[1] = 10;  // 帧长度：8 + 2
        tx_buffer_[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
        
        // 电压
        tx_buffer_[3] = voltage_scaled & 0xFF;
        tx_buffer_[4] = (voltage_scaled >> 8) & 0xFF;
        
        // 电流
        tx_buffer_[5] = current_scaled & 0xFF;
        tx_buffer_[6] = (current_scaled >> 8) & 0xFF;
        
        // 容量（24位）
        tx_buffer_[7] = data->battery_capacity & 0xFF;
        tx_buffer_[8] = (data->battery_capacity >> 8) & 0xFF;
        tx_buffer_[9] = (data->battery_capacity >> 16) & 0xFF;
        
        // 剩余电量百分比
        tx_buffer_[10] = data->battery_percent;
        
        // CRC（简化）
        tx_buffer_[11] = 0;
        
        HAL_UART_Transmit_DMA(uart_driver_.GetUartHandle(), tx_buffer_, 12);
        
        last_battery_send_ = now;
    }
    
    // 发送GPS数据（如果可用，定期发送）
//    if (data->gps_satellites > 0 && now - last_gps_send_ >= telemetry_gps_interval_)
//    {
//        const int32_t lat_scaled = (int32_t)(data->gps_latitude * 1e7);
//        const int32_t lon_scaled = (int32_t)(data->gps_longitude * 1e7);
//        const uint16_t speed_scaled = (uint16_t)(data->gps_speed * 10.0f);  // km/h * 10
//        
//        tx_buffer_[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
//        tx_buffer_[1] = 17;  // 15 + 2
//        tx_buffer_[2] = CRSF_FRAMETYPE_GPS;
//        
//        // 纬度
//        tx_buffer_[3] = lat_scaled & 0xFF;
//        tx_buffer_[4] = (lat_scaled >> 8) & 0xFF;
//        tx_buffer_[5] = (lat_scaled >> 16) & 0xFF;
//        tx_buffer_[6] = (lat_scaled >> 24) & 0xFF;
//        
//        // 经度
//        tx_buffer_[7] = lon_scaled & 0xFF;
//        tx_buffer_[8] = (lon_scaled >> 8) & 0xFF;
//        tx_buffer_[9] = (lon_scaled >> 16) & 0xFF;
//        tx_buffer_[10] = (lon_scaled >> 24) & 0xFF;
//        
//        // 地速
//        tx_buffer_[11] = speed_scaled & 0xFF;
//        tx_buffer_[12] = (speed_scaled >> 8) & 0xFF;
//        
//        // 航向（使用小车航向或固定值）
//        uint16_t heading = 0;  // 0-359度
//        tx_buffer_[13] = heading & 0xFF;
//        tx_buffer_[14] = (heading >> 8) & 0xFF;
//        
//        // 海拔（默认0）
//        uint16_t altitude = 0;
//        tx_buffer_[15] = altitude & 0xFF;
//        tx_buffer_[16] = (altitude >> 8) & 0xFF;
//        
//        // 卫星数量
//        tx_buffer_[17] = data->gps_satellites;
//        
//        // CRC（简化）
//        tx_buffer_[18] = 0;
//        
//        HAL_UART_Transmit_DMA(uart_driver_.GetUartHandle(), tx_buffer_, 19);
//        
//        last_gps_send_ = now;
//    }
}

// 主处理函数（在main循环中调用）
void CrsfReceiver::process()
{
    // 如果有新数据，可以在这里处理
    if (new_data_available_)
    {
        // 可以添加额外处理逻辑
        new_data_available_ = false;
    }
    
        if (new_data_available_)
        {
            // 处理完毕后清标志（上层调用者可自行clearNewDataFlag）
            new_data_available_ = false;
        }

        // 先消费ISR写入的环形缓冲（将会执行解包/映射/CRC校验等）
        consumeRingBuffer();
}