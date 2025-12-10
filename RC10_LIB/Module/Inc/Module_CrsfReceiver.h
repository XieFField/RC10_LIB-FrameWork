/**
 * @file Module_CrsfReceiver.h
 * @brief RadioMaster POCKET CRSF接收机
 *
 *硬件连接（4根线）
 * POCKET JR插槽 → STM32 UART
 * GND → GND, 5V → VCC, TX → RX, RX → TX
 * 
 * CubeMX配置（3步）
 * 1. UART: 波特率=420000, 8N1
 * 2. DMA: 开启USART_RX（Circular模式）
 * 3. NVIC: 使能UART和DMA中断
 * 
 * 代码集成（3行）
 * CrsfReceiver *radio = new CrsfReceiver(&huart1); // 初始化
 * radio->process();                               // 主循环调用
 * radio->getControlData(&ctrl);                   // 获取数据
 * 
 * 控制数据结构（RmPocketData_t）
 * 摇杆: throttle(前进), steering(转向), auxiliary1/2(备用)
 * 开关: sw_left/sw_right(三段), sw_sa/sw_sb/sw_sc(两段)
 * 按钮: btn_l1/l2/r1/r2/menu/enter
 * 安全: emergency_stop（触发时停车）
 * 
 * 如何修改遥测数据（核心）
 * 默认发送电池电压/电流/百分比。
 * 想发送其他数据（如温度/距离）：
 * 
 * 方法1：复用字段（最快）
 *   telem.battery_current = motor_temperature; // 电流字段改成温度
 *   radio->sendTelemetryData(&telem);
 * 
 * 方法2：添加新字段（推荐）
 *   // 1. 在RmPocketData_t结构体添加：
 *   float motor_temp;        // 电机温度
 *   uint16_t obstacle_dist;  // 障碍物距离
 *   
 *   // 2. 在sendTelemetryData()函数中添加发送逻辑
 *   // 3. main中填充数据：
 *   telem.motor_temp = ReadTemperature();
 *   telem.obstacle_dist = sonar_read();
 *   radio->sendTelemetryData(&telem); // 自动发送
 
 
 * 常用函数
 * radio->setStickDeadzone(0.05f);    // 设置死区
 * radio->setThrottleCurve(1.2f);     // 设置曲线
 * radio->isEmergencyStop();          // 检查急停
 * radio->getRawChannel(1);           // 获取原始通道值
 */


#ifndef Module_CRSF_RECEIVER_H
#define Module_CRSF_RECEIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "BSP_USB_UART_Driver.h"
#include "Module_crsf_protocol_defines.h"
#include <string.h>
#include "usart.h"	
#include <stdint.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// RadioMaster-POCKET 典型通道值
#define RM_POCKET_CHANNEL_MIN 172      // 最小值
#define RM_POCKET_CHANNEL_MID 992      // 中间值
#define RM_POCKET_CHANNEL_MAX 1811     // 最大值

// 按钮阈值（根据RadioMaster-POCKET调整）
#define RM_BTN_OFF 191                 // 按钮释放
#define RM_BTN_ON 1792                 // 按钮按下

// RadioMaster-POCKET 典型开关设置（三段开关）
#define RM_SWITCH_LOW 191              // 下位置
#define RM_SWITCH_MID 1004             // 中位置
#define RM_SWITCH_HIGH 1792            // 上位置

#define crclen 256

// 遥控小车专用数据结构
typedef struct
{
    // === 输入：从RadioMaster-POCKET接收的控制数据 ===
    // 摇杆数据（映射到-1.0到1.0范围）
    float throttle;                   // 油门/前进后退 (-1.0后退 ~ 1.0前进) - 建议：左摇杆垂直
    float steering;                   // 转向 (-1.0左转 ~ 1.0右转) - 建议：左摇杆水平
    
    float auxiliary1;                 // 辅助控制1 - 建议：右摇杆水平（云台左右）
    float auxiliary2;                 // 辅助控制2 - 建议：右摇杆垂直（云台上下）
    
    // RadioMaster-POCKET 开关状态
    uint8_t sw_left;                  // 左3段开关：0=下, 1=中, 2=上（通常用于模式选择）
    uint8_t sw_right;                 // 右3段开关：0=下, 1=中, 2=上（通常用于速度档位）
    uint8_t sw_sa;                    // SA开关：0=下, 1=上（2段开关）
    uint8_t sw_sb;                    // SB开关：0=下, 1=上（2段开关）
    uint8_t sw_sc;                    // SC开关：0=下, 1=上（2段开关）
    
    // 按钮状态（RadioMaster-POCKET通常有6个可编程按钮）
    uint8_t btn_l1;                   // L1按钮（左上方）
    uint8_t btn_l2;                   // L2按钮（左下方）
    uint8_t btn_r1;                   // R1按钮（右上方）
    uint8_t btn_r2;                   // R2按钮（右下方）
    uint8_t btn_menu;                 // 菜单按钮
    uint8_t btn_enter;                // 确认按钮
    
    // 特殊功能
    uint8_t emergency_stop;           // 紧急停止标志（通常绑定到某个按钮）
    uint8_t trigger_flag;             // 触发标志
    
    // === 输出：发送到RadioMaster-POCKET显示的数据 ===
    // 电池信息（显示在遥控器屏幕）
    float battery_voltage;            // 电池电压(V)
    float battery_current;            // 电池电流(A)
    uint8_t battery_percent;          // 剩余电量(%)
    uint32_t battery_capacity;        // 电池容量(mAh)
    
    // 小车状态
    float speed_kmh;                  // 当前速度(km/h)
    float distance_km;                // 行驶距离(km)
    uint16_t run_time_minutes;        // 运行时间(分钟)
    
    // 传感器数据
    float temperature;                // 温度(°C)
    uint8_t signal_strength;          // 信号强度(%)
    
    // GPS数据（如果小车有GPS）
    double gps_latitude;              // 纬度
    double gps_longitude;             // 经度
    float gps_speed;                  // GPS速度(km/h)
    uint8_t gps_satellites;           // 卫星数量
    
} RmPocketData_t;

// 原始CRSF数据结构（保持与协议一致）
typedef struct
{
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    crsf_channels_t channels;
    uint8_t crc;
} PACKED CrsfRcChannelsFrame_t;

typedef struct
{
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
} PACKED CrsfAttitudePayload_t;

typedef struct
{
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    CrsfAttitudePayload_t payload;
    uint8_t crc;
} PACKED CrsfAttitudeFrame_t;

typedef struct
{
    uint16_t voltage;
    uint16_t current;
    uint8_t capacity[3];
    uint8_t remaining;
} PACKED CrsfBatteryPayload_t;

typedef struct
{
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    CrsfBatteryPayload_t payload;
    uint8_t crc;
} PACKED CrsfBatteryFrame_t;

// CRC8计算类
class GENERIC_CRC8
{
private:
    uint8_t crc8tab[crclen];
    uint8_t crcpoly;

public:
    GENERIC_CRC8(uint8_t poly);
    uint8_t calc(const uint8_t data);
    uint8_t calc(const uint8_t *data, uint16_t len, uint8_t crc = 0);
};

// RadioMaster-POCKET CRSF接收器类
class CrsfReceiver
{
public:
    // 构造函数
    CrsfReceiver(UART_HandleTypeDef *huart);
    ~CrsfReceiver();
    
    // ========== 主要控制接口 ==========
    
    // 获取RadioMaster-POCKET控制数据（简化接口）
    void getControlData(RmPocketData_t *data);
    
    // 发送遥测数据到RadioMaster-POCKET（简化接口）
    void sendTelemetryData(const RmPocketData_t *data);
    
    // 检查是否有新控制数据
    bool hasNewData() const { return new_data_available_; }
    void clearNewDataFlag() { new_data_available_ = false; }
    
    // 紧急停止相关
    bool isEmergencyStop() const { return emergency_stop_triggered_; }
    void resetEmergencyStop() { emergency_stop_triggered_ = false; }
    
    // 获取原始通道值（高级用户）
    int getRawChannel(uint8_t ch) const;
    const int* getAllRawChannels() const { return channels_; }
    
    // ========== 配置接口 ==========
    
    // 设置摇杆死区（防止摇杆轻微抖动）
    void setStickDeadzone(float deadzone) { stick_deadzone_ = deadzone; }
    
    // 设置油门曲线（使响应更线性或更灵敏）
    void setThrottleCurve(float curve_factor) { throttle_curve_ = curve_factor; }
    
    // 设置转向曲线
    void setSteeringCurve(float curve_factor) { steering_curve_ = curve_factor; }
    
    // 设置发送频率
    void setTelemetryRate(uint32_t battery_ms, uint32_t gps_ms) {
        telemetry_battery_interval_ = battery_ms;
        telemetry_gps_interval_ = gps_ms;
    }
    
    // 运行一次处理（在主循环中调用）
    void process();

private:
    // 内部数据
    int channels_[CRSF_NUM_CHANNELS];
    uint8_t channels_payload_[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE];
    uint8_t packet_byte_index_;
    uint8_t *payload_ptr_;
    bool new_data_available_;
    bool emergency_stop_triggered_;
    
    // UART相关
    UART_* uart_driver_;
    uint8_t rx_buffer_[256];
    uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE];
    
    // CRC
    GENERIC_CRC8 crc_;
    
    // 配置参数
    float stick_deadzone_ = 0.05f;           // 5%死区
    float throttle_curve_ = 1.0f;            // 1.0=线性
    float steering_curve_ = 1.0f;            // 1.0=线性
    
    // 遥测发送定时
    uint32_t telemetry_battery_interval_ = 1000;  // 电池数据发送间隔(ms)
    uint32_t telemetry_gps_interval_ = 2000;      // GPS数据发送间隔(ms)
    uint32_t last_battery_send_ = 0;
    uint32_t last_gps_send_ = 0;
    
    // 接收状态机
    enum RxState {
        STATE_WAIT_ADDR,
        STATE_WAIT_SIZE,
        STATE_WAIT_TYPE,
        STATE_WAIT_PAYLOAD,
        STATE_WAIT_CRC,
        STATE_PACKET_COMPLETE
    } rx_state_;
    
    // 内部处理函数
    void processBatchData(uint8_t *buf, uint16_t len);
    void handleByte(uint8_t byte);
    void processRcChannels();
    void unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS]);
    void computeMappedValues();
    void updateSwitchesAndButtons();
    
    // 静态回调
    static CrsfReceiver* instance_;
    static void StaticUartCallback(uint8_t *buf, uint16_t len);
    
    // 映射值
    float throttle_raw_ = 0.0f;
    float steering_raw_ = 0.0f;
    float aux1_raw_ = 0.0f;
    float aux2_raw_ = 0.0f;
    
    // 开关/按钮状态
    uint8_t sw_left_ = 1;      // 默认为中间位置
    uint8_t sw_right_ = 1;
    uint8_t sw_sa_ = 0;
    uint8_t sw_sb_ = 0;
    uint8_t sw_sc_ = 0;
    uint8_t btn_l1_ = 0;
    uint8_t btn_l2_ = 0;
    uint8_t btn_r1_ = 0;
    uint8_t btn_r2_ = 0;
    uint8_t btn_menu_ = 0;
    uint8_t btn_enter_ = 0;
    
    // 内部标志
    uint8_t last_emergency_btn_ = 0;

public:
    // 调试信息（可选）
    float debug_throttle = 0.0f;
    float debug_steering = 0.0f;
    uint8_t debug_mode = 0;
    
    // 触发标志操作（向后兼容）
    void reset_trigger_flag() { }
    void set_trigger_flag_busy() { }
};

#endif // __cplusplus

#endif // Module_CRSF_RECEIVER_H