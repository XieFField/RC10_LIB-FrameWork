/**
 * @file Module_CrsfReceiver.h
 * @brief  RadioMaster POCKET CRSF 接收机驱动头文件
 *          单例模式 STM32H7 uart7
 *          420 kBd  UART+DMA 循环接收，解析 16 通道、开关、按钮、遥测回发
 *         单例模式,对象是crsf_rc,直接用里面的数据就行
 *         使用例子:
 *									if (crsf_rc.hasNewData()) {              // 1 有新数据
 *									RmPocketData_t d;                    // 2 取出
 *									crsf_rc.getControlData(&d);						//3 赋值
 *									然后d里面的数据就是需要的数据 	
 *					
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

// RadioMaster-POCKET通道原始值范围定义
#define RM_POCKET_CHANNEL_MIN 172      // 通道最小值
#define RM_POCKET_CHANNEL_MID 992      // 通道中值（中心位置）
#define RM_POCKET_CHANNEL_MAX 1811     // 通道最大值

// 按钮状态阈值（RadioMaster-POCKET特定）
#define RM_BTN_OFF 191                 // 按钮释放状态
#define RM_BTN_ON 1792                 // 按钮按下状态

// RadioMaster-POCKET三段开关阈值（用于开关量映射）
#define RM_SWITCH_LOW 191              // 开关低位
#define RM_SWITCH_MID 1004             // 开关中位
#define RM_SWITCH_HIGH 1792            // 开关高位

#define crclen 256  // CRC表长度

// 遥控器数据处理后的归一化结构体（用于应用层）
typedef struct
{
    // === 控制输入部分（来自RadioMaster-POCKET遥控器）===
    // 摇杆和辅助通道值，已归一化到-1.0到1.0范围
    float left_y;                   // 油门/前进后退 (-1.0后退 ~ 1.0前进) - 左手摇杆上下
    float left_x;                   // 转向 (-1.0左转 ~ 1.0右转) - 左手摇杆左右
    
    float right_x;                 // 辅助通道1 - 右手摇杆上下或其他功能
    float right_y;                 // 辅助通道2 - 右手摇杆左右或其他功能
    
    // RadioMaster-POCKET开关状态（映射后）
    uint8_t sw_left;                  // 左侧3段开关 0=低, 1=中, 2=高（如：速度档位）
    uint8_t sw_right;                 // 右侧3段开关 0=低, 1=中, 2=高（如：工作模式）
    uint8_t sw_sa;                    // SA瞬时开关 0=关, 1=开（2段开关）
    uint8_t sw_sb;                    // SB瞬时开关 0=关, 1=开（2段开关）
    uint8_t sw_sc;                    // SC瞬时开关 0=关, 1=开（2段开关）
    
    // 按钮状态（RadioMaster-POCKET有6个自定义按钮）
    uint8_t btn_l1;                   // L1按钮（左上角）
    uint8_t btn_l2;                   // L2按钮（左下角扳机）
    uint8_t btn_r1;                   // R1按钮（右上角）
    uint8_t btn_r2;                   // R2按钮（右下角扳机）
    uint8_t btn_menu;                 // 菜单按钮
    uint8_t btn_enter;                // 确认按钮
    
    // 安全标志
    uint8_t emergency_stop;           // 紧急停止标志（L2长按触发）
    uint8_t trigger_flag;             // 触发标志（预留）
    
    // === 遥测数据部分（发送给RadioMaster-POCKET遥控器显示）===
    // 电池相关数据（遥控器OSD显示）
    float battery_voltage;            // 电池电压(V)
    float battery_current;            // 电池电流(A)
    uint8_t battery_percent;          // 电池百分比(%)
    uint32_t battery_capacity;        // 电池容量(mAh)
    
    // 里程/速度数据
    float speed_kmh;                  // 当前速度(km/h)
    float distance_km;                // 行驶距离(km)
    uint16_t run_time_minutes;        // 运行时间(分钟)
    
    // 系统状态数据
    float temperature;                // 温度(°C)
    uint8_t signal_strength;          // 信号强度(%)
    
//    // GPS相关数据（需要GPS模块）
//    double gps_latitude;              // 纬度
//    double gps_longitude;             // 经度
//    float gps_speed;                  // GPS速度(km/h)
//    uint8_t gps_satellites;           // 卫星数量
//    
} RmPocketData_t;

// CRSF协议帧结构体定义（使用紧凑打包）
typedef struct
{
    uint8_t device_addr;     // 设备地址
    uint8_t frame_size;      // 帧大小
    uint8_t type;            // 帧类型
    crsf_channels_t channels; // 通道数据
    uint8_t crc;             // CRC校验
} PACKED CrsfRcChannelsFrame_t;  // 遥控通道数据帧

typedef struct
{
    int16_t pitch;           // 俯仰角
    int16_t roll;            // 横滚角
    int16_t yaw;             // 偏航角
} PACKED CrsfAttitudePayload_t;  // 姿态数据载荷

typedef struct
{
    uint8_t device_addr;     // 设备地址
    uint8_t frame_size;      // 帧大小
    uint8_t type;            // 帧类型
    CrsfAttitudePayload_t payload; // 姿态数据
    uint8_t crc;             // CRC校验
} PACKED CrsfAttitudeFrame_t;  // 姿态数据帧

typedef struct
{
    uint16_t voltage;        // 电压
    uint16_t current;        // 电流
    uint8_t capacity[3];     // 容量（3字节）
    uint8_t remaining;       // 剩余百分比
} PACKED CrsfBatteryPayload_t;  // 电池数据载荷

typedef struct
{
    uint8_t device_addr;     // 设备地址
    uint8_t frame_size;      // 帧大小
    uint8_t type;            // 帧类型
    CrsfBatteryPayload_t payload; // 电池数据
    uint8_t crc;             // CRC校验
} PACKED CrsfBatteryFrame_t;  // 电池数据帧

// CRC8计算类（用于CRSF协议CRC校验）
class GENERIC_CRC8
{
private:
    uint8_t crc8tab[crclen]; // CRC8查找表
    uint8_t crcpoly;         // CRC多项式

public:
    // 构造函数：初始化CRC表
    GENERIC_CRC8(uint8_t poly);
    // 计算单个字节的CRC
    uint8_t calc(const uint8_t data);
    // 计算数据块的CRC
    uint8_t calc(const uint8_t *data, uint16_t len, uint8_t crc = 0);

};

// RadioMaster-POCKET CRSF接收机主类（继承UART基类）
class CrsfReceiver:public UART_
{
public:
    // 单例模式获取实例（全局唯一实例）
    static CrsfReceiver& instance(UART_HandleTypeDef* huart = nullptr);
    /* 禁止拷贝构造和赋值操作（单例模式） */
    CrsfReceiver(const CrsfReceiver&) = delete;
    CrsfReceiver& operator=(const CrsfReceiver&) = delete;
		static RmPocketData_t data;

		
    // ========== 主要公共接口 ==========
    
    // 获取处理后的遥控器数据（归一化到结构体）
    void getControlData(RmPocketData_t *data);
    // 初始化函数（调用UART基类初始化）
    void init(void){    this->UART_Init();}
    // 发送遥测数据到遥控器（CRSF协议格式）
    void sendTelemetryData(const RmPocketData_t *data);
    
    // 新数据标志管理
    bool hasNewData() const { return new_data_available_; }
    void clearNewDataFlag() { new_data_available_ = false; }
    
    // 紧急停止状态管理
    bool isEmergencyStop() const { return emergency_stop_triggered_; }
    void resetEmergencyStop() { emergency_stop_triggered_ = false; }
    
    // 获取原始通道数据（未经处理）
    int getRawChannel(uint8_t ch) const;
    const int* getAllRawChannels() const { return channels_; }
    
    // ========== 配置接口 ==========
    
    // 设置摇杆死区（减少摇杆中立点附近的漂移）
    void setStickDeadzone(float deadzone) { stick_deadzone_ = deadzone; }
    
    // 设置油门曲线（指数曲线，>1.0为凸曲线，<1.0为凹曲线）
    void setThrottleCurve(float curve_factor) { throttle_curve_ = curve_factor; }
    
    // 设置转向曲线
    void setSteeringCurve(float curve_factor) { steering_curve_ = curve_factor; }
    
    // 设置遥测发送频率
    void setTelemetryRate(uint32_t battery_ms, uint32_t gps_ms) {
        telemetry_battery_interval_ = battery_ms;
        telemetry_gps_interval_ = gps_ms;
    }
    
    // 主处理函数（必须在主循环中定期调用）
    void process();
    // 测试接口：临时禁用/启用D-Cache（用于调试Cache问题）
    void setDisableDCacheForTest(bool disable);
    bool isDCacheTestDisabled() const { return dcache_test_disabled_; }
    // 验证UART/DMA配置是否正确（检查是否为循环DMA接收）
    bool isDmaConfiguredCorrectly() const { return dma_config_ok_; }
    
    // UART句柄和回调函数
    UART_HandleTypeDef *uart_handle;
    void Callback_Fuc(uint8_t *buf, uint16_t len) override;
    
private:
    // 私有构造函数（单例模式）
    explicit CrsfReceiver(UART_HandleTypeDef* huart);
    
    // 数据存储成员
    int channels_[CRSF_NUM_CHANNELS];                     // 原始通道值存储
    uint8_t channels_payload_[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]; // 通道数据载荷
    uint8_t packet_byte_index_;                           // 数据包字节索引
    uint8_t *payload_ptr_;                                // 当前载荷指针
    volatile bool new_data_available_;                    // 新数据可用标志（volatile）
    bool emergency_stop_triggered_;                       // 紧急停止触发标志
    
    // UART DMA缓冲区（使用特殊内存区域以避免Cache一致性问题）
    // 可选：将DMA缓冲放到非缓存内存区域（如D2域）以解决D-Cache问题
    // 在项目中使用CRSF_DMA_SECTION_NAME定义内存段名，如："\.dma_buffer"
#ifdef CRSF_DMA_SECTION_NAME
#define CRSF_DMA_ATTR __attribute__((section(CRSF_DMA_SECTION_NAME), aligned(32)))
#else
#define CRSF_DMA_ATTR __attribute__((aligned(32)))
#endif
    uint8_t rx_buffer_[256] CRSF_DMA_ATTR;  // DMA接收缓冲区（32字节对齐）
    uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE] CRSF_DMA_ATTR;  // DMA发送缓冲区

    // 环形缓冲区（用于在ISR和主循环之间传递数据）
    // ISR将数据写入环形缓冲区，主循环process()从中读取并处理
    static const uint16_t RX_RING_SIZE = 512;  // 环形缓冲区大小
    uint8_t rx_ring_[RX_RING_SIZE] CRSF_DMA_ATTR;  // 环形缓冲区
    volatile uint16_t rx_ring_head_ = 0; // 缓冲区头指针（ISR写入位置）
    volatile uint16_t rx_ring_tail_ = 0; // 缓冲区尾指针（主循环读取位置）
    
    // CRC计算对象
    GENERIC_CRC8 crc_;
    
    // 用户可调参数
    float stick_deadzone_ = 0.05f;           // 摇杆死区（5%）
    float throttle_curve_ = 1.0f;            // 油门曲线因子（1.0=线性）
    float steering_curve_ = 1.0f;            // 转向曲线因子（1.0=线性）
    
    // 遥测发送定时
    uint32_t telemetry_battery_interval_ = 1000;  // 电池数据发送间隔(ms)
    uint32_t telemetry_gps_interval_ = 2000;      // GPS数据发送间隔(ms)
    uint32_t last_battery_send_ = 0;              // 上次电池数据发送时间
    uint32_t last_gps_send_ = 0;                  // 上次GPS数据发送时间
    
    // 接收状态机状态定义
    enum RxState {
        STATE_WAIT_ADDR,        // 等待设备地址
        STATE_WAIT_SIZE,        // 等待帧大小
        STATE_WAIT_TYPE,        // 等待帧类型
        STATE_WAIT_PAYLOAD,     // 等待数据载荷
        STATE_WAIT_CRC,         // 等待CRC校验
        STATE_PACKET_COMPLETE   // 数据包完整接收
    } rx_state_;
    
    // 私有处理函数
    void processBatchData(uint8_t *buf, uint16_t len);    // 批量处理数据
    void handleByte(uint8_t byte);                        // 单个字节处理（状态机）
    void processRcChannels();                            // 处理遥控通道数据
    void unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS]); // 解包通道数据
    void computeMappedValues();                          // 计算映射值（归一化）
    void updateSwitchesAndButtons();                     // 更新开关和按钮状态
    // ISR友好型数据追加（无锁，用于中断服务程序）
    void appendFromISR(const uint8_t *buf, uint16_t len);
    // 消费环形缓冲区数据（主循环调用）
    void consumeRingBuffer();
    // 内部DMA配置检查
    void checkDmaConfig();
    
    // 单例模式静态成员
    static CrsfReceiver* instance_;
    static void StaticUartCallback(uint8_t *buf, uint16_t len);
    
    // 处理后的归一化数据（中间变量）
    float left_y = 0.0f;  // 原始油门值（-1.0~1.0）
    float left_x = 0.0f;  // 原始转向值（-1.0~1.0）
    float right_x = 0.0f;      // 辅助通道1原始值
    float right_y = 0.0f;      // 辅助通道2原始值
    
    // 开关和按钮状态（中间变量）
    uint8_t sw_left_ = 1;      // 左开关状态（默认中位）
    uint8_t sw_right_ = 1;     // 右开关状态（默认中位）
    uint8_t sw_sa_ = 0;        // SA开关状态（默认关）
    uint8_t sw_sb_ = 0;        // SB开关状态（默认关）
    uint8_t sw_sc_ = 0;        // SC开关状态（默认关）
    uint8_t btn_l1_ = 0;       // L1按钮状态
    uint8_t btn_l2_ = 0;       // L2按钮状态
    uint8_t btn_r1_ = 0;       // R1按钮状态
    uint8_t btn_r2_ = 0;       // R2按钮状态
    uint8_t btn_menu_ = 0;     // 菜单按钮状态
    uint8_t btn_enter_ = 0;    // 确认按钮状态
    
    // 内部状态标志
    uint8_t last_emergency_btn_ = 0;  // 上次紧急按钮状态（用于边沿检测）
    bool dma_config_ok_ = false;      // DMA配置正确标志
    bool dcache_test_disabled_ = false; // D-Cache禁用测试标志

public:
    // 调试变量（可外部访问用于调试）
    float debug_throttle = 0.0f;  // 调试油门值
    float debug_steering = 0.0f;  // 调试转向值
    uint8_t debug_mode = 0;       // 调试模式
    
    // 预留接口（当前为空实现）
    void reset_trigger_flag() { }
    void set_trigger_flag_busy() { }
};

#endif // __cplusplus

#endif // Module_CRSF_RECEIVER_H