/**
 * @file Module_CrsfReceiver.h
 * @brief RadioMaster POCKET CRSF接收器驱动
 * @author Zhan Hong li
 * @details 硬件连接说明
 *
 * 硬件连接需要4根线：
 * POCKET JR接收机 接 STM32 UART
 * GND 接 GND, 5V 接 VCC, TX 接 RX, RX 接 TX
 * 
 * CubeMX配置需要3个步骤：
 * 1. UART: 波特率=420000, 8N1
 * 2. DMA: 开启USART_RX的Circular模式
 * 3. NVIC: 开启UART和DMA中断
 * 
 * 使用方法分3个步骤：
 * // 1. 在文件顶部（全局变量区，main函数外）定义实例
 * static CrsfReceiver radio(&huart1); // 根据实际使用的UART修改
 * // 2. 在main函数内初始化
 * // CrsfReceiver radio(&huart1); // 不要这样写
 * radio.process();                               // 处理接收到的数据
 * radio.getControlData(&ctrl);                   // 获取控制数据
 * 
 * 数据结构说明：RmPocketData_t包含
 * 摇杆: throttle(油门), steering(转向), auxiliary1/2(辅助通道)
 * 按钮: SWD/botton_click(点动按钮), SWA/SWB/SWC(三档开关)
 * 其他: scroll_wheel/l2/r1/r2/menu/enter
 * 安全: emergency_stop紧急停止标志（任何开关可触发）
 * 
 * 遥测数据：通过sendTelemetryData()发送回遥控器
 * 可在遥控器上显示/语音播报电池电压/电流/剩余电量
 * 距离、速度、温度等运行数据在遥控器屏幕显示
 * 
 * 示例1：发送电池数据（覆盖现有字段）
 *   telem.battery_current = motor_temperature; // 将电流字段用于发送温度
 *   radio->sendTelemetryData(&telem);
 * 
 * 示例2：发送自定义数据（需要修改结构体）
 *   // 1. 在RmPocketData_t结构体中添加字段
 *   float motor_temp;        // 电机温度
 *   uint16_t obstacle_dist;  // 障碍物距离（毫米）
 *   
 *   // 2. 修改sendTelemetryData()函数以包含新字段
 *   // 3. 在main循环中更新并发送
 *   telem.motor_temp = ReadTemperature();
 *   telem.obstacle_dist = sonar_read();
 *   radio->sendTelemetryData(&telem); // 发送遥测数据
 
 * radio->setStickDeadzone(0.05f);    // 设置摇杆死区
 * radio->setThrottleCurve(1.2f);     // 设置油门曲线
 * radio->isEmergencyStop();          // 检查紧急停止
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

// RadioMaster-POCKET 通道值范围定义
#define RM_POCKET_CHANNEL_MIN 172      // 最小值
#define RM_POCKET_CHANNEL_MID 992      // 中点值
#define RM_POCKET_CHANNEL_MAX 1811     // 最大值

// 按钮判定阈值（根据RadioMaster-POCKET实际输出校准）
#define RM_BTN_OFF 191                 // 按钮释放状态
#define RM_BTN_ON 1792                 // 按钮按下状态

// RadioMaster-POCKET 三档开关判定阈值（低/中/高）
#define RM_SWITCH_LOW 191              // 低档位置
#define RM_SWITCH_MID 1004             // 中档位置
#define RM_SWITCH_HIGH 1792            // 高档位置

#define crclen 256

// 遥控器数据结构体定义
typedef struct
{
    // === 摇杆数据：RadioMaster-POCKET输出值映射到浮点数 ===
    // 所有摇杆值已归一化到-1.0到1.0范围，中点为0
    float left_y;            //left_y       // 左摇杆Y轴/油门控制 (-1.0最小值 ~ 1.0最大值) - 通常用于前进后退控制
    float left_x;            //left_x       // 左摇杆X轴/转向控制 (-1.0左满 ~ 1.0右满) - 通常用于左右转向控制
    
    float right_x;                 // 右摇杆X轴/辅助通道1 - 通常用于云台水平旋转或其他功能
    float right_y;                 // 右摇杆Y轴/辅助通道2 - 通常用于云台俯仰或其他功能
    
    // RadioMaster-POCKET 点动按钮
    uint8_t SWD;                  // 右侧3档开关位置：0=低档, 1=中档, 2=高档（映射为点动按钮功能）
    uint8_t botton_click;                 // 左侧3档开关位置：0=低档, 1=中档, 2=高档（映射为点动按钮功能）
    uint8_t SWA;                    // SA开关状态：0=低档, 1=中档, 2=高档（三档开关）
    uint8_t SWB;                    // SB开关状态：0=低档, 1=中档, 2=高档（三档开关）
    uint8_t SWC;                    // SC开关状态：0=低档, 1=中档, 2=高档（三档开关）
    
    // 其他按钮：RadioMaster-POCKET面板上6个额外功能按钮
    uint8_t scroll_wheel;                   // L1滚轮/旋钮（通常用于菜单导航）
    uint8_t btn_l2;                   // L2按钮（通常用于功能切换）
    uint8_t btn_r1;                   // R1按钮（通常用于确认/选择）
    uint8_t btn_r2;                   // R2按钮（通常用于返回/取消）
    uint8_t btn_menu;                 // 菜单按钮
    uint8_t btn_enter;                // 确认/回车按钮
    
    // 安全控制标志
    uint8_t emergency_stop;           // 紧急停止标志（任何开关触发都会设置此标志）
    uint8_t trigger_flag;             // 触发标志（用于事件触发）
    
    // === 遥测数据：从机器人发回RadioMaster-POCKET显示的数据 ===
    // 这些数据通过sendTelemetryData()函数发送
    float battery_voltage;            // 电池电压(V)
    float battery_current;            // 电池电流(A)
    uint8_t battery_percent;          // 电池剩余百分比(%)
    uint32_t battery_capacity;        // 电池已消耗容量(mAh)
    
    // 运行数据
    float speed_kmh;                  // 当前速度(km/h)
    float distance_km;                // 累计运行距离(km)
    uint16_t run_time_minutes;        // 累计运行时间(分钟)
    
    // 系统状态
    float temperature;                // 系统温度(摄氏度)
    uint8_t signal_strength;          // 信号强度(%)
    
//    // GPS数据（如果需要可取消注释并配置GPS模块）
//    double gps_latitude;              // 纬度
//    double gps_longitude;             // 经度
//    float gps_speed;                  // GPS速度(km/h)
//    uint8_t gps_satellites;           // 卫星数量
//    
} RmPocketData_t;

// 标准CRSF通道数据帧结构（用于解析遥控器发送的通道数据）
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

// CRC8校验计算类
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

// RadioMaster-POCKET CRSF接收器主类
class CrsfReceiver:public UART_
{
public:
	
    void send_uint8(uint8_t sub_type, uint8_t value);
    void send_uint16(uint8_t sub_type, uint16_t value);
		void send_float(uint8_t sub_type, float value);
		void send_robot(float x, float y, float yaw);
		void send_kfsandSpear(int8_t kfs1, int8_t kfs2, int8_t Spear);
		void send_controlmode(float mode);
    static CrsfReceiver* GetInstance(UART_HandleTypeDef *huart);
    CrsfReceiver(const CrsfReceiver&) = delete;
    CrsfReceiver& operator=(const CrsfReceiver&) = delete;

    // ========== 数据获取接口 ==========
    
    // 获取RadioMaster-POCKET遥控器解析后的控制数据
    void getControlData(RmPocketData_t *data);
    void init(void){    this->UART_Init();}
    // 发送遥测数据回RadioMaster-POCKET遥控器显示
    void sendTelemetryData(const RmPocketData_t *data);
    
    // 检查是否有新数据到达
    bool hasNewData() const { return new_data_available_; }
    void clearNewDataFlag() { new_data_available_ = false; }
    
    // 紧急停止状态检查
    bool isEmergencyStop() const { return emergency_stop_triggered_; }
    void resetEmergencyStop() { emergency_stop_triggered_ = false; }
    
    // 获取原始通道值（用于调试或自定义映射）
    int getRawChannel(uint8_t ch) const;
    const int* getAllRawChannels() const { return channels_; }
    
    // ========== 参数配置接口 ==========
    
    // 设置摇杆死区（防止摇杆在中点附近的抖动）
    void setStickDeadzone(float deadzone) { stick_deadzone_ = deadzone; }
    
    // 设置油门曲线（大于1.0增加灵敏度，小于1.0降低灵敏度）
    void setThrottleCurve(float curve_factor) { throttle_curve_ = curve_factor; }
    
    // 设置转向曲线
    void setSteeringCurve(float curve_factor) { steering_curve_ = curve_factor; }
    
    // 设置遥测数据发送速率
    void setTelemetryRate(uint32_t battery_ms, uint32_t gps_ms) {
        telemetry_battery_interval_ = battery_ms;
        telemetry_gps_interval_ = gps_ms;
    }
    
    // 主处理函数：需要在主循环中持续调用以处理接收数据
    void process();
    // 测试功能：临时禁用/启用 D-Cache，用于调试验证
    void setDisableDCacheForTest(bool disable);
    bool isDCacheTestDisabled() const { return dcache_test_disabled_; }
    // 验证 UART/DMA 配置是否正确（检查 DMA 是否已启动）
    bool isDmaConfiguredCorrectly() const { return dma_config_ok_; }
		UART_HandleTypeDef *uart_handle;
		void Callback_Fuc(uint8_t *buf, uint16_t len) override;
private:

		
    // 通道数据存储
    int channels_[CRSF_NUM_CHANNELS];
    uint8_t channels_payload_[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE];
    uint8_t packet_byte_index_;
    uint8_t *payload_ptr_;
    volatile bool new_data_available_;
    bool emergency_stop_triggered_;
    
    // UART驱动
    // 可选: 将 DMA 缓冲区放到特定内存段 (如 D2) 以避免 D-Cache 问题。
    // 如果已定义 CRSF_DMA_SECTION_NAME 为字符串，例如: "\.dma_buffer"
#ifdef CRSF_DMA_SECTION_NAME
#define CRSF_DMA_ATTR __attribute__((section(CRSF_DMA_SECTION_NAME), aligned(32)))
#else
#define CRSF_DMA_ATTR __attribute__((aligned(32)))
#endif
    uint8_t rx_buffer_[256] CRSF_DMA_ATTR;
    //UART_ uart_driver_;
    uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE] CRSF_DMA_ATTR;

    // 环形缓冲区：用于缓存ISR接收的数据，在process()中处理
    static const uint16_t RX_RING_SIZE = 512;
    uint8_t rx_ring_[RX_RING_SIZE] CRSF_DMA_ATTR;
    volatile uint16_t rx_ring_head_ = 0; // 写入指针，由ISR更新
    volatile uint16_t rx_ring_tail_ = 0; // 读取指针，由process()更新
    
    // CRC校验器
    GENERIC_CRC8 crc_;
    
    // 参数配置
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
    
    // 内部处理方法
    void processBatchData(uint8_t *buf, uint16_t len);
    void handleByte(uint8_t byte);
    void processRcChannels();
    void unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS]);
    void computeMappedValues();
    void updateSwitchesAndButtons();
    // ISR-friendly append（在中断服务程序中快速缓存数据）
    void appendFromISR(const uint8_t *buf, uint16_t len);

    void consumeRingBuffer();
    // 内部配置检查
    void checkDmaConfig();
    
    // 单例模式
    static CrsfReceiver* instance_;
    static void StaticUartCallback(uint8_t *buf, uint16_t len);

    RmPocketData_t telemetry_data_;
    
    // 辅助变量
    uint8_t last_emergency_btn_ = 0;
    bool dma_config_ok_ = false;
    bool dcache_test_disabled_ = false;

public:
    // 调试用公共变量
    float debug_throttle = 0.0f;
    float debug_steering = 0.0f;
    uint8_t debug_mode = 0;
    
    // 触发标志控制（用于外部事件触发）
    void reset_trigger_flag() { }
    void set_trigger_flag_busy() { }
};
typedef struct {
    uint8_t kfs1 = 0;
    uint8_t kfs2 = 0;
    int8_t Spear = 0;  
    uint8_t mode = 0;
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
} TargetSet_t;
#endif // __cplusplus

#endif // Module_CRSF_RECEIVER_H