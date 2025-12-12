/**
 * @file Module_CrsfReceiver.h
 * @brief RadioMaster POCKET CRSF锟斤拷锟秸伙�?
 *
 *�?锟斤拷锟斤拷锟接ｏ�??4锟斤拷锟�?ｏ�??
 * POCKET JR锟斤拷锟�? 锟斤�? STM32 UART
 * GND 锟斤�? GND, 5V 锟斤�? VCC, TX 锟斤�? RX, RX 锟斤�? TX
 * 
 * CubeMX锟斤拷锟�?ｏ�??3锟斤拷锟斤拷
 * 1. UART: 锟斤拷锟斤拷锟斤�?=420000, 8N1
 * 2. DMA: 锟斤拷锟斤拷USART_RX锟斤拷Circular模式锟斤�?
 * 3. NVIC: 使锟斤拷UART锟斤拷DMA锟叫讹拷
 * 
 * 锟斤拷锟�?集锟缴ｏ�?3锟叫ｏ拷
 * // 锟狡硷拷锟斤拷嵌锟斤拷式锟斤拷锟斤拷锟斤拷使锟矫撅拷态锟斤拷锟皆讹拷锟斤拷锟斤拷实锟斤拷锟斤拷锟斤拷锟斤拷逊锟斤拷锟�
 * static CrsfReceiver radio(&huart1); // 全锟街伙拷态实锟斤拷锟斤拷锟狡硷拷锟斤�?
 * // 锟斤拷锟斤拷锟斤拷main锟叫讹拷锟藉�?
 * // CrsfReceiver radio(&huart1); // 锟皆讹拷锟芥储锟斤拷
 * radio.process();                               // 锟斤拷循锟斤拷锟斤拷锟斤�?
 * radio.getControlData(&ctrl);                   // 锟斤拷取锟斤拷锟斤拷
 * 
 * 锟斤拷锟斤拷锟斤拷锟捷结构锟斤拷RmPocketData_t锟斤�?
 * 摇锟斤拷: throttle(前锟斤拷), steering(�?锟斤�?), auxiliary1/2(锟斤拷锟斤拷)
 * 锟斤拷锟斤拷: sw_left/sw_right(锟斤拷锟斤拷), sw_sa/sw_sb/sw_sc(锟斤拷锟斤拷)
 * 锟斤拷钮: btn_l1/l2/r1/r2/menu/enter
 * 锟斤拷全: emergency_stop锟斤拷锟斤拷锟斤拷时停锟斤拷锟斤�?
 * 
 * 锟斤拷锟斤拷薷锟�?ｏ拷锟斤拷锟斤拷荩锟斤拷锟斤拷模锟�??
 * 默锟较凤拷锟酵�?�拷氐锟窖�??/锟斤拷锟斤拷/锟劫分比★拷
 * 锟�??发锟斤拷锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟铰讹拷/锟斤拷锟�?）锟斤�??
 * 
 * 锟斤拷锟斤拷1锟斤拷锟斤拷锟斤拷锟街�?�ｏ拷锟斤拷欤�??
 *   telem.battery_current = motor_temperature; // 锟斤拷锟斤拷锟�?��?�改筹拷锟铰讹拷
 *   radio->sendTelemetryData(&telem);
 * 
 * 锟斤拷锟斤拷2锟斤拷锟斤拷锟斤拷锟斤拷锟�?��?�ｏ拷锟狡硷拷锟斤拷
 *   // 1. 锟斤拷RmPocketData_t锟结构锟斤拷锟斤拷锟接ｏ�?
 *   float motor_temp;        // 锟斤拷锟斤拷露锟�?
 *   uint16_t obstacle_dist;  // 锟较�?拷锟斤拷锟斤拷锟�?
 *   
 *   // 2. 锟斤拷sendTelemetryData()锟斤拷锟斤拷锟斤拷锟斤拷锟接凤拷锟斤拷锟�?硷�??
 *   // 3. main锟斤拷锟斤拷锟斤拷锟斤拷荩锟�?
 *   telem.motor_temp = ReadTemperature();
 *   telem.obstacle_dist = sonar_read();
 *   radio->sendTelemetryData(&telem); // 锟皆讹拷锟斤拷锟斤拷
 
 
 * 锟斤拷锟�?猴拷锟斤�?
 * radio->setStickDeadzone(0.05f);    // 锟斤拷锟斤拷锟斤拷锟斤拷
 * radio->setThrottleCurve(1.2f);     // 锟斤拷锟斤拷锟斤拷锟斤拷
 * radio->isEmergencyStop();          // 锟斤拷榧蓖�
 * radio->getRawChannel(1);           // 锟斤拷取原�?�通锟斤拷�?
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

// RadioMaster-POCKET 锟斤拷锟斤拷通锟斤拷�?
#define RM_POCKET_CHANNEL_MIN 172      // 锟斤拷小�?
#define RM_POCKET_CHANNEL_MID 992      // 锟叫硷拷�?
#define RM_POCKET_CHANNEL_MAX 1811     // 锟斤拷锟街�

// 锟斤拷钮锟斤拷值锟斤拷锟斤拷锟斤拷RadioMaster-POCKET锟斤拷锟斤拷锟斤�?
#define RM_BTN_OFF 191                 // 锟斤拷钮锟酵凤拷
#define RM_BTN_ON 1792                 // 锟斤拷钮锟斤拷锟斤拷

// RadioMaster-POCKET 锟斤拷锟酵匡拷锟斤拷锟斤拷锟�?ｏ拷锟斤拷锟轿匡拷锟�?ｏ拷
#define RM_SWITCH_LOW 191              // 锟斤拷位锟斤�?
#define RM_SWITCH_MID 1004             // 锟斤拷位锟斤�?
#define RM_SWITCH_HIGH 1792            // 锟斤拷位锟斤�?

#define crclen 256

// 遥锟斤拷小锟斤拷专锟斤拷锟斤拷锟捷结�?
typedef struct
{
    // === 锟斤拷锟�?：锟斤拷RadioMaster-POCKET锟斤拷锟秸的匡拷锟斤拷锟斤拷锟斤�? ===
    // 摇锟斤拷锟斤拷锟捷ｏ拷映锟戒�?-1.0锟斤�?1.0锟斤拷围锟斤�?
    float throttle;                   // 锟斤拷锟斤拷/前锟斤拷锟斤拷锟斤拷 (-1.0锟斤拷锟斤拷 ~ 1.0前锟斤拷) - 锟斤拷锟介：锟斤拷摇锟剿达拷�?
    float steering;                   // �?锟斤�? (-1.0锟斤拷转 ~ 1.0锟斤拷转) - 锟斤拷锟介：锟斤拷摇锟斤拷水�?
    
    float auxiliary1;                 // 锟斤拷锟斤拷锟斤拷锟斤拷1 - 锟斤拷锟介：锟斤拷摇锟斤拷水平锟斤拷锟斤拷台锟斤拷锟�?ｏ�??
    float auxiliary2;                 // 锟斤拷锟斤拷锟斤拷锟斤拷2 - 锟斤拷锟介：锟斤拷摇锟剿达拷直锟斤拷锟斤拷台锟斤拷锟铰ｏ�?
    
    // RadioMaster-POCKET 锟斤拷锟斤拷状�?
    uint8_t sw_left;                  // 锟斤�?3锟轿匡拷锟截ｏ拷0=锟斤�?, 1=锟斤�?, 2=锟较ｏ拷通锟斤拷锟斤拷锟斤拷模式选锟斤拷
    uint8_t sw_right;                 // 锟斤�?3锟轿匡拷锟截ｏ拷0=锟斤�?, 1=锟斤�?, 2=锟较ｏ拷通锟斤拷锟斤拷锟斤拷锟劫度�?�拷位锟斤�??
    uint8_t sw_sa;                    // SA锟斤拷锟�?ｏ�??0=锟斤�?, 1=锟较ｏ拷2锟轿匡拷锟截ｏ拷
    uint8_t sw_sb;                    // SB锟斤拷锟�?ｏ�??0=锟斤�?, 1=锟较ｏ拷2锟轿匡拷锟截ｏ拷
    uint8_t sw_sc;                    // SC锟斤拷锟�?ｏ�??0=锟斤�?, 1=锟较ｏ拷2锟轿匡拷锟截ｏ拷
    
    // 锟斤拷钮状态锟斤拷RadioMaster-POCKET通锟斤拷锟斤�?6锟斤拷锟缴憋拷贪锟脚�?拷锟�?
    uint8_t btn_l1;                   // L1锟斤拷钮锟斤拷锟斤拷锟较凤拷锟斤�?
    uint8_t btn_l2;                   // L2锟斤拷钮锟斤拷锟斤拷锟铰凤拷锟斤�?
    uint8_t btn_r1;                   // R1锟斤拷钮锟斤拷锟斤拷锟较凤拷锟斤�?
    uint8_t btn_r2;                   // R2锟斤拷钮锟斤拷锟斤拷锟铰凤拷锟斤�?
    uint8_t btn_menu;                 // 锟剿碉拷锟斤拷钮
    uint8_t btn_enter;                // �?锟较�?拷钮
    
    // 锟斤拷锟解功锟斤�?
    uint8_t emergency_stop;           // 锟斤拷锟斤拷停�?�锟斤拷志锟斤拷通锟斤拷锟�?�定碉拷某锟斤拷锟斤拷钮锟斤�?
    uint8_t trigger_flag;             // 锟斤拷锟斤拷锟斤拷志
    
    // === 锟斤拷锟斤拷锟斤拷锟斤拷偷锟絉adioMaster-POCKET锟斤拷示锟斤拷锟斤拷锟斤�? ===
    // 锟斤拷锟斤拷锟较�?拷锟斤拷锟绞撅拷锟揭ｏ拷锟斤拷锟斤拷锟侥伙拷锟�??
    float battery_voltage;            // 锟斤拷氐锟窖�?(V)
    float battery_current;            // 锟斤拷氐锟斤拷锟�?(A)
    uint8_t battery_percent;          // 剩锟斤拷锟斤拷锟�?(%)
    uint32_t battery_capacity;        // 锟斤拷锟斤拷锟斤拷锟�?(mAh)
    
    // 小锟斤拷状�?
    float speed_kmh;                  // 锟斤拷前锟劫讹拷(km/h)
    float distance_km;                // 锟斤拷驶锟斤拷锟斤拷(km)
    uint16_t run_time_minutes;        // 锟斤拷锟斤拷时锟斤拷(锟斤拷锟斤拷)
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤�?
    float temperature;                // 锟铰讹拷(锟斤拷C)
    uint8_t signal_strength;          // 锟脚猴拷强锟斤拷(%)
    
//    // GPS锟斤拷锟捷ｏ拷锟斤拷锟叫★拷锟斤拷锟紾PS锟斤�?
//    double gps_latitude;              // �?锟斤�?
//    double gps_longitude;             // 锟斤拷锟斤拷
//    float gps_speed;                  // GPS锟劫讹拷(km/h)
//    uint8_t gps_satellites;           // 锟斤拷锟斤拷锟斤拷锟斤拷
//    
} RmPocketData_t;

// 原�?�CRSF锟斤拷锟捷结构锟斤拷锟斤拷锟斤拷锟斤拷协锟斤拷一锟铰ｏ拷
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

// CRC8锟斤拷锟斤拷锟斤�?
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

// RadioMaster-POCKET CRSF锟斤拷锟斤拷锟斤拷锟斤拷
class CrsfReceiver:public UART_
{
public:
    static CrsfReceiver* GetInstance(UART_HandleTypeDef *uart_handle);

    CrsfReceiver(const CrsfReceiver&) = delete;
    CrsfReceiver& operator=(const CrsfReceiver&) = delete;
    
    // ========== 锟斤拷�?�锟斤拷锟狡接匡�? ==========
    
    // 锟斤拷取RadioMaster-POCKET锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟津化接口ｏ拷
    void getControlData(RmPocketData_t *data);
    void InitUART(void);
    // 锟斤拷锟斤拷遥锟斤拷锟斤拷锟捷�?�拷RadioMaster-POCKET锟斤拷锟津化接口ｏ拷
    void sendTelemetryData(const RmPocketData_t *data);
    
    // 锟斤拷锟斤拷欠锟斤拷锟斤拷驴锟斤拷锟斤拷锟斤拷锟�?
    bool hasNewData() const { return new_data_available_; }
    void clearNewDataFlag() { new_data_available_ = false; }
    
    // 锟斤拷锟斤拷停�?�锟斤拷锟�??
    bool isEmergencyStop() const { return emergency_stop_triggered_; }
    void resetEmergencyStop() { emergency_stop_triggered_ = false; }
    
    // 锟斤拷取原�?�通锟斤拷值锟斤拷锟�??硷拷锟矫伙拷锟斤�?
    int getRawChannel(uint8_t ch) const;
    const int* getAllRawChannels() const { return channels_; }
    
    // ========== 锟斤拷锟�?接匡�? ==========
    
    // 锟斤拷锟斤拷摇锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷止摇锟斤拷锟斤拷�?锟斤拷锟斤拷锟斤�?
    void setStickDeadzone(float deadzone) { stick_deadzone_ = deadzone; }
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�?ｏ拷使锟斤拷应锟斤拷锟斤拷锟皆伙拷锟斤拷锟斤拷锟斤拷锟�??
    void setThrottleCurve(float curve_factor) { throttle_curve_ = curve_factor; }
    
    // 锟斤拷锟斤拷�?锟斤拷锟斤拷锟斤�?
    void setSteeringCurve(float curve_factor) { steering_curve_ = curve_factor; }
    
    // 锟斤拷锟�?凤拷锟斤拷�?�锟斤拷
    void setTelemetryRate(uint32_t battery_ms, uint32_t gps_ms) {
        telemetry_battery_interval_ = battery_ms;
        telemetry_gps_interval_ = gps_ms;
    }
    
    // 锟斤拷锟斤拷一锟轿达拷锟斤拷锟斤拷锟斤拷锟斤拷�?锟斤拷锟�?碉拷锟矫ｏ拷
    void process();
    // 娴�??�?锛氫复鏃剁�?�?/鍚�? D-Cache锛堜粎鐢ㄤ簬蹇�?熼獙璇侊�?
    void setDisableDCacheForTest(bool disable);
    bool isDCacheTestDisabled() const { return dcache_test_disabled_; }
    // 楠岃�? UART/DMA 閰嶇疆鏄惁婊¤冻寰幆鎺ユ敹涓� DMA 宸插垎閰�?
    bool isDmaConfiguredCorrectly() const { return dma_config_ok_; }
		UART_HandleTypeDef *uart_handle;
		void Callback_Fuc(uint8_t *buf, uint16_t len) override;
        // ���Թ۲⣺ͳ�ƽ��������ѵ��ֽ���
        uint32_t getIsrBytes() const { return isr_bytes_; }
        uint32_t getConsumedBytes() const { return consumed_bytes_; }
private:
    CrsfReceiver(UART_HandleTypeDef *huart);
    ~CrsfReceiver() = default;
    // 锟节诧拷锟斤拷锟斤拷
    int channels_[CRSF_NUM_CHANNELS];
    uint8_t channels_payload_[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE];
    uint8_t packet_byte_index_;
    uint8_t *payload_ptr_;
    volatile bool new_data_available_;
    bool emergency_stop_triggered_;
    
    // UART锟斤拷锟�?
    // 鍙�?�?: 灏� DMA 缂撳啿鍖烘斁鍒伴�?缂撳瓨澶�?鍐�?�? (姣斿�? D2) 浠ラ伩鍏�? D-Cache 闂銆�?
    // 鍦ㄩ」鐩腑瀹氫�? CRSF_DMA_SECTION_NAME 涓哄瓧绗︿�?�甯搁噺锛屽�?: "\.dma_buffer"
#ifdef CRSF_DMA_SECTION_NAME
#define CRSF_DMA_ATTR __attribute__((section(CRSF_DMA_SECTION_NAME), aligned(32)))
#else
#define CRSF_DMA_ATTR __attribute__((aligned(32)))
#endif
    uint8_t rx_buffer_[256] CRSF_DMA_ATTR;
    //UART_ uart_driver_;
    uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE] CRSF_DMA_ATTR;

    // 锟斤拷锟轿伙拷锟藉：锟斤拷UART锟截碉拷锟斤拷ISR锟斤拷锟斤拷锟侥ｏ拷锟斤拷锟斤拷写锟�?，锟斤拷锟斤拷循锟斤拷锟斤拷process()锟斤拷锟斤拷锟斤�?
    static const uint16_t RX_RING_SIZE = 512;
    uint8_t rx_ring_[RX_RING_SIZE] CRSF_DMA_ATTR;
    volatile uint16_t rx_ring_head_ = 0; // 写指锟�??（锟斤拷ISR锟斤拷锟铰ｏ�?
    volatile uint16_t rx_ring_tail_ = 0; // 锟斤拷指锟�??（锟斤拷锟斤拷循锟斤拷锟斤拷锟铰ｏ拷
    UART_* uart_instance_ = nullptr;
    bool uart_initialized_ = false;
    volatile uint32_t isr_bytes_ = 0;
    volatile uint32_t consumed_bytes_ = 0;
    
    // CRC
    GENERIC_CRC8 crc_;
    
    // 锟斤拷锟�?诧拷锟斤�?
    float stick_deadzone_ = 0.05f;           // 5%锟斤拷锟斤拷
    float throttle_curve_ = 1.0f;            // 1.0=锟斤拷锟斤拷
    float steering_curve_ = 1.0f;            // 1.0=锟斤拷锟斤拷
    
    // 遥锟解发锟酵讹拷�?
    uint32_t telemetry_battery_interval_ = 1000;  // 锟斤拷锟斤拷锟斤拷莘锟斤拷图锟斤�?(ms)
    uint32_t telemetry_gps_interval_ = 2000;      // GPS锟斤拷锟捷凤拷锟酵硷拷锟�?(ms)
    uint32_t last_battery_send_ = 0;
    uint32_t last_gps_send_ = 0;
    
    // 锟斤拷锟斤拷状态锟斤拷
    enum RxState {
        STATE_WAIT_ADDR,
        STATE_WAIT_SIZE,
        STATE_WAIT_TYPE,
        STATE_WAIT_PAYLOAD,
        STATE_WAIT_CRC,
        STATE_PACKET_COMPLETE
    } rx_state_;
    
    // 锟节诧拷锟斤拷锟斤拷锟斤拷锟斤拷
    void processBatchData(uint8_t *buf, uint16_t len);
    void handleByte(uint8_t byte);
    void processRcChannels();
    void unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS]);
    void computeMappedValues();
    void updateSwitchesAndButtons();
    // ISR-friendly append (锟斤拷锟�?匡拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤�??)
    void appendFromISR(const uint8_t *buf, uint16_t len);
    // 锟斤拷锟斤拷�?锟斤拷锟斤拷锟斤拷锟窖伙拷锟轿伙拷锟斤拷锟斤拷锟斤拷锟�??
    void consumeRingBuffer();
    // 鍐呴儴閰嶇疆�?�?鏌�
    void checkDmaConfig();
    
    // 锟斤拷态锟�?碉�??
    static CrsfReceiver* instance_;
    static void StaticUartCallback(uint8_t *buf, uint16_t len);
    
    // 映锟斤拷�?
    float throttle_raw_ = 0.0f;
    float steering_raw_ = 0.0f;
    float aux1_raw_ = 0.0f;
    float aux2_raw_ = 0.0f;
    
    // 锟斤拷锟斤拷/锟斤拷钮状�?
    uint8_t sw_left_ = 1;      // 默锟斤拷为锟�?硷拷位锟斤�??
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
    
    // 锟节诧拷锟斤拷志
    uint8_t last_emergency_btn_ = 0;
    bool dma_config_ok_ = false;
    bool dcache_test_disabled_ = false;

public:
    // 锟斤拷锟斤拷锟斤拷息锟斤拷锟斤拷选锟斤拷
    float debug_throttle = 0.0f;
    float debug_steering = 0.0f;
    uint8_t debug_mode = 0;
    
    // 锟斤拷锟斤拷锟斤拷志锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷ｏ�?
    void reset_trigger_flag() { }
    void set_trigger_flag_busy() { }
};

#endif // __cplusplus

#endif // Module_CRSF_RECEIVER_H