/**
 * @file Module_CrsfReceiver.h
 * @brief RadioMaster POCKET CRSF���ջ�
 *
 *Ӳ�����ӣ�4���ߣ�
 * POCKET JR��� �� STM32 UART
 * GND �� GND, 5V �� VCC, TX �� RX, RX �� TX
 * 
 * CubeMX���ã�3����
 * 1. UART: ������=420000, 8N1
 * 2. DMA: ����USART_RX��Circularģʽ��
 * 3. NVIC: ʹ��UART��DMA�ж�
 * 
 * ���뼯�ɣ�3�У�
 * // �Ƽ���Ƕ��ʽ������ʹ�þ�̬���Զ�����ʵ��������ѷ���
 * static CrsfReceiver radio(&huart1); // ȫ�ֻ�̬ʵ�����Ƽ���
 * // ������main�ж��壺
 * // CrsfReceiver radio(&huart1); // �Զ��洢��
 * radio.process();                               // ��ѭ������
 * radio.getControlData(&ctrl);                   // ��ȡ����
 * 
 * �������ݽṹ��RmPocketData_t��
 * ҡ��: throttle(ǰ��), steering(ת��), auxiliary1/2(����)
 * ����: sw_left/sw_right(����), sw_sa/sw_sb/sw_sc(����)
 * ��ť: btn_l1/l2/r1/r2/menu/enter
 * ��ȫ: emergency_stop������ʱͣ����
 * 
 * ����޸�ң�����ݣ����ģ�
 * Ĭ�Ϸ��͵�ص�ѹ/����/�ٷֱȡ�
 * �뷢���������ݣ����¶�/���룩��
 * 
 * ����1�������ֶΣ���죩
 *   telem.battery_current = motor_temperature; // �����ֶθĳ��¶�
 *   radio->sendTelemetryData(&telem);
 * 
 * ����2���������ֶΣ��Ƽ���
 *   // 1. ��RmPocketData_t�ṹ�����ӣ�
 *   float motor_temp;        // ����¶�
 *   uint16_t obstacle_dist;  // �ϰ������
 *   
 *   // 2. ��sendTelemetryData()���������ӷ����߼�
 *   // 3. main��������ݣ�
 *   telem.motor_temp = ReadTemperature();
 *   telem.obstacle_dist = sonar_read();
 *   radio->sendTelemetryData(&telem); // �Զ�����
 
 
 * ���ú���
 * radio->setStickDeadzone(0.05f);    // ��������
 * radio->setThrottleCurve(1.2f);     // ��������
 * radio->isEmergencyStop();          // ��鼱ͣ
 * radio->getRawChannel(1);           // ��ȡԭʼͨ��ֵ
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

// RadioMaster-POCKET ����ͨ��ֵ
#define RM_POCKET_CHANNEL_MIN 172      // ��Сֵ
#define RM_POCKET_CHANNEL_MID 992      // �м�ֵ
#define RM_POCKET_CHANNEL_MAX 1811     // ���ֵ

// ��ť��ֵ������RadioMaster-POCKET������
#define RM_BTN_OFF 191                 // ��ť�ͷ�
#define RM_BTN_ON 1792                 // ��ť����

// RadioMaster-POCKET ���Ϳ������ã����ο��أ�
#define RM_SWITCH_LOW 191              // ��λ��
#define RM_SWITCH_MID 1004             // ��λ��
#define RM_SWITCH_HIGH 1792            // ��λ��

#define crclen 256

// ң��С��ר�����ݽṹ
typedef struct
{
    // === ���룺��RadioMaster-POCKET���յĿ������� ===
    // ҡ�����ݣ�ӳ�䵽-1.0��1.0��Χ��
    float throttle;                   // ����/ǰ������ (-1.0���� ~ 1.0ǰ��) - ���飺��ҡ�˴�ֱ
    float steering;                   // ת�� (-1.0��ת ~ 1.0��ת) - ���飺��ҡ��ˮƽ
    
    float auxiliary1;                 // ��������1 - ���飺��ҡ��ˮƽ����̨���ң�
    float auxiliary2;                 // ��������2 - ���飺��ҡ�˴�ֱ����̨���£�
    
    // RadioMaster-POCKET ����״̬
    uint8_t sw_left;                  // ��3�ο��أ�0=��, 1=��, 2=�ϣ�ͨ������ģʽѡ��
    uint8_t sw_right;                 // ��3�ο��أ�0=��, 1=��, 2=�ϣ�ͨ�������ٶȵ�λ��
    uint8_t sw_sa;                    // SA���أ�0=��, 1=�ϣ�2�ο��أ�
    uint8_t sw_sb;                    // SB���أ�0=��, 1=�ϣ�2�ο��أ�
    uint8_t sw_sc;                    // SC���أ�0=��, 1=�ϣ�2�ο��أ�
    
    // ��ť״̬��RadioMaster-POCKETͨ����6���ɱ�̰�ť��
    uint8_t btn_l1;                   // L1��ť�����Ϸ���
    uint8_t btn_l2;                   // L2��ť�����·���
    uint8_t btn_r1;                   // R1��ť�����Ϸ���
    uint8_t btn_r2;                   // R2��ť�����·���
    uint8_t btn_menu;                 // �˵���ť
    uint8_t btn_enter;                // ȷ�ϰ�ť
    
    // ���⹦��
    uint8_t emergency_stop;           // ����ֹͣ��־��ͨ���󶨵�ĳ����ť��
    uint8_t trigger_flag;             // ������־
    
    // === ��������͵�RadioMaster-POCKET��ʾ������ ===
    // �����Ϣ����ʾ��ң������Ļ��
    float battery_voltage;            // ��ص�ѹ(V)
    float battery_current;            // ��ص���(A)
    uint8_t battery_percent;          // ʣ�����(%)
    uint32_t battery_capacity;        // �������(mAh)
    
    // С��״̬
    float speed_kmh;                  // ��ǰ�ٶ�(km/h)
    float distance_km;                // ��ʻ����(km)
    uint16_t run_time_minutes;        // ����ʱ��(����)
    
    // ����������
    float temperature;                // �¶�(��C)
    uint8_t signal_strength;          // �ź�ǿ��(%)
    
//    // GPS���ݣ����С����GPS��
//    double gps_latitude;              // γ��
//    double gps_longitude;             // ����
//    float gps_speed;                  // GPS�ٶ�(km/h)
//    uint8_t gps_satellites;           // ��������
//    
} RmPocketData_t;

// ԭʼCRSF���ݽṹ��������Э��һ�£�
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

// CRC8������
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

// RadioMaster-POCKET CRSF��������
class CrsfReceiver:public UART_
{
public:
    // ���캯��
    CrsfReceiver(UART_HandleTypeDef *huart);
    
    // ========== ��Ҫ���ƽӿ� ==========
    
    // ��ȡRadioMaster-POCKET�������ݣ��򻯽ӿڣ�
    void getControlData(RmPocketData_t *data);
    void init(void){    this->UART_Init();}
    // ����ң�����ݵ�RadioMaster-POCKET���򻯽ӿڣ�
    void sendTelemetryData(const RmPocketData_t *data);
    
    // ����Ƿ����¿�������
    bool hasNewData() const { return new_data_available_; }
    void clearNewDataFlag() { new_data_available_ = false; }
    
    // ����ֹͣ���
    bool isEmergencyStop() const { return emergency_stop_triggered_; }
    void resetEmergencyStop() { emergency_stop_triggered_ = false; }
    
    // ��ȡԭʼͨ��ֵ���߼��û���
    int getRawChannel(uint8_t ch) const;
    const int* getAllRawChannels() const { return channels_; }
    
    // ========== ���ýӿ� ==========
    
    // ����ҡ����������ֹҡ����΢������
    void setStickDeadzone(float deadzone) { stick_deadzone_ = deadzone; }
    
    // �����������ߣ�ʹ��Ӧ�����Ի��������
    void setThrottleCurve(float curve_factor) { throttle_curve_ = curve_factor; }
    
    // ����ת������
    void setSteeringCurve(float curve_factor) { steering_curve_ = curve_factor; }
    
    // ���÷���Ƶ��
    void setTelemetryRate(uint32_t battery_ms, uint32_t gps_ms) {
        telemetry_battery_interval_ = battery_ms;
        telemetry_gps_interval_ = gps_ms;
    }
    
    // ����һ�δ���������ѭ���е��ã�
    void process();
    // 测试：临时禁用/启用 D-Cache（仅用于快速验证）
    void setDisableDCacheForTest(bool disable);
    bool isDCacheTestDisabled() const { return dcache_test_disabled_; }
    // 验证 UART/DMA 配置是否满足循环接收且 DMA 已分配
    bool isDmaConfiguredCorrectly() const { return dma_config_ok_; }
		UART_HandleTypeDef *uart_handle;
		void Callback_Fuc(uint8_t *buf, uint16_t len) override;
private:
    // �ڲ�����
    int channels_[CRSF_NUM_CHANNELS];
    uint8_t channels_payload_[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE];
    uint8_t packet_byte_index_;
    uint8_t *payload_ptr_;
    volatile bool new_data_available_;
    bool emergency_stop_triggered_;
    
    // UART���
    // 可选: 将 DMA 缓冲区放到非缓存外设内存 (比如 D2) 以避免 D-Cache 问题。
    // 在项目中定义 CRSF_DMA_SECTION_NAME 为字符串常量，如: "\.dma_buffer"
#ifdef CRSF_DMA_SECTION_NAME
#define CRSF_DMA_ATTR __attribute__((section(CRSF_DMA_SECTION_NAME), aligned(32)))
#else
#define CRSF_DMA_ATTR __attribute__((aligned(32)))
#endif
    uint8_t rx_buffer_[256] CRSF_DMA_ATTR;
    //UART_ uart_driver_;
    uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE] CRSF_DMA_ATTR;

    // ���λ��壺��UART�ص���ISR�����ģ�����д�룬����ѭ����process()������
    static const uint16_t RX_RING_SIZE = 512;
    uint8_t rx_ring_[RX_RING_SIZE] CRSF_DMA_ATTR;
    volatile uint16_t rx_ring_head_ = 0; // дָ�루��ISR���£�
    volatile uint16_t rx_ring_tail_ = 0; // ��ָ�루����ѭ�����£�
    
    // CRC
    GENERIC_CRC8 crc_;
    
    // ���ò���
    float stick_deadzone_ = 0.05f;           // 5%����
    float throttle_curve_ = 1.0f;            // 1.0=����
    float steering_curve_ = 1.0f;            // 1.0=����
    
    // ң�ⷢ�Ͷ�ʱ
    uint32_t telemetry_battery_interval_ = 1000;  // ������ݷ��ͼ��(ms)
    uint32_t telemetry_gps_interval_ = 2000;      // GPS���ݷ��ͼ��(ms)
    uint32_t last_battery_send_ = 0;
    uint32_t last_gps_send_ = 0;
    
    // ����״̬��
    enum RxState {
        STATE_WAIT_ADDR,
        STATE_WAIT_SIZE,
        STATE_WAIT_TYPE,
        STATE_WAIT_PAYLOAD,
        STATE_WAIT_CRC,
        STATE_PACKET_COMPLETE
    } rx_state_;
    
    // �ڲ���������
    void processBatchData(uint8_t *buf, uint16_t len);
    void handleByte(uint8_t byte);
    void processRcChannels();
    void unpackChannels(const uint8_t *payload, int channels[CRSF_NUM_CHANNELS]);
    void computeMappedValues();
    void updateSwitchesAndButtons();
    // ISR-friendly append (���ٿ�����������������)
    void appendFromISR(const uint8_t *buf, uint16_t len);
    // ����ѭ�������ѻ��λ��������
    void consumeRingBuffer();
    // 内部配置检查
    void checkDmaConfig();
    
    // ��̬�ص�
    static CrsfReceiver* instance_;
    static void StaticUartCallback(uint8_t *buf, uint16_t len);
    
    // ӳ��ֵ
    float throttle_raw_ = 0.0f;
    float steering_raw_ = 0.0f;
    float aux1_raw_ = 0.0f;
    float aux2_raw_ = 0.0f;
    
    // ����/��ť״̬
    uint8_t sw_left_ = 1;      // Ĭ��Ϊ�м�λ��
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
    
    // �ڲ���־
    uint8_t last_emergency_btn_ = 0;
    bool dma_config_ok_ = false;
    bool dcache_test_disabled_ = false;

public:
    // ������Ϣ����ѡ��
    float debug_throttle = 0.0f;
    float debug_steering = 0.0f;
    uint8_t debug_mode = 0;
    
    // ������־�����������ݣ�
    void reset_trigger_flag() { }
    void set_trigger_flag_busy() { }
};

#endif // __cplusplus

#endif // Module_CRSF_RECEIVER_H