// Serial1Protocol.h
#ifndef MODULE_SERIAL1_PROTOCOL_H
#define MODULE_SERIAL1_PROTOCOL_H

#include "main.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 协议常量定义 */
#define SERIAL1_FRAME_HEAD0     0xFC
#define SERIAL1_FRAME_HEAD1     0xFB
#define SERIAL1_FRAME_TAIL0     0xFD
#define SERIAL1_FRAME_TAIL1     0xFE

#define SERIAL1_DATA_LEN        3
#define SERIAL1_FRAME_LEN       8

#define SERIAL1_SEND_TIMES      3          // 每批发送3次
#define SERIAL1_RETRY_INTERVAL  500        // 重发间隔500ms

/* 校验字节位定义 */
#define SERIAL1_CHECKSUM_MASK   0x3F
#define SERIAL1_PARITY_BIT_MASK 0x40
#define DATA_BUFFER_SIZE        16
typedef enum {
    DATA_TYPE_KFS = 0,
    DATA_TYPE_CMD = 1
} DataType_t;

// 数据包结构体
typedef struct {
    uint8_t type;     // 类型：KFS=0, CMD=1
    union {
        uint8_t cmd;      // 命令字节
        uint8_t kfs[3];   // KFS 3字节数据
    } data;
} DataPacket_t;
/* 状态枚举 */
typedef enum {
    SERIAL1_STATE_IDLE,           // 空闲
    SERIAL1_STATE_SENDING,        // 主动发送中
    SERIAL1_STATE_WAITING_ACK     // 等待主动发送的应答
} Serial1State_t;

/* 发送结果回调 */
typedef void (*Serial1SendResultCallback)(uint8_t* data, uint8_t parity, uint8_t success);

/* 数据接收回调（串口2主动发来的数据） */
typedef void (*Serial1DataReceiveCallback)(uint8_t* data, uint8_t parity);

class Serial1Protocol {
public:
    static Serial1Protocol& getInstance();
    
    void init(UART_HandleTypeDef* huart);
    void process(void);
    // 串口回调
    void onUartReceive(uint8_t* buffer, uint16_t size);
    void onUartTxComplete(void);
    
    void setSendResultCallback(Serial1SendResultCallback callback);
    void setDataReceiveCallback(Serial1DataReceiveCallback callback);
    
    // 获取接收到的数据（轮询方式）
    void getReceivedData(uint8_t* data_out, uint8_t* parity_out);
		void waitForSendComplete(void);
    void R1_Send_KFS(uint8_t KFS1, uint8_t KFS2, uint8_t KFS3);
    void send_cmd_to_R2(uint8_t);
		void sendAckFrame(void);
		bool hasData() const { return m_data_count > 0; }
		bool getLatestData(DataPacket_t* packet);
private:
    Serial1Protocol();
    bool sendCommand(uint8_t* data);
    uint8_t calculateChecksum(uint8_t* data);
    void buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out);
    int parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out);
    void sendFrame(uint8_t* data, uint8_t parity);
    uint32_t getTickMs(void);
    void notifySendResult(uint8_t success);
    bool has_consecutive_zeros_exceed_10(const uint8_t* data);
    
    // 奇偶位管理
    int findHistoryIndex(uint8_t* data);
    uint8_t getNextParity(uint8_t* data);
    void updateSendHistory(uint8_t* data, uint8_t parity);
    // 应答发送
    
private:
    UART_HandleTypeDef* m_huart;
    
    Serial1State_t m_state;
    Serial1SendResultCallback m_resultCallback;
    Serial1DataReceiveCallback m_dataReceiveCallback;
    
    // 主动发送相关
    uint8_t m_current_send_data[SERIAL1_DATA_LEN];
    uint8_t m_current_send_parity;
    uint8_t m_uart_send_frame[SERIAL1_FRAME_LEN];
    uint8_t m_send_batch_count;
    volatile uint8_t m_tx_complete;
    
    // 接收相关
    uint8_t m_rx_buffer[30];
    volatile uint8_t m_rx_ready;
    volatile uint16_t m_rx_size;
    
    uint8_t m_received_data[SERIAL1_DATA_LEN];
    uint8_t m_received_parity;
    uint8_t m_new_data_available;
    
    // 接收去重（用于主动数据）
    uint8_t m_last_rx_data[SERIAL1_DATA_LEN];
    uint8_t m_last_rx_parity;
    uint32_t m_last_rx_time;
    uint32_t m_last_send_time; 
    // 主动数据应答管理（用于判断相同数据重发）
    uint8_t m_last_processed_data[SERIAL1_DATA_LEN];   // 上次处理的数据
    uint8_t m_last_processed_parity;                  // 上次处理的奇偶
    
    // 发送历史记录
    #define MAX_HISTORY 10
    
    typedef struct {
        uint8_t data[SERIAL1_DATA_LEN];
        uint8_t last_parity;
        uint8_t send_count;
    } SendHistory_t;
    
    SendHistory_t m_send_history[MAX_HISTORY];
    uint8_t m_history_count;
    
    uint8_t m_command_send_data[SERIAL1_DATA_LEN];
    uint8_t m_command_send_parity;
		    // 数据存储数组
    DataPacket_t m_data_buffer[DATA_BUFFER_SIZE];  // 结构体数组
    volatile int m_data_write_index;  // 写入索引
    volatile int m_data_read_index;   // 读取索引
    volatile int m_data_count;        // 当前存储数量
    // 最新数据存储
    DataPacket_t m_latest_packet;      // 最新接收到的数据包
    bool m_has_latest_data;            // 是否有未消费的新数据
    // 辅助函数
    void storeReceivedData(uint8_t* data, uint8_t parity);
};

#ifdef __cplusplus
}
#endif

#endif