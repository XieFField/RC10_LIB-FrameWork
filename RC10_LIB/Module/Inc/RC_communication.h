#pragma once

#include "usart.h"

#define RING_BUF_SIZE 256
#define DMA_BUF_SIZE  64

#ifdef __cplusplus

namespace communication{
    typedef struct {
    uint8_t* buffer;
    volatile uint16_t head;
    volatile uint16_t tail;
    } comm_FIFO_t;

    /* 强制一字节对齐的数据帧 */
    #pragma pack(push, 1)

    // 接收帧：摇杆4个通道 (4 x 16位)
    typedef struct {
        uint8_t header[2]; // e.g. 0xAA 0x55
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint16_t ch4;
        uint16_t key;
        uint8_t crc;
        uint8_t tail;      // e.g. 0xDE
    } JoystickFrame_t;

    //接收帧：接收设置好的KFS位置
    typedef struct {
        uint8_t header[2]; // e.g. 0xAA 0x66
        uint8_t command;   // e.g. 0x01表示发送KFS位置
        uint8_t load1;     // e.g. comand=0x01时，load1表示高四位为索引为1的位置，低四位为索引为0位置
        uint8_t load2;     // e.g. comand=0x01时，load2表示高四位为索引为3的位置，低四位为索引为2位置
        uint8_t crc;
        uint8_t tail;      // e.g. 0xDE
    } SettingFrame_t;

    // 发送帧：XYZ (3 x 16位)
    typedef struct {
        uint8_t header[2]; // e.g. 0x55 0xAA
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint8_t status;   // bit5-3:夹爪状态 bit2-1：吸盘状态 bit0:自动模式状态
        uint8_t mode;
        uint8_t command1;
        uint8_t command2;
        uint8_t crc;
        uint8_t tail;      // e.g. 0xED
    } XYZFrame_t;

    #pragma pack(pop)

    class Communication {
    public:
        /**
         * @brief 构造函数，初始化通信类状态并绑定底层硬件资源
         * @param txhuart   发使用的UART句柄
         * @param rxhuart   收使用的UART句柄
         * @param tx_ring_buf  发送环形缓冲区数组指针 (大小建议为 RING_BUF_SIZE)
         * @param tx_dma_buf   发送DMA线性缓冲区数组指针 (大小建议为 DMA_BUF_SIZE)
         * @param rx_ring_buf  接收环形缓冲区数组指针 (大小建议为 RING_BUF_SIZE)
         * @param rx_dma_buf   接收DMA线性缓冲区数组指针 (大小建议为 DMA_BUF_SIZE)
         * @note 时机/用法：在系统初始化阶段(如main函数或应用层初始化代码中)调用一次。传入的缓冲区内存通常由外部按需静态分配传递进来，便于内存管理。
         */
        Communication(UART_HandleTypeDef *txhuart,UART_HandleTypeDef *rxhuart,
            uint8_t *tx_ring_buf,uint8_t *tx_dma_buf,uint8_t *rx_ring_buf,uint8_t *rx_dma_buf,
            GPIO_TypeDef *tx_aux_port, uint16_t tx_aux_pin,
            GPIO_TypeDef *rx_aux_port, uint16_t rx_aux_pin);
        
        ~Communication();

        /**
         * @brief 通信解析主循环，负责从接收环形队列里寻头、校验并解包数据帧
         * @return bool 如果找到合法帧并且校验成功，返回 true；无数据或者数据未成满帧返回 false
         * @note 时机/用法：需要放置在主循环(如 while(1))或RTOS通信任务的死循环中高频调用。当返回true时，意味着内部已经刷新了摇杆数据等变量，外部可进一步提取。
         */
        bool Comm_Task_Loop(void);

        /**
         * @brief 将发送环形缓冲区的数据转移到DMA缓冲区并拉起底层硬件发送
         * @param txhuart 触发调用的UART句柄（内部用于防重防错判）
         * @note 时机/用法：写在发送的GPIO外部中断里面，不可以写在发送回调！！！！！！！！
         */
        void Comm_TxBufferToTxDMA(UART_HandleTypeDef *txhuart);

        /**
         * @brief 将任意格式的一段数据塞入发送队列，并尝试激活发送
         * @param data 要发送的数据首地址
         * @param len  数据长度
         * @note 时机/用法：当业务逻辑需要通过该串口打印日志、发送其他非标准结构的数据流时直接调用。
         */
        void Comm_SendAnyDataToTxBuffer(const uint8_t* data, uint16_t len);

        /**
         * @brief 将底层DMA接收到的无序缓存数据推入业务侧接收环形缓冲区
         * @param rxhuart 产生中断的接收UART句柄
         * @param size    本次DMA/空闲中断接收到的实际长度
         * @note 时机/用法：必须在串口空闲中断 (UARTEx_RxEventCallback)  中第一时间调用，防止新一轮DMA覆盖旧数据。
         */
        void Comm_RxDMAToRxBuffer(UART_HandleTypeDef *rxhuart, uint16_t size);

        /**
         * @brief 将XYZ坐标值结合帧头包尾等组装为标准包，压入发送队列并触发发送
         * @param x X轴坐标 (16位)
         * @param y Y轴坐标 (16位)
         * @param z Z轴坐标 (16位)
         * @note 时机/用法：定时器更新中断调用发送
         */
        void Comm_SendAxisDataToTxBuffer(uint16_t  x, uint16_t y, uint16_t z,
            uint8_t Gripper_Status, uint8_t Suction_Cup_Status,uint8_t Automatic_status, uint8_t mode, uint8_t command1, uint8_t command2);

        /**
         * @brief 纯虚函数：启动底层物理发送动作
         * @param huart 待发送串口句柄
         * @param data  待发送固定线性缓存区指针
         * @param size  待发送字节数
         * @note 时机/用法：用户需继承此 Communication 类，重写并在内部实现 `HAL_UART_Transmit_DMA(huart, data, size);` 等平台相关底层操作。
         */
        virtual void Comm_TxUseTxDMA(UART_HandleTypeDef * huart, uint8_t* data, uint16_t size)=0;

        /**
         * @brief 获取接收到的业务数据
         */
        void GetRecvData(uint16_t* joystick, uint16_t& key) {
            for(int i = 0; i < 4; i++) joystick[i] = rec_joystick[i];
            key = rec_send_key;
        }

        /**
         * @brief 获取接收到的设置帧数据
         */
        void GetSettingData(uint8_t& command, uint8_t& load1, uint8_t& load2) {
            command = rec_setting_command;
            load1 = rec_setting_load1;
            load2 = rec_setting_load2;
        }
    private:
        void FIFO_Push(comm_FIFO_t& fifo, uint8_t data);

        int FIFO_Pop(comm_FIFO_t& fifo, uint8_t& data);

        uint16_t FIFO_Count(comm_FIFO_t& fifo);

        uint8_t crc8(const uint8_t *data, uint8_t len);
        
        UART_HandleTypeDef* txhuart;
        UART_HandleTypeDef* rxhuart;  

        GPIO_TypeDef* tx_aux_port;
        uint16_t tx_aux_pin;
        GPIO_TypeDef* rx_aux_port;
        uint16_t rx_aux_pin;

        /* 专供 DMA 直接收发的线性缓冲区 */
        uint8_t* dma_rx_buf;
        uint8_t* dma_tx_buf;

        /* 用于业务逻辑和DMA之间解耦的环形缓冲区 */
        comm_FIFO_t rx_fifo;
        comm_FIFO_t tx_fifo;

        volatile uint8_t tx_busy; // 发送忙碌标志

        /* 解析出来/待发送的业务数据 */
        uint16_t send_xyz[3]; 
        uint8_t send_mode;
        uint8_t send_status;   // bit5-3:夹爪状态 bit2-1：吸盘状态 bit0:自动模式状态
        uint8_t send_command1;
        uint8_t send_command2;
        uint16_t rec_joystick[4];
        uint16_t rec_send_key;
        uint8_t rec_setting_command;
        uint8_t rec_setting_load1;
        uint8_t rec_setting_load2;
    protected:
    };
}

#endif

