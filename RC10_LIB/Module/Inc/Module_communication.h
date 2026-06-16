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
        uint8_t page;
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

    //接收帧：串口屏转发的命令帧
    typedef struct {
        uint8_t header[2]; // 0xAA 0x77
        uint8_t command;   // 0-99分别表示不同的命令，机器人侧查表执行
        uint8_t load1;     // 发送的累计次数，8位 0-255
        uint8_t load2;     // 保留扩展
        uint8_t crc;
        uint8_t tail;      // 0xDE
    } CommandFrame_t;

    // 发送帧：XYZ (3 x 16位 有符号)  — 与遥控器端 XYZFrame_t 对齐
    typedef struct {
        uint8_t header[2]; // e.g. 0x55 0xAA
        uint16_t x;
        uint16_t y;
        uint16_t z;
        uint8_t status;   // bit5-3:夹爪状态 bit2-1：吸盘状态 bit0:自动模式状态
        uint8_t mode;
        uint8_t command1;
        uint8_t command2;
        uint8_t KFS_want_place1; // （高四位为索引1的位置，低四位为索引0位置）
        uint8_t KFS_want_place2; // （高四位为索引3的位置，低四位为索引2位置）
        uint8_t spear;           // 0x00-0x07分别表示不同的武器头夹取状态
        uint8_t KFS_Keepplace;   // KFS存储区
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
         * @note 时机/用法：写在发送的GPIO外部中断里面，模块繁忙时，锁tx_busy=1，模块空闲时，解锁tx_busy=0，不建议写在发送回调，因为DMA发送完成不代表模块可发送下一帧
         */
        void Comm_TxBufferToTxDMA(UART_HandleTypeDef *txhuart);

        /**
         * @brief 将任意格式的一段数据塞入发送队列，并尝试激活发送，需要在接收工程中添加实现，谨慎使用
         * @param data 要发送的数据首地址
         * @param len  数据长度
         * @note 时机/用法：暂时设计为公开接口，理论上任何时候都可以调用，但需要用户自行保证调用时机和数据格式的合理性，以及发送过程中对 tx_busy 标志的正确管理
         */
        void Comm_SendAnyDataToTxBuffer(const uint8_t* data, uint16_t len);

        /**
         * @brief 将底层DMA接收到的无序缓存数据推入业务侧接收环形缓冲区
         * @param rxhuart 产生中断的接收UART句柄
         * @param size    本次DMA/空闲中断接收到的实际长度
         * @note 时机/用法：在接收中断服务程序中调用，将DMA接收到的数据推入业务侧接收环形缓冲区
         */
        void Comm_RxDMAToRxBuffer(UART_HandleTypeDef *rxhuart, uint16_t size);

        /**
         * @brief 将XYZ坐标值结合帧头包尾等组装为标准包，压入发送队列并触发发送
         * @param x X轴坐标 (16位)
         * @param y Y轴坐标 (16位)
         * @param z Z轴坐标 (16位)
         * @param Gripper_Status 夹爪状态 (bit5-3)
         * @param Suction_Cup_Status 吸盘状态 (bit2-1)
         * @param Automatic_status 自动模式状态 (bit0)
         * @param mode 模式 (8位)
         * @param command1 预留命令1 (8位)
         * @param command2 预留命令2 (8位)
         * @param KFS_want_place1 KFS位置索引0和1（高四位索引1，低四位索引0）
         * @param KFS_want_place2 KFS位置索引2和3（高四位索引3，低四位索引2）
         * @param spear 武器头夹取状态 0x00-0x07
         * @param KFS_Keepplace KFS存储区
         * @note 时机/用法：检查在1ms定时中调用，外部需先判断锁tx_busy是否为0，且根据实际业务需求合理设置坐标和状态参数。函数内部会自动组装成标准帧并尝试触发发送。
         */
        void Comm_SendAxisDataToTxBuffer(uint16_t  x, uint16_t y, uint16_t z,
            uint8_t Gripper_Status, uint8_t Suction_Cup_Status,uint8_t Automatic_status, uint8_t mode, uint8_t command1, uint8_t command2,
            uint8_t KFS_want_place1, uint8_t KFS_want_place2, uint8_t spear, uint8_t KFS_Keepplace);

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
         * @param joystick 存放摇杆数据的数组，length为4，分别对应4个通道的16位数据
         * @note 时机/用法：需要获取最新摇杆数据时调用，前提是 Comm_Task_Loop 已经成功解析出至少一帧合法数据并刷新了相关变量。调用后，外部即可获得最新的摇杆数据。
         */
        void GetRecvJoystickData(uint16_t* joystick) {
            for(int i = 0; i < 4; i++) joystick[i] = rec_joystick[i];
        }

        /**
         * @brief 获取接收到的命令帧数据（串口屏转发的命令）
         * @param command 存放命令的变量
         * @param load1 存放负载1的变量（累计次数）
         * @param load2 存放负载2的变量（保留扩展）
         * @note 时机/用法：在 Comm_Task_Loop 返回 true 后调用，获取最新的命令帧数据。
         */
        void GetRecvCommandFrameData(uint8_t& command, uint8_t& load1, uint8_t& load2) {
            command = rec_command_command;
            load1 = rec_command_load1;
            load2 = rec_command_load2;
        }

        /**
         * @brief [已废弃] 设置待发送的KFS相关数据，已合并至 Comm_SendAxisDataToTxBuffer 参数中
         * @deprecated 请直接在 Comm_SendAxisDataToTxBuffer 调用时传入 KFS 参数，无需单独调用此函数
         */
        void SetSendKFSData(uint8_t KFS_want_place1, uint8_t KFS_want_place2, uint8_t spear, uint8_t KFS_Keepplace) {
            send_KFS_want_place1 = KFS_want_place1;
            send_KFS_want_place2 = KFS_want_place2;
            send_spear = spear;
            send_KFS_Keepplace = KFS_Keepplace;
        }

        /**
         * @brief 获取接收到的KFS位置数据
         * @param index 位置索引 (0~2)，对应 rec_setting_load1 的低四位和高四位，以及 rec_setting_load2 的低四位
         * @return uint8_t 索引对应的KFS位置值，若索引无效或当前命令不是0x01，则返回0
         * @note 时机/用法：在 Comm_Task_Loop 返回 true 后调用，获取最新的KFS位置数据。典型用法：uint8_t kfs0 = comm.GetRecvFKFS1Data(0); // 获取索引0位置
         */
        uint8_t GetRecvFKFS1Data(uint8_t index) {
            switch (index) {
                case 1: return rec_KFS1_place1; // 索引1位置
                case 2: return rec_KFS1_place2; // 索引2位置
                case 3: return rec_KFS1_place3; // 索引3位置
                default: return 0; // 无效索引返回0   
            }
        }

        uint8_t GetRecvFKFS2Data(uint8_t index) {
            switch (index) {
                case 1: return rec_KFS2_place1; // 索引1位置
                case 2: return rec_KFS2_place2; // 索引2位置
                case 3: return rec_KFS2_place3; // 索引3位置
                case 4: return rec_KFS2_place4; // 索引4位置
                default: return 0; // 无效索引返回0   
            }
        }

        uint8_t GetRecvFKFSfData(uint8_t index) {
            switch (index) {
                case 1: return rec_KFSf_place1; // 索引1位置
                default: return 0; // 无效索引返回0   
            }
        }

        /**
         * @brief 查询指定索引的按键是否被按下
         * @param key_index 按键索引 (0~15)，对应 rec_send_key 的 bit0~bit15
         * @return true 表示该按键按下（对应位为1），false 表示未按下
         * @note 时机/用法：在 Comm_Task_Loop 返回 true 后调用，获取最新按键状态。
         *       典型用法：if (comm.GetRecvKeyData(0)) { // KEY0 按下处理 }
         */
        bool GetRecvKeyData(uint8_t key_index) {
            if (key_index >= 16) return false;
            return (rec_send_key >> key_index) & 0x01;
        }

        /**
         * @brief 获取全部16个按键的原始位图
         * @return uint16_t 每一位代表一个按键状态，bit0=KEY0, bit1=KEY1, ... bit15=KEY15
         * @note 时机/用法：在 Comm_Task_Loop 返回 true 后调用，适合需要批量处理按键的场景。
         */
        uint16_t GetRecvAllKeyData(void) {
            return rec_send_key;
        }

        /**
         * @brief 获取接收到的 page 值
         * @return uint8_t 当前帧的 page 字段
         * @note 时机/用法：在 Comm_Task_Loop 返回 true 后调用，获取最新 page 值。
         */
        uint8_t GetPage(void) {
            return rec_page&0x0F; // 低四位为 page，返回时屏蔽高四位的颜色信息
        }

        uint8_t GetColor(void) {
            return rec_page>>4;
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
        uint8_t rec_page;
        uint8_t rec_setting_command;
        uint8_t rec_setting_load1;
        uint8_t rec_setting_load2;

        // 接收到的命令帧数据（串口屏转发，0xAA 0x66，command 0-99）
        uint8_t rec_command_command;
        uint8_t rec_command_load1;
        uint8_t rec_command_load2;

        // 待发送的KFS相关数据（填入XYZ帧扩展字段）
        uint8_t send_KFS_want_place1;
        uint8_t send_KFS_want_place2;
        uint8_t send_spear;
        uint8_t send_KFS_Keepplace;

        uint8_t rec_KFS1_place1;
        uint8_t rec_KFS1_place2;
        uint8_t rec_KFS1_place3;
        uint8_t rec_KFS2_place1;
        uint8_t rec_KFS2_place2;
        uint8_t rec_KFS2_place3;
        uint8_t rec_KFS2_place4;
        uint8_t rec_KFSf_place1;
    protected:
    };
}

#endif
