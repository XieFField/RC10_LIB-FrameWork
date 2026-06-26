#include "Module_communication.h"

uint32_t rx_cnt; // 接收计数
uint32_t tx_cnt; // 发送计数

namespace communication{
    Communication::Communication(UART_HandleTypeDef *txhuart,UART_HandleTypeDef *rxhuart,
        uint8_t *tx_ring_buf,uint8_t *tx_dma_buf,uint8_t *rx_ring_buf,uint8_t *rx_dma_buf,
        GPIO_TypeDef *tx_aux_port, uint16_t tx_aux_pin,
        GPIO_TypeDef *rx_aux_port, uint16_t rx_aux_pin)
    {
        this->txhuart = txhuart;
        this->rxhuart = rxhuart;
        this->tx_aux_port = tx_aux_port;
        this->tx_aux_pin = tx_aux_pin;
        this->rx_aux_port = rx_aux_port;
        this->rx_aux_pin = rx_aux_pin;

        this->dma_tx_buf = tx_dma_buf;
        this->dma_rx_buf = rx_dma_buf;

        this->tx_fifo.buffer = tx_ring_buf;
        this->tx_fifo.head = 0;
        this->tx_fifo.tail = 0;
        
        this->rx_fifo.buffer = rx_ring_buf;
        this->rx_fifo.head = 0;
        this->rx_fifo.tail = 0;

        this->tx_busy = 0; // 初始状态为非忙碌

        send_xyz[0]=0xA9CB;
        send_xyz[1]=0x6587;
        send_xyz[2]=0x2143;
        send_mode = 0xAA;
        send_status = 0xBB;
        send_command1 = 0xCC;
        send_command2 = 0xDD;

        
        rec_setting_command = 0;
        rec_setting_load1 = 0;
        rec_setting_load2 = 0;

        // 初始化命令帧接收变量
        rec_command_command = 0;
        rec_command_load1 = 0;
        rec_command_load2 = 0;

        // 初始化KFS发送变量
        send_KFS_want_place1 = 0;
        send_KFS_want_place2 = 0;
        send_spear = 0;
        send_KFS_Keepplace = 0;

        rec_KFS1_place1=0;
        rec_KFS1_place2=0;
        rec_KFS1_place3=0;
        rec_KFS2_place1=0;
        rec_KFS2_place2=0;
        rec_KFS2_place3=0;
        rec_KFS2_place4=0;
        rec_KFSf_place1=0;

        saven_color = 1;
        // Communication_RX_DMA(rxhuart, dma_rx_buf, DMA_BUF_SIZE);
        // __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 
    }

    Communication::~Communication()
    {
    }

    void Communication::FIFO_Push(comm_FIFO_t& fifo, uint8_t data) {
        uint16_t next = (fifo.tail + 1) % RING_BUF_SIZE;
        if (next != fifo.head) { 
            fifo.buffer[fifo.tail] = data;
            fifo.tail = next;
        }
    }

    // 主循环从头部 (head) 取出处理
    int Communication::FIFO_Pop(comm_FIFO_t& fifo, uint8_t& data) {
        if (fifo.head == fifo.tail) return 0; 
        data = fifo.buffer[fifo.head];
        fifo.head = (fifo.head + 1) % RING_BUF_SIZE;
        return 1;
    }

    uint16_t Communication::FIFO_Count(comm_FIFO_t& fifo) {
        return (fifo.tail + RING_BUF_SIZE - fifo.head) % RING_BUF_SIZE;
    }

    namespace {//0xD5
        const uint8_t crc8_dvb_table[256] = {
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
            0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
            0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
            0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
            0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
            0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
            0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
            0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
            0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
            0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
            0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
            0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
            0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
            0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
            0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
        };
    }

    uint8_t Communication::crc8(const uint8_t *data, uint8_t len) {
        uint8_t crc = 0x00;// initial value
        for (uint8_t i = 0; i < len; i++) {
            crc = crc8_dvb_table[crc ^ data[i]];
        }
        return crc;
    }

    bool Communication::Comm_Task_Loop(void)
    {
        bool data_updated = false;
        // 不断处理 RX FIFO 里面收到的数据，直到剩余数据不足最小帧长度
        while (FIFO_Count(rx_fifo) >= sizeof(SettingFrame_t)) {
        
        uint16_t t_head = rx_fifo.head;
        uint8_t byte1 = rx_fifo.buffer[t_head];
        uint8_t byte2 = rx_fifo.buffer[(t_head + 1) % RING_BUF_SIZE];
        
        // 查找帧头 0xAA 0x55 (摇杆帧)
        if (byte1 == 0xAA && byte2 == 0x55) {
            // 摇杆帧需要 14 字节，不足则退出等下一轮
            if (FIFO_Count(rx_fifo) < sizeof(JoystickFrame_t)) break;
            
            uint8_t frame_buf[sizeof(JoystickFrame_t)];
            uint16_t p = t_head;
            
            // 复制疑似一帧的所有数据
            for(int i = 0; i < sizeof(JoystickFrame_t); i++) {
                frame_buf[i] = rx_fifo.buffer[p];
                p = (p + 1) % RING_BUF_SIZE;
            }
            
            JoystickFrame_t* pFrame = (JoystickFrame_t*)frame_buf;
            
            // 验证帧尾是否对应 0xDE
            if (pFrame->tail == 0xDE && pFrame->crc == crc8(frame_buf+2, sizeof(JoystickFrame_t) - 4)) {
                // 提取解包好的摇杆数据 (均为 16位 uint16_t 数据)
                rec_joystick[0] = pFrame->ch1;
                rec_joystick[1] = pFrame->ch2;
                rec_joystick[2] = pFrame->ch3;
                rec_joystick[3] = pFrame->ch4;
                rec_send_key = pFrame->key;
                rec_page = pFrame->page;

                // 将 FIFO 头部读取指针越过已经正确消费的这一帧
                rx_fifo.head = p;
                rx_cnt++;
                data_updated = true; // 标记数据已更新
            } else {
                // 坏帧，跳过头部第一个错误字节，继续往后寻找
                rx_fifo.head = (rx_fifo.head + 1) % RING_BUF_SIZE;
            }
        }
        // 查找帧头 0xAA 0x66 (设置帧：KFS位置)
        else if (byte1 == 0xAA && byte2 == 0x66) {
            uint8_t frame_buf[sizeof(SettingFrame_t)];
            uint16_t p = t_head;
            
            // 复制疑似一帧的所有数据
            for(int i = 0; i < sizeof(SettingFrame_t); i++) {
                frame_buf[i] = rx_fifo.buffer[p];
                p = (p + 1) % RING_BUF_SIZE;
            }
            
            SettingFrame_t* pFrame = (SettingFrame_t*)frame_buf;
            
            // 验证帧尾 0xDE 与 CRC（CRC 覆盖 command + load1 + load2 共 3 字节）
            if (pFrame->tail == 0xDE && pFrame->crc == crc8(frame_buf+2, sizeof(SettingFrame_t) - 4)) {
                // 提取解包好的设置数据
                switch(pFrame->command) {
                    case 0x04: // KFS1位置设置
                        rec_setting_command = pFrame->command;
                        rec_setting_load1 = pFrame->load1;
                        rec_setting_load2 = pFrame->load2;

                        rec_KFS1_place1 = pFrame->load1 & 0x0F;
                        rec_KFS1_place2 = (pFrame->load1 >> 4) & 0x0F;
                        rec_KFS1_place3 = pFrame->load2 & 0x0F;
                        break;
                    
                    case 0x07: // KFS2位置设置
                        rec_setting_command = pFrame->command;
                        rec_setting_load1 = pFrame->load1;
                        rec_setting_load2 = pFrame->load2;

                        rec_KFS2_place1 = pFrame->load1 & 0x0F;
                        rec_KFS2_place2 = (pFrame->load1 >> 4) & 0x0F;
                        rec_KFS2_place3 = pFrame->load2 & 0x0F;
                        rec_KFS2_place4 = (pFrame->load2 >> 4) & 0x0F;
                        break;                    

                    case 0x08: // KFSf位置设置
                        rec_setting_command = pFrame->command;
                        rec_setting_load1 = pFrame->load1;
                        rec_setting_load2 = pFrame->load2;

                        rec_KFSf_place1 = pFrame->load1 & 0x0F;
                        break;

                    default:
                        // 其他KFS命令暂不处理
                        break;
                }

                // 将 FIFO 头部读取指针越过已经正确消费的这一帧
                rx_fifo.head = p;
                rx_cnt++;
                data_updated = true; // 标记数据已更新
            } else {
                // 坏帧，跳过头部第一个错误字节，继续往后寻找
                rx_fifo.head = (rx_fifo.head + 1) % RING_BUF_SIZE;
            }
        }
        // 查找帧头 0xAA 0x77 (命令帧：串口屏转发)
        else if (byte1 == 0xAA && byte2 == 0x77) {
            uint8_t frame_buf[sizeof(CommandFrame_t)];
            uint16_t p = t_head;

            for(int i = 0; i < sizeof(CommandFrame_t); i++) {
                frame_buf[i] = rx_fifo.buffer[p];
                p = (p + 1) % RING_BUF_SIZE;
            }

            CommandFrame_t* pCmdFrame = (CommandFrame_t*)frame_buf;

            if (pCmdFrame->tail == 0xDE && pCmdFrame->crc == crc8(frame_buf+2, sizeof(CommandFrame_t) - 4)) {
                rec_command_command = pCmdFrame->command;
                rec_command_load1 = pCmdFrame->load1;
                rec_command_load2 = pCmdFrame->load2;

                rx_fifo.head = p;
                rx_cnt++;
                data_updated = true;
            } else {
                rx_fifo.head = (rx_fifo.head + 1) % RING_BUF_SIZE;
            }
        }
        else {
                // 没有找到帧头，抛弃头部第一字节，继续循环寻头
                rx_fifo.head = (rx_fifo.head + 1) % RING_BUF_SIZE;
            }
        }        
        return data_updated;
    }

    void Communication::Comm_SendAxisDataToTxBuffer(uint16_t  x, uint16_t y, uint16_t z,uint8_t Gripper_Status, uint8_t Suction_Cup_Status,uint8_t Automatic_status, uint8_t mode, uint8_t command1, uint8_t command2,
            uint8_t KFS_want_place1, uint8_t KFS_want_place2, uint8_t spear, uint8_t KFS_Keepplace)
    {
        if(tx_busy==0) {
            send_xyz[0] = x;
            send_xyz[1] = y;
            send_xyz[2] = z;
            send_status = (Gripper_Status << 3) | (Suction_Cup_Status << 1) | Automatic_status;
            send_mode = mode;
            send_command1 = command1;
            send_command2 = command2;
            send_KFS_want_place1 = KFS_want_place1;
            send_KFS_want_place2 = KFS_want_place2;
            send_spear = spear;
            send_KFS_Keepplace = KFS_Keepplace;

                XYZFrame_t frame;
            frame.header[0] = 0x55;
            frame.header[1] = 0xAA;
            // 把 3 个16位的坐标数据数据打包
            frame.x = send_xyz[0];
            frame.y = send_xyz[1];
            frame.z = send_xyz[2];
            frame.status = send_status;
            frame.mode = send_mode;
            frame.command1 = send_command1;
            frame.command2 = send_command2;
            // 填充扩展的KFS字段（与遥控器端 XYZFrame_t 对齐）
            frame.KFS_want_place1 = send_KFS_want_place1;
            frame.KFS_want_place2 = send_KFS_want_place2;
            frame.spear = send_spear;
            frame.KFS_Keepplace = send_KFS_Keepplace;
            frame.crc = crc8((uint8_t*)&frame + 2, sizeof(XYZFrame_t) - 4);
            frame.tail = 0xED;

            uint8_t* ptr = (uint8_t*)&frame;
            for (int i = 0; i < sizeof(XYZFrame_t); i++) {
                FIFO_Push(tx_fifo, ptr[i]);
            }
            
            // 尝试拉起发送：如果底部DMA空闲，且队列里有东西
            if (tx_busy == 0 && FIFO_Count(tx_fifo) > 0) {
                if(HAL_GPIO_ReadPin(this->tx_aux_port, this->tx_aux_pin) == GPIO_PIN_SET)
                {
                    Comm_TxBufferToTxDMA(txhuart); 
                }
            }
        }
        
    }

    void Communication::Comm_SendAnyDataToTxBuffer(const uint8_t* data, uint16_t len)
    {
        if (data == NULL || len == 0) return;

        // __disable_irq(); 
        for (uint16_t i = 0; i < len; i++) {
            FIFO_Push(tx_fifo, data[i]);
        }
        // __enable_irq();

        if (tx_busy == 0 && FIFO_Count(tx_fifo) > 0) {
            if(HAL_GPIO_ReadPin(this->tx_aux_port, this->tx_aux_pin) == GPIO_PIN_SET) {
                Comm_TxBufferToTxDMA(this->txhuart); 
            }
        }
    }

    void Communication::Comm_TxBufferToTxDMA(UART_HandleTypeDef *txhuart_param)
    {
        if (this->txhuart == txhuart_param) {
            // 安全保护：如果在忙碌期间误入或对方为低电平（繁忙），直接退出
            if (HAL_GPIO_ReadPin(this->tx_aux_port, this->tx_aux_pin) == GPIO_PIN_RESET) {
                return;
            }

            uint16_t count = FIFO_Count(tx_fifo);
            if (count > 0) {
                if (count > DMA_BUF_SIZE) count = DMA_BUF_SIZE;
                
                // 将欲发送的队列数据腾出到 DMA 使用的固定线性数组中
                for (uint16_t i = 0; i < count; i++) {
                    FIFO_Pop(tx_fifo, dma_tx_buf[i]);
                }
                
                // 【新增】将 D-Cache 里的数据主动推入 SRAM 供 DMA 读取
                // 注意这里要保证清除完整的 count 长度，如果你传入的是部分长度可能没刷新干净
                SCB_CleanDCache_by_Addr((uint32_t*)dma_tx_buf, count);
                
                tx_busy = 1; // 锁定发送状态
                Comm_TxUseTxDMA(txhuart, dma_tx_buf, count);
                tx_cnt++;
            } else {
                // 如果且仅如果队列已经为空了，清除忙碌标志位
                tx_busy = 0;
            }
        }
    }


    void Communication::Comm_RxDMAToRxBuffer(UART_HandleTypeDef *rxhuart_param, uint16_t size)
    {
        if (this->rxhuart == rxhuart_param) {
            // 注意：SCB_InvalidateDCache 和 HAL_UARTEx_ReceiveToIdle_DMA
            // 已由 RC_serial.cpp 的 All_Uart_Rx_It_Process 统一处理，
            // 此处重复调用会导致 DMA 连续重启两次，中间窗口期可能丢数据

            // DMA 中断来了，将其固定缓存段里收到的数据复制到业务层的 RX 环形缓冲区中
            for (uint16_t i = 0; i < size; i++) {
                FIFO_Push(rx_fifo, dma_rx_buf[i]);
            }
        }
    }
}