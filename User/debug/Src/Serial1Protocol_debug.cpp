#include "Serial1Protocol_Debug.h"

void Serial1Protocol_Debug::loop()
{
    static uint32_t last_print_time = 0;
    uint32_t now = HAL_GetTick();
    
    // ========== 1. 处理协议层（必须调用） ==========
//    if (m_serial1) {
//        m_serial1->process();  // 处理接收和发送
//    }
    
    // ========== 2. 接收数据测试 ==========
    DataPacket_t packet;
    if (m_serial1 && m_serial1->getLatestData(&packet)) 
			{
            if (packet.type == DATA_TYPE_KFS) 
						{
                // 处理 KFS 数据
                uint8_t k1 = packet.data.kfs[0];
                uint8_t k2 = packet.data.kfs[1];
                uint8_t k3 = packet.data.kfs[2];
            } 
						else 
						{
                // 处理 CMD
                uint8_t cmd = packet.data.cmd;
            }
        }
//     sendTestKFS(1, 2, 3);
				m_serial1->sendStop();

}