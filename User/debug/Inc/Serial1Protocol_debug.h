#ifndef SERIAL1PROTOCOL_DEBUG_H
#define SERIAL1PROTOCOL_DEBUG_H

#ifdef __cplusplus
extern "C"{
}

#endif
#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "Module_Serial1Protocol.h"
#include "APP_debugTool.h"

class Serial1Protocol_Debug : public RtosTask
{
public:
    Serial1Protocol_Debug() : RtosTask("Serial1_Debug\0", 2) {}
    ~Serial1Protocol_Debug() {}

    void init(void)
    {
        // 获取 Serial1Protocol 实例并初始化
        m_serial1 = &Serial1Protocol::getInstance();
        m_serial1->init(&huart2);
        
        start(osPriorityNormal, 512);
    }

    // 手动发送测试数据
    void sendTestKFS(uint8_t kfs1, uint8_t kfs2, uint8_t kfs3)
    {
        if (m_serial1) {
            m_serial1->R1_Send_KFS(kfs1, kfs2, kfs3);
        }
    }

    void sendTestCMD(uint8_t cmd)
    {
        if (m_serial1) {
            m_serial1->send_cmd_to_R2(cmd);
        }
    }
Serial1Protocol* m_serial1 = nullptr;
protected:
    void loop() override;
    
private:
    
    
    // 测试计数器
    uint32_t m_test_counter = 0;
    uint32_t m_last_send_time = 0;
    
    // 接收数据统计
    uint32_t m_recv_kfs_count = 0;
    uint32_t m_recv_cmd_count = 0;
    uint32_t m_recv_error_count = 0;
    
    Debug_Printf debug_uart = Debug_Printf(&huart8);  // 使用 uart8 输出调试信息
};

#endif

#endif // SERIAL1PROTOCOL_DEBUG_H