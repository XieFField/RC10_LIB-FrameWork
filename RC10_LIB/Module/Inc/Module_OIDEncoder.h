#include "BSP_CanFrame.h"
#include <cstdint>
#include <cstddef>

class fdCANbus; // 前置声明


class OIDEncoder {
public:
    OIDEncoder(uint32_t id, bool isExt, fdCANbus* bus)
    {
        device_id_ = id;
        isExtended_ = isExt;
        bus_ = bus;
    }
    ~OIDEncoder() = default;

    /**
     * 发送： 0x04（数据长度）+0x01（编码器地址）+0x01（指令码）+0x00（数据1）
     * 接收：0X07（数据长度）+0X01（编码器地址）+0X01（指令码）+0x00012345（数据）
     */

    void 
private:
    uint32_t device_id_;
    bool isExtended_;
    fdCANbus* bus_;
};