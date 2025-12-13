#include "CRSF_DEMO.h"


void crsf_demo::loop() 
{
    // 假设 crsf_test_ 已经指向一个有效的 CrsfReceiver 实例
    if (crsf_test_ != nullptr) {
        // 处理接收到的数据
        crsf_test_->process();
        // 获取控制数据
        crsf_test_->getControlData(&data1);

        // 更新变量 a，例如使用接收到的油门值
        a = data.throttle;  // 假设 data.throttle 是接收到的油门值
        // 可以添加更多的数据处理逻辑
    }

    // 其他任务逻辑...
}