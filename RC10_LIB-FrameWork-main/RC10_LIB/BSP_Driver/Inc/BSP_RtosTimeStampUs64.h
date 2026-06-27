/**
 * @file BSP_RtosTimeStampUs64.h
 * @author 桑叁年
 * @brief RTOS 微秒时间戳接口
 */

#ifndef BSP_RTOS_TIME_STAMP_US64_H_
#define BSP_RTOS_TIME_STAMP_US64_H_

#include <cstdint>

namespace jia {

class RtosTimeStampUs64 {
public:
    static std::uint64_t getTimeUs();

private:
    RtosTimeStampUs64() = delete;
};

}  // namespace jia

#endif  // BSP_RTOS_TIME_STAMP_US64_H_
