#include <cstddef>
#include <iostream>

#include "main.h"
#include "chassis.h"

int main()
{
    using Chassis = jia::FourSteerChassis::Chassis;

#if !defined(JIA_CHASSIS_PROFILE_RUNTIME_MIN) || !defined(JIA_CHASSIS_PROFILE_FULL_DEBUG)
#error "chassis.h must expose chassis profile macros for host slim smoke."
#endif

#if JIA_CHASSIS_PROFILE != JIA_CHASSIS_PROFILE_RUNTIME_MIN
#error "slim smoke must compile the runtime-min chassis profile."
#endif

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE || JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG || \
    JIA_CHASSIS_ENABLE_DEBUG_OUTPUT || \
    JIA_CHASSIS_ENABLE_PID_TUNE_CACHE || JIA_CHASSIS_ENABLE_DEBUG_MIRROR ||       \
    JIA_CHASSIS_ENABLE_TASK_PERF_STAT
#error "runtime-min profile must compile out optional debug and telemetry features."
#endif

    Chassis chassis{};
    const std::size_t size_bytes = sizeof(chassis);
    std::cout << "JIA_CHASSIS_PROFILE=" << JIA_CHASSIS_PROFILE << "\n";
    std::cout << "sizeof(Chassis)=" << size_bytes << "\n";

    // 4944 bytes 是瘦身前 host 侧 FULL_DEBUG 的参考基线。这里保留对齐和编译器差异余量，
    // 只要求 RUNTIME_MIN 明确砍掉大块调试缓存，避免正常功能的小幅扩展导致 smoke 误报。
    if (size_bytes >= 4200U)
    {
        std::cerr << "runtime-min chassis object is still too large: " << size_bytes << " bytes\n";
        return 1;
    }

    return 0;
}
