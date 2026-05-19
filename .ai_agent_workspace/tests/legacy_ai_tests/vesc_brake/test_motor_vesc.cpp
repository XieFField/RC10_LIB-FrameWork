#include <cstdint>
#include <cstdlib>
#include <cstdio>

#include "Motor_VESC.h"

namespace
{
int32_t decode_i32_be(const uint8_t data[8])
{
    return (static_cast<int32_t>(data[0]) << 24) |
           (static_cast<int32_t>(data[1]) << 16) |
           (static_cast<int32_t>(data[2]) << 8) |
           static_cast<int32_t>(data[3]);
}

bool expect(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}
} // namespace

int main()
{
    bool ok = true;
    VESC_Motor motor(101, nullptr, 21.0f);
    CanFrame frame{};

    motor.setTargetRPM(0.0f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_ERPM << 8) | 101U),
                 "setTargetRPM(0) 应继续发送 ERPM 命令，而不是隐式刹车命令");
    ok &= expect(decode_i32_be(frame.data) == 0,
                 "setTargetRPM(0) 的报文负载应为 0 eRPM");

    motor.setBrake(70000.0f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_CURRENT_BRAKE << 8) | 101U),
                 "setBrake() 应发送 CURRENT_BRAKE 命令");
    ok &= expect(decode_i32_be(frame.data) == 70000,
                 "setBrake(70000) 应保留完整刹车电流，不应被截断");

    motor.setTargetRPM(120.0f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_ERPM << 8) | 101U),
                 "非零 RPM 应发送 ERPM 命令");
    ok &= expect(decode_i32_be(frame.data) == 2520,
                 "120 RPM 在 21 极对下应转换为 2520 eRPM");

    if (!ok)
    {
        return EXIT_FAILURE;
    }

    std::puts("PASS");
    return EXIT_SUCCESS;
}
