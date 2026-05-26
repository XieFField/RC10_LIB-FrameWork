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

CanFrame makeStatus1Frame(uint32_t motor_id, int32_t erpm, int16_t current_raw, int16_t duty_raw)
{
    CanFrame frame{};
    frame.ID = (CAN_PACKET_STATUS_1 << 8) | (motor_id & 0xFFU);
    frame.isextended = true;
    frame.DLC = 8;
    frame.data[0] = static_cast<uint8_t>((erpm >> 24) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((erpm >> 16) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((erpm >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(erpm & 0xFF);
    frame.data[4] = static_cast<uint8_t>((current_raw >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(current_raw & 0xFF);
    frame.data[6] = static_cast<uint8_t>((duty_raw >> 8) & 0xFF);
    frame.data[7] = static_cast<uint8_t>(duty_raw & 0xFF);
    return frame;
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
                 "native rpm mode should keep sending ERPM command for setTargetRPM(0)");
    ok &= expect(decode_i32_be(frame.data) == 0,
                 "native rpm mode setTargetRPM(0) payload should be 0 eRPM");

    motor.setBrake(70000.0f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_CURRENT_BRAKE << 8) | 101U),
                 "setBrake() should send CURRENT_BRAKE command");
    ok &= expect(decode_i32_be(frame.data) == 70000,
                 "setBrake(70000) should preserve full brake current payload");

    motor.setTargetRPM(120.0f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_ERPM << 8) | 101U),
                 "native non-zero rpm should send ERPM command");
    ok &= expect(decode_i32_be(frame.data) == 2520,
                 "120 RPM should convert to 2520 eRPM for 21 pole pairs");

    PID_Param_Config pid_params = {
        15.0f,
        0.0f,
        0.0f,
        0.0f,
        true,
        5000.0f,
        0.0f
    };

    motor.pid_init(pid_params, 0.0f);
    motor.setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
    motor.updateFeedback(makeStatus1Frame(101U, 2100, 0, 0));
    motor.setTargetRPM(120.0f);
    motor.update();
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_CURRENT << 8) | 101U),
                 "pid speed mode should send CURRENT command for setTargetRPM()");
    ok &= expect(decode_i32_be(frame.data) == 300,
                 "pid speed mode should output 300 mA for 20 RPM error with kp=15");

    motor.updateFeedback(makeStatus1Frame(101U, 0, 0, 0));
    motor.setTargetRPM(0.0f);
    motor.update();
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_CURRENT << 8) | 101U),
                 "pid speed mode setTargetRPM(0) should still use CURRENT command");
    ok &= expect(decode_i32_be(frame.data) == 0,
                 "pid speed mode setTargetRPM(0) should output 0 mA");

    motor.setDuty(0.25f);
    motor.packCommand(&frame, 1);
    ok &= expect(frame.ID == ((CAN_CMD_SET_DUTY << 8) | 101U),
                 "setDuty() should still override back to DUTY command after pid mode");

    if (!ok)
    {
        return EXIT_FAILURE;
    }

    std::puts("PASS");
    return EXIT_SUCCESS;
}
