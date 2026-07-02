#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "Motor_EL05.h"

namespace
{
bool expect(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

bool expectNear(float actual, float expected, float tolerance, const char *message)
{
    if (std::fabs(actual - expected) > tolerance)
    {
        std::fprintf(stderr, "FAIL: %s (actual=%f expected=%f tolerance=%f)\n",
                     message,
                     static_cast<double>(actual),
                     static_cast<double>(expected),
                     static_cast<double>(tolerance));
        return false;
    }
    return true;
}

uint16_t floatToUint16(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }
    if (value > max_value)
    {
        value = max_value;
    }
    return static_cast<uint16_t>((value - min_value) * 65535.0f / (max_value - min_value));
}

CanFrame makeType2FeedbackFrame(uint8_t motor_id,
                                uint8_t fault_code,
                                uint8_t mode_state,
                                float position_rad,
                                float velocity_rad_s,
                                float torque_nm,
                                float temperature_c)
{
    CanFrame frame{};
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = (static_cast<uint32_t>(2U) << 24) |
               ((static_cast<uint32_t>(fault_code & 0x3FU) << 8) |
                (static_cast<uint32_t>(mode_state & 0x03U) << 14) |
                static_cast<uint32_t>(motor_id))
               << 8;

    const uint16_t position_raw = floatToUint16(position_rad, -12.57f, 12.57f);
    const uint16_t velocity_raw = floatToUint16(velocity_rad_s, -50.0f, 50.0f);
    const uint16_t torque_raw = floatToUint16(torque_nm, -6.0f, 6.0f);
    const uint16_t temperature_raw = static_cast<uint16_t>(temperature_c * 10.0f);

    frame.data[0] = static_cast<uint8_t>((position_raw >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(position_raw & 0xFF);
    frame.data[2] = static_cast<uint8_t>((velocity_raw >> 8) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(velocity_raw & 0xFF);
    frame.data[4] = static_cast<uint8_t>((torque_raw >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(torque_raw & 0xFF);
    frame.data[6] = static_cast<uint8_t>((temperature_raw >> 8) & 0xFF);
    frame.data[7] = static_cast<uint8_t>(temperature_raw & 0xFF);
    return frame;
}
} // namespace

int main()
{
    bool ok = true;
    EL05_Motor motor(0x01U, 0xFDU, nullptr);
    CanFrame frame{};

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "first startup frame should be emitted");
    ok &= expect(frame.isextended, "disable frame should use extended id");
    ok &= expect(frame.ID == ((4U << 24) | (0xFDU << 8) | 0x01U),
                 "first startup frame should be type 4 disable");

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "second startup frame should be emitted");
    ok &= expect(frame.ID == ((18U << 24) | (0xFDU << 8) | 0x01U),
                 "second startup frame should be type 18 write");
    ok &= expect(frame.data[0] == 0x05 && frame.data[1] == 0x70,
                 "run_mode write should target index 0x7005");
    ok &= expect(frame.data[4] == 0x00,
                 "run_mode write should set MIT/operation mode value 0");

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "third startup frame should be emitted");
    ok &= expect(frame.ID == ((18U << 24) | (0xFDU << 8) | 0x01U),
                 "third startup frame should be type 18 write");
    ok &= expect(frame.data[0] == 0x26 && frame.data[1] == 0x70,
                 "feedback period write should target index 0x7026");
    ok &= expect(frame.data[4] == 0x01,
                 "feedback period write should request 10ms interval");

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "fourth startup frame should be emitted");
    ok &= expect(frame.ID == ((24U << 24) | (0xFDU << 8) | 0x7FU),
                 "fourth startup frame should be type 24 active report enable");
    ok &= expect(frame.data[6] == 0x01,
                 "active report frame should enable periodic feedback");

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "fifth startup frame should be emitted");
    ok &= expect(frame.ID == ((6U << 24) | (0xFDU << 8) | 0x01U),
                 "fifth startup frame should be type 6 set zero");

    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "sixth startup frame should be emitted");
    ok &= expect(frame.ID == ((3U << 24) | (0xFDU << 8) | 0x01U),
                 "sixth startup frame should be type 3 enable");

    motor.setTargetAngle(90.0f);
    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "ready state should emit motion control frame");
    ok &= expect(frame.ID == ((1U << 24) | (32767U << 8) | 0x01U),
                 "motion control frame should encode zero torque into data area 2");
    ok &= expect(frame.data[4] == 0x0F && frame.data[5] == 0x5C,
                 "motion control frame should encode default kp=30");
    ok &= expect(frame.data[6] == 0x33 && frame.data[7] == 0x33,
                 "motion control frame should encode default kd=1");

    const CanFrame feedback = makeType2FeedbackFrame(0x01U, 0x05U, 0x02U, 1.0f, 10.0f, 2.0f, 36.5f);
    ok &= expect(motor.matchesFrame(feedback),
                 "motor should accept matching type 2 extended feedback");
    motor.updateFeedback(feedback);
    ok &= expectNear(motor.getAngle(), 57.2958f, 0.5f,
                     "feedback position should convert from rad to deg");
    ok &= expectNear(motor.getRPM(), 95.4929f, 0.8f,
                     "feedback velocity should convert from rad/s to rpm");
    ok &= expectNear(motor.getTorque(), 2.0f, 0.05f,
                     "feedback torque should decode from type 2 payload");

    motor.motorDisable();
    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "manual disable should emit one disable frame");
    ok &= expect(frame.ID == ((4U << 24) | (0xFDU << 8) | 0x01U),
                 "manual disable should use type 4");

    motor.motorEnable();
    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "manual enable should emit one enable frame");
    ok &= expect(frame.ID == ((3U << 24) | (0xFDU << 8) | 0x01U),
                 "manual enable should use type 3");

    motor.motorSetZero();
    ok &= expect(motor.packCommand(&frame, 1) == 1U,
                 "manual set zero should emit one set zero frame");
    ok &= expect(frame.ID == ((6U << 24) | (0xFDU << 8) | 0x01U),
                 "manual set zero should use type 6");

    if (!ok)
    {
        return EXIT_FAILURE;
    }

    std::puts("PASS");
    return EXIT_SUCCESS;
}
