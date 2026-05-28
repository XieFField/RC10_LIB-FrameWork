#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include "Module_ChassisOmni.h"

float TimeStamp::now_s_ = 0.0f;

namespace
{
int g_failures = 0;

void expectTrue(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::printf("FAIL line %d: %s\n", line, expression);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float tolerance, const char *expression, int line)
{
    if (std::fabs(actual - expected) > tolerance)
    {
        std::printf("FAIL line %d: %s actual=%f expected=%f tolerance=%f\n",
                    line,
                    expression,
                    static_cast<double>(actual),
                    static_cast<double>(expected),
                    static_cast<double>(tolerance));
        ++g_failures;
    }
}

#define EXPECT_TRUE(expr) expectTrue((expr), #expr, __LINE__)
#define EXPECT_NEAR(actual, expected, tolerance) expectNear((actual), (expected), (tolerance), #actual, __LINE__)

class FakeMotor : public Motor_Base
{
public:
    FakeMotor() : Motor_Base(0, false, nullptr) {}

    void setTargetRPM(float rpm_set) override
    {
        target_rpm_ = rpm_set;
        ++rpm_calls;
    }

    void setTargetCurrent(float current_set) override
    {
        target_current_ = current_set;
        ++current_calls;
    }

    std::size_t packCommand(CanFrame[], std::size_t) override
    {
        return 0;
    }

    void updateFeedback(const CanFrame &) override {}

    int rpm_calls = 0;
    int current_calls = 0;
};

void testBaseRejectsOutOfRangeWheelRegistration()
{
    Chassis_Omni<3> chassis(0.05f, 1000.0f, 0.2f);
    FakeMotor motor;
    EXPECT_TRUE(chassis.registerWheelMotor(0, &motor));
    EXPECT_TRUE(!chassis.registerWheelMotor(3, &motor));
    EXPECT_TRUE(!chassis.registerWheelMotor(255, &motor));
}

void testZeroModesOnlyTouchRegisteredMotors()
{
    Chassis_Omni<3> chassis(0.05f, 1000.0f, 0.2f);
    FakeMotor motor0;
    FakeMotor motor2;
    chassis.registerWheelMotor(0, &motor0);
    chassis.registerWheelMotor(2, &motor2);

    Robot_Twist target{};
    target.vx = 1.0f;

    chassis.set_ControlMode(CURRENT_ZERO_MODE);
    chassis.set_Target(target);
    EXPECT_TRUE(motor0.current_calls == 1);
    EXPECT_TRUE(motor2.current_calls == 1);
    EXPECT_NEAR(motor0.getTargetCurrent(), 0.0f, 0.0001f);
    EXPECT_NEAR(motor2.getTargetCurrent(), 0.0f, 0.0001f);

    chassis.set_ControlMode(SPEED_ZERO_MODE);
    chassis.set_Target(target);
    EXPECT_TRUE(motor0.rpm_calls == 1);
    EXPECT_TRUE(motor2.rpm_calls == 1);
    EXPECT_NEAR(motor0.getTargetRPM(), 0.0f, 0.0001f);
    EXPECT_NEAR(motor2.getTargetRPM(), 0.0f, 0.0001f);
}

void testRobotSpeedModeComputesAndAppliesOmniWheelRpm()
{
    Chassis_Omni<3>::init_config config{};
    config.wheel_radius = 0.05f;
    config.max_wheel_rpm = 1000.0f;
    config.wheels[0] = {0.0f, 0.0f, 0.2f};
    config.wheels[1] = {120.0f, -0.17320508f, -0.1f};
    config.wheels[2] = {240.0f, 0.17320508f, -0.1f};
    Chassis_Omni<3> chassis(config);

    FakeMotor motors[3];
    for (int i = 0; i < 3; ++i)
    {
        chassis.registerWheelMotor(static_cast<uint8_t>(i), &motors[i]);
    }

    Robot_Twist target{};
    target.vx = 1.0f;
    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    TimeStamp::setTestSeconds(0.001f);
    chassis.update();

    const float rpm_scale = 60.0f / (2.0f * PI * config.wheel_radius);
    EXPECT_NEAR(chassis.getWheelTargetRPM(0), 1.0f * rpm_scale, 0.01f);
    EXPECT_NEAR(chassis.getWheelTargetRPM(1), -0.5f * rpm_scale, 0.01f);
    EXPECT_NEAR(chassis.getWheelTargetRPM(2), -0.5f * rpm_scale, 0.01f);
    EXPECT_NEAR(motors[0].getTargetRPM(), chassis.getWheelTargetRPM(0), 0.01f);
    EXPECT_NEAR(motors[1].getTargetRPM(), chassis.getWheelTargetRPM(1), 0.01f);
    EXPECT_NEAR(motors[2].getTargetRPM(), chassis.getWheelTargetRPM(2), 0.01f);
}

void testRobotSpeedModeScalesAllWheelRpmWhenAnyWheelExceedsLimit()
{
    Chassis_Omni<3>::init_config config{};
    config.wheel_radius = 0.05f;
    config.max_wheel_rpm = 100.0f;
    config.wheels[0] = {0.0f, 0.0f, 0.2f};
    config.wheels[1] = {120.0f, -0.17320508f, -0.1f};
    config.wheels[2] = {240.0f, 0.17320508f, -0.1f};
    Chassis_Omni<3> chassis(config);

    FakeMotor motors[3];
    for (int i = 0; i < 3; ++i)
    {
        chassis.registerWheelMotor(static_cast<uint8_t>(i), &motors[i]);
    }

    Robot_Twist target{};
    target.vx = 10.0f;
    chassis.set_ControlMode(ROBOT_SPEED_MODE);
    chassis.set_Target(target);
    TimeStamp::setTestSeconds(0.002f);
    chassis.update();

    EXPECT_NEAR(chassis.getWheelTargetRPM(0), 100.0f, 0.01f);
    EXPECT_NEAR(chassis.getWheelTargetRPM(1), -50.0f, 0.01f);
    EXPECT_NEAR(chassis.getWheelTargetRPM(2), -50.0f, 0.01f);
}

void testZeroModesDoNotTouchMissingWheelSlotsOnFourWheelBase()
{
    Chassis_Omni<4> chassis(0.05f, 1000.0f, 0.2f);
    FakeMotor motor0;
    FakeMotor motor3;
    chassis.registerWheelMotor(0, &motor0);
    chassis.registerWheelMotor(3, &motor3);

    Robot_Twist target{};
    target.vx = 2.0f;

    chassis.set_ControlMode(CURRENT_ZERO_MODE);
    chassis.set_Target(target);
    EXPECT_TRUE(motor0.current_calls == 1);
    EXPECT_TRUE(motor3.current_calls == 1);
    EXPECT_NEAR(motor0.getTargetCurrent(), 0.0f, 0.0001f);
    EXPECT_NEAR(motor3.getTargetCurrent(), 0.0f, 0.0001f);

    chassis.set_ControlMode(SPEED_ZERO_MODE);
    chassis.set_Target(target);
    EXPECT_TRUE(motor0.rpm_calls == 1);
    EXPECT_TRUE(motor3.rpm_calls == 1);
    EXPECT_NEAR(motor0.getTargetRPM(), 0.0f, 0.0001f);
    EXPECT_NEAR(motor3.getTargetRPM(), 0.0f, 0.0001f);
}

} // namespace

int main()
{
    testBaseRejectsOutOfRangeWheelRegistration();
    testZeroModesOnlyTouchRegisteredMotors();
    testRobotSpeedModeComputesAndAppliesOmniWheelRpm();
    testRobotSpeedModeScalesAllWheelRpmWhenAnyWheelExceedsLimit();
    testZeroModesDoNotTouchMissingWheelSlotsOnFourWheelBase();

    if (g_failures != 0)
    {
        std::printf("chassis_module test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("chassis_module test: PASS");
    return EXIT_SUCCESS;
}
