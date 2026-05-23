#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "BSP_TimeStamp.h"
#include "APP_PID.h"
#include "Motor_DJI.h"

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

void advancePidTick()
{
    testPidReconnectAdvanceTimeSeconds(0.001f);
}

void testM3508SpeedPidReactivationProducesImmediateCurrentJumpAfterCurrentZeroHold()
{
    testPidReconnectResetTimeSeconds();

    M3508 steer_motor(1U, nullptr, false, false);
    steer_motor.pid_init(foursteer_steer_speed_pid_params, 0.0f, foursteer_steer_angle_pid_params, 0.0f);

    for (int i = 0; i < 8; ++i)
    {
        steer_motor.setTargetRPM(120.0f);
        advancePidTick();
        steer_motor.update();
    }

    const float current_before_disconnect = steer_motor.getTargetCurrent();
    EXPECT_TRUE(std::fabs(current_before_disconnect) > 1.0e-3f);

    steer_motor.setTargetCurrent(0.0f);
    advancePidTick();
    steer_motor.update();
    EXPECT_NEAR(steer_motor.getTargetCurrent(), 0.0f, 1.0e-6f);

    steer_motor.setTargetRPM(15.0f);
    advancePidTick();
    steer_motor.update();

    const float reconnect_first_current = steer_motor.getTargetCurrent();
    EXPECT_TRUE(std::fabs(reconnect_first_current) > 1.0e-3f);

    std::printf("pid_reconnect_current_before_disconnect=%f\n", static_cast<double>(current_before_disconnect));
    std::printf("pid_reconnect_first_current_after_reenable=%f\n", static_cast<double>(reconnect_first_current));
}

} // namespace

int main()
{
    testM3508SpeedPidReactivationProducesImmediateCurrentJumpAfterCurrentZeroHold();

    if (g_failures != 0)
    {
        std::printf("steer_pid_reconnect test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("steer_pid_reconnect test: PASS");
    return EXIT_SUCCESS;
}
