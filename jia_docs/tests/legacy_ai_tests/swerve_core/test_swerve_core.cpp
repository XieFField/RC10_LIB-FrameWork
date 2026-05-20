#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Module_ChassisSwerve.h"

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

jia::swerve::SwerveConfig makeTestConfig()
{
    jia::swerve::SwerveConfig config = {};
    config.shared.wheel_radius_m = 0.1f;
    config.shared.max_drive_omega_rad_s = 100.0f;
    config.shared.max_drive_alpha_rad_s2 = 1000.0f;
    config.shared.max_steer_rate_rad_s = 100.0f;
    config.shared.max_steer_alpha_rad_s2 = 1000.0f;
    config.shared.stationary_speed_epsilon_m_s = 0.001f;
    config.shared.enable_cosine_compensation = false;

    config.modules[0].geometry = {0.25f, 0.25f, 0.0f, 1.0f, 1.0f};
    config.modules[1].geometry = {0.25f, -0.25f, 0.0f, 1.0f, 1.0f};
    config.modules[2].geometry = {-0.25f, 0.25f, 0.0f, 1.0f, 1.0f};
    config.modules[3].geometry = {-0.25f, -0.25f, 0.0f, 1.0f, 1.0f};

    for (std::size_t module_index = 0; module_index < jia::swerve::kModuleCount; ++module_index)
    {
        config.modules[module_index].homing.enabled = false;
    }

    return config;
}

void testAngleHelpers()
{
    EXPECT_NEAR(jia::swerve::wrapToPi(3.5f), 3.5f - 2.0f * jia::swerve::kPi, 1.0e-4f);
    EXPECT_NEAR(jia::swerve::wrapTo2Pi(-0.5f), 2.0f * jia::swerve::kPi - 0.5f, 1.0e-4f);
    EXPECT_NEAR(jia::swerve::shortestAngularDistance(0.0f, jia::swerve::kPi * 1.5f), -0.5f * jia::swerve::kPi, 1.0e-4f);
}

void testPureVx()
{
    const jia::swerve::SwerveConfig config = makeTestConfig();
    jia::swerve::SwerveController controller(config);
    jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};
    jia::swerve::ModuleCommand commands[jia::swerve::kModuleCount] = {};

    controller.step({1.0f, 0.0f, 0.0f}, feedback, 0.01f, commands);

    for (std::size_t module_index = 0; module_index < jia::swerve::kModuleCount; ++module_index)
    {
        EXPECT_NEAR(commands[module_index].raw_target_steer_angle_oa_rad, 0.0f, 1.0e-4f);
        EXPECT_NEAR(commands[module_index].selected_target_drive_omega_rad_s, 10.0f, 1.0e-4f);
    }
}

void testMinimalTurnFlip()
{
    const jia::swerve::SwerveConfig config = makeTestConfig();
    jia::swerve::SwerveController controller(config);
    jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};
    jia::swerve::ModuleCommand commands[jia::swerve::kModuleCount] = {};

    for (std::size_t module_index = 0; module_index < jia::swerve::kModuleCount; ++module_index)
    {
        feedback[module_index].steer_motor_total_angle_rad = jia::swerve::kPi - 0.05f;
    }

    controller.step({1.0f, 0.0f, 0.0f}, feedback, 0.01f, commands);

    EXPECT_TRUE(commands[0].flipped_drive_direction);
    EXPECT_TRUE(commands[0].selected_target_drive_omega_rad_s < 0.0f);
}

void testForwardKinematics()
{
    const jia::swerve::SwerveConfig config = makeTestConfig();
    const jia::swerve::ChassisCommand expected_motion{0.8f, -0.35f, 0.6f};
    jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};

    for (std::size_t module_index = 0; module_index < jia::swerve::kModuleCount; ++module_index)
    {
        const jia::swerve::WheelGeometry &geometry = config.modules[module_index].geometry;
        const float wheel_vx = expected_motion.vx_m_s - expected_motion.wz_rad_s * geometry.pos_y_m;
        const float wheel_vy = expected_motion.vy_m_s + expected_motion.wz_rad_s * geometry.pos_x_m;
        feedback[module_index].steer_motor_total_angle_rad = std::atan2(wheel_vy, wheel_vx);
        feedback[module_index].drive_omega_rad_s = std::sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy) /
                                                   config.shared.wheel_radius_m;
    }

    jia::swerve::ChassisCommand estimated_motion = {};
    const bool ok = jia::swerve::estimateChassisMotion(config, feedback, &estimated_motion);
    EXPECT_TRUE(ok);
    EXPECT_NEAR(estimated_motion.vx_m_s, expected_motion.vx_m_s, 1.0e-3f);
    EXPECT_NEAR(estimated_motion.vy_m_s, expected_motion.vy_m_s, 1.0e-3f);
    EXPECT_NEAR(estimated_motion.wz_rad_s, expected_motion.wz_rad_s, 1.0e-3f);
}

void testConfigureUpdatesConfigAndResetsActuatorHistory()
{
    jia::swerve::SwerveConfig config_a = makeTestConfig();
    config_a.shared.max_drive_alpha_rad_s2 = 1.0f;
    config_a.shared.max_steer_rate_rad_s = 100.0f;
    config_a.shared.max_steer_alpha_rad_s2 = 100.0f;

    jia::swerve::SwerveController controller(config_a);
    jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};
    jia::swerve::ModuleCommand commands[jia::swerve::kModuleCount] = {};

    controller.step({1.0f, 0.0f, 0.0f}, feedback, 0.1f, commands);
    EXPECT_NEAR(commands[0].selected_target_drive_omega_rad_s, 0.1f, 1.0e-4f);

    jia::swerve::SwerveConfig config_b = makeTestConfig();
    config_b.shared.wheel_radius_m = 0.2f;
    config_b.shared.max_drive_alpha_rad_s2 = 1.0f;
    controller.configure(config_b);

    EXPECT_NEAR(controller.config().shared.wheel_radius_m, 0.2f, 1.0e-6f);

    controller.step({1.0f, 0.0f, 0.0f}, feedback, 0.1f, commands);
    EXPECT_NEAR(commands[0].selected_target_drive_omega_rad_s, 0.1f, 1.0e-4f);
}
} // namespace

int main()
{
    testAngleHelpers();
    testPureVx();
    testMinimalTurnFlip();
    testForwardKinematics();
    testConfigureUpdatesConfigAndResetsActuatorHistory();

    if (g_failures != 0)
    {
        std::printf("swerve_core test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("swerve_core test: PASS");
    return EXIT_SUCCESS;
}
