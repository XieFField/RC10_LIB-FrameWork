#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "APP_Utils.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;

constexpr float degToRadF32(float deg)
{
    return deg * (2.0f * kPi) / 360.0f;
}

int g_failures = 0;

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

void expectTrue(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::printf("FAIL line %d: %s\n", line, expression);
        ++g_failures;
    }
}

#define EXPECT_NEAR(actual, expected, tolerance) expectNear((actual), (expected), (tolerance), #actual, __LINE__)
#define EXPECT_TRUE(expr) expectTrue((expr), #expr, __LINE__)

#if JIA_APP_MATH_MODE == JIA_APP_MATH_MODE_STD
constexpr const char *kBackendName = "STD";
#else
constexpr const char *kBackendName = "DSP";
#endif

void testBackendTrigonometryMatchesHostReference()
{
    const float angles_rad[] = {
        0.0f,
        0.25f * kPi,
        -0.5f * kPi,
        kPi,
    };

    for (float angle_rad : angles_rad)
    {
        EXPECT_NEAR(jia::sinRadF32(angle_rad), std::sin(angle_rad), 1.0e-6f);
        EXPECT_NEAR(jia::cosRadF32(angle_rad), std::cos(angle_rad), 1.0e-6f);
    }
}

void testBackendMagnitudeAndSqrtSemanticsStayStable()
{
    EXPECT_NEAR(jia::magnitude2DF32(3.0f, 4.0f), 5.0f, 1.0e-6f);
    EXPECT_NEAR(jia::magnitude2DF32(0.0f, 0.0f), 0.0f, 1.0e-6f);
    EXPECT_NEAR(jia::sqrtF32(9.0f), 3.0f, 1.0e-6f);
    EXPECT_NEAR(jia::sqrtF32(0.0f), 0.0f, 1.0e-6f);
    EXPECT_NEAR(jia::sqrtF32(-1.0f), 0.0f, 1.0e-6f);
}

void testAngleWrappingAndNearestEquivalentSemantics()
{
    EXPECT_NEAR(jia::wrapToPiF32(kPi), -kPi, 1.0e-6f);
    EXPECT_NEAR(jia::wrapToPiF32(-kPi), -kPi, 1.0e-6f);
    EXPECT_NEAR(jia::wrapToPiF32(1.5f * kPi), -0.5f * kPi, 1.0e-6f);
    EXPECT_NEAR(jia::wrapTo2PiF32(2.0f * kPi), 0.0f, 1.0e-6f);
    EXPECT_NEAR(jia::wrapTo2PiF32(-0.5f * kPi), 1.5f * kPi, 1.0e-6f);

    const float from_rad = degToRadF32(170.0f);
    const float to_rad = degToRadF32(-170.0f);
    EXPECT_NEAR(jia::shortestAngularDistanceF32(from_rad, to_rad), degToRadF32(20.0f), 1.0e-6f);
    EXPECT_NEAR(jia::shortestAngularDistanceF32(to_rad, from_rad), degToRadF32(-20.0f), 1.0e-6f);
    EXPECT_NEAR(jia::shortestAngularDistanceF32(0.0f, kPi), -kPi, 1.0e-6f);

    const float current_rad = degToRadF32(350.0f);
    const float target_mod_rad = degToRadF32(10.0f);
    const float nearest_rad = jia::nearestEquivalentAngleF32(current_rad, target_mod_rad);

    EXPECT_NEAR(nearest_rad, degToRadF32(370.0f), 1.0e-6f);
    EXPECT_NEAR(jia::wrapTo2PiF32(nearest_rad), jia::wrapTo2PiF32(target_mod_rad), 1.0e-6f);
    EXPECT_NEAR(nearest_rad - current_rad, jia::shortestAngularDistanceF32(current_rad, target_mod_rad), 1.0e-6f);

    const float mirrored_current_rad = degToRadF32(10.0f);
    const float mirrored_target_mod_rad = degToRadF32(350.0f);
    const float mirrored_nearest_rad = jia::nearestEquivalentAngleF32(mirrored_current_rad, mirrored_target_mod_rad);

    EXPECT_NEAR(mirrored_nearest_rad, degToRadF32(-10.0f), 1.0e-6f);
    EXPECT_NEAR(jia::wrapTo2PiF32(mirrored_nearest_rad), jia::wrapTo2PiF32(mirrored_target_mod_rad), 1.0e-6f);
}
} // namespace

int main()
{
    std::printf("app_utils_backend test: backend=%s\n", kBackendName);

    testBackendTrigonometryMatchesHostReference();
    testBackendMagnitudeAndSqrtSemanticsStayStable();
    testAngleWrappingAndNearestEquivalentSemantics();

    if (g_failures != 0)
    {
        std::printf("app_utils_backend test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("app_utils_backend test: PASS");
    return EXIT_SUCCESS;
}
