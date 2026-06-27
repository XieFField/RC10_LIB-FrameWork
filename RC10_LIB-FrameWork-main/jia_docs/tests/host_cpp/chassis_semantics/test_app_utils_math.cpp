#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "APP_Utils.h"

namespace
{
constexpr float kTolerance = 1.0e-6f;
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

void testRadiansHelpersMatchBackend()
{
    const float angle_rad = jia::degToRadF32(30.0f);
    EXPECT_NEAR(jia::sinRadF32(angle_rad), 0.5f, kTolerance);
    EXPECT_NEAR(jia::cosRadF32(angle_rad), 0.8660254f, 1.0e-5f);
    EXPECT_NEAR(jia::sqrtF32(9.0f), 3.0f, kTolerance);
    EXPECT_NEAR(jia::sqrtF32(0.0f), 0.0f, kTolerance);
    EXPECT_NEAR(jia::sqrtF32(-1.0f), 0.0f, kTolerance);
    EXPECT_NEAR(jia::magnitude2DF32(3.0f, 4.0f), 5.0f, kTolerance);
}

void testDegreeHelpersStayDegreeBased()
{
    EXPECT_NEAR(jia::sinDegF32(30.0f), 0.5f, kTolerance);
    EXPECT_NEAR(jia::cosDegF32(60.0f), 0.5f, kTolerance);
    EXPECT_NEAR(jia::sinDegWrap360F32(-330.0f), 0.5f, kTolerance);
    EXPECT_NEAR(jia::cosDegWrap360F32(420.0f), 0.5f, kTolerance);
}

void testAngleWrappingHelpersStayStable()
{
    EXPECT_NEAR(jia::wrapToPiF32(jia::kPi), -jia::kPi, kTolerance);
    EXPECT_NEAR(jia::wrapTo2PiF32(-0.5f * jia::kPi), 1.5f * jia::kPi, kTolerance);
    EXPECT_NEAR(jia::shortestAngularDistanceF32(jia::degToRadF32(170.0f), jia::degToRadF32(-170.0f)),
                jia::degToRadF32(20.0f),
                kTolerance);
    EXPECT_NEAR(jia::nearestEquivalentAngleF32(jia::degToRadF32(350.0f), jia::degToRadF32(10.0f)),
                jia::degToRadF32(370.0f),
                kTolerance);
}

void testNormalizeAngleToPiUsesMinusPiToPiRange()
{
    EXPECT_NEAR(jia::normalizeAngleToPi(0.0f), 0.0f, kTolerance);
    EXPECT_NEAR(jia::normalizeAngleToPi(jia::kPi), -jia::kPi, kTolerance);
    EXPECT_NEAR(jia::normalizeAngleToPi(-jia::kPi), -jia::kPi, kTolerance);
    EXPECT_NEAR(jia::normalizeAngleToPi(1.5f * jia::kPi), -0.5f * jia::kPi, kTolerance);
    EXPECT_TRUE(jia::normalizeAngleToPi(0.75f * jia::kPi) < jia::kPi);
    EXPECT_TRUE(jia::normalizeAngleToPi(0.75f * jia::kPi) >= -jia::kPi);
}
} // namespace

int main()
{
    testRadiansHelpersMatchBackend();
    testDegreeHelpersStayDegreeBased();
    testAngleWrappingHelpersStayStable();
    testNormalizeAngleToPiUsesMinusPiToPiRange();

    if (g_failures != 0)
    {
        std::printf("app_utils_math test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("app_utils_math test: PASS");
    return EXIT_SUCCESS;
}
