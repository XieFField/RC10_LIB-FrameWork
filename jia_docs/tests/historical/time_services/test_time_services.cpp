#include <cstdio>
#include <cstdlib>

#include "BSP_TimeDwt.h"
#include "BSP_TimeUs64.h"

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

void expectEqUnsigned(unsigned long long actual, unsigned long long expected, const char *expression, int line)
{
    if (actual != expected)
    {
        std::printf("FAIL line %d: %s actual=%llu expected=%llu\n",
                    line,
                    expression,
                    actual,
                    expected);
        ++g_failures;
    }
}

#define EXPECT_TRUE(expr) expectTrue((expr), #expr, __LINE__)
#define EXPECT_EQ(actual, expected) expectEqUnsigned((actual), (expected), #actual, __LINE__)

void testDwtConversions()
{
    jia::TimeDwt::Init(1000000U);
    EXPECT_EQ(jia::TimeDwt::CyclesToUs32(1234U), 1234U);
    EXPECT_EQ(jia::TimeDwt::GetElapsedCycles32(100U, 90U), static_cast<unsigned int>(90U - 100U));
    EXPECT_EQ(jia::TimeDwt::GetElapsedUs32(100U, 250U), 150U);

    jia::TimeDwt::Init(500000U);
    EXPECT_EQ(jia::TimeDwt::CyclesToUs32(10U), 10U);
}

void testTimeStampUs64Conversions()
{
    EXPECT_EQ(jia::TimeStampUs64::TicksToUs64(123U, 1000U), 123000ULL);
    EXPECT_EQ(jia::TimeStampUs64::ComposeTimeUs64(12U, 1000U, 345U), 12345ULL);
    EXPECT_EQ(jia::TimeStampUs64::ComposeTimeUs64(12U, 1000U, 2000U), 13000ULL);
    EXPECT_EQ(jia::TimeStampUs64::GetTimeUs(), 0ULL);
}
} // namespace

int main()
{
    testDwtConversions();
    testTimeStampUs64Conversions();

    if (g_failures != 0)
    {
        std::printf("time_services test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("time_services test: PASS");
    return EXIT_SUCCESS;
}
