#pragma once

#include <cmath>
#include <cstdio>

namespace jia::tests
{
class FailureTracker
{
public:
    void expectTrue(bool condition, const char *expression, int line)
    {
        if (!condition)
        {
            std::printf("FAIL line %d: %s\n", line, expression);
            ++failures_;
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
            ++failures_;
        }
    }

    int failures() const
    {
        return failures_;
    }

private:
    int failures_ = 0;
};
} // namespace jia::tests

#define JIA_TEST_EXPECT_TRUE(tracker, expr) (tracker).expectTrue((expr), #expr, __LINE__)
#define JIA_TEST_EXPECT_NEAR(tracker, actual, expected, tolerance) \
    (tracker).expectNear((actual), (expected), (tolerance), #actual, __LINE__)
