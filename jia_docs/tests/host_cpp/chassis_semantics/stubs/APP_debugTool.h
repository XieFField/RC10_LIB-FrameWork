#ifndef TEST_TDD_APP_DEBUG_TOOL_H
#define TEST_TDD_APP_DEBUG_TOOL_H

#include <cstddef>

struct UART_HandleTypeDef
{
};

inline UART_HandleTypeDef huart8{};

struct TestJustFloatCapture
{
    float values[64] = {0.0f};
    unsigned short size = 0U;
    bool called = false;
};

extern TestJustFloatCapture g_test_justfloat_capture;

inline void testHostResetJustFloatCapture()
{
    g_test_justfloat_capture = TestJustFloatCapture{};
}

class Debug_Printf
{
public:
    explicit Debug_Printf(UART_HandleTypeDef *) {}

    void printf_DMA(char *, ...) {}
    void printf_DMA_JustFloat(const float *data, unsigned short size)
    {
        testHostResetJustFloatCapture();
        g_test_justfloat_capture.called = true;
        g_test_justfloat_capture.size = size;
        const std::size_t copy_count = (size < 64U) ? static_cast<std::size_t>(size) : 64U;
        for (std::size_t i = 0; i < copy_count; ++i)
        {
            g_test_justfloat_capture.values[i] = data[i];
        }
    }
    void printf_UART(char *, ...) {}
};

#endif
