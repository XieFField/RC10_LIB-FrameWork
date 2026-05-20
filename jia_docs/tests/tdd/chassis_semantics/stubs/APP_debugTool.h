#ifndef TEST_TDD_APP_DEBUG_TOOL_H
#define TEST_TDD_APP_DEBUG_TOOL_H

struct UART_HandleTypeDef
{
};

inline UART_HandleTypeDef huart8{};

class Debug_Printf
{
public:
    explicit Debug_Printf(UART_HandleTypeDef *) {}

    void printf_DMA(char *, ...) {}
    void printf_DMA_JustFloat(const float *, unsigned short) {}
    void printf_UART(char *, ...) {}
};

#endif
