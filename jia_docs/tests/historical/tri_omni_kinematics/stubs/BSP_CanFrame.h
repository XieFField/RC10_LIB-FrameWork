#ifndef TEST_BSP_CANFRAME_H
#define TEST_BSP_CANFRAME_H

#include <cstdint>

struct CanFrame
{
    uint32_t ID = 0;
    bool isextended = false;
    uint8_t DLC = 0;
    uint8_t data[8] = {0};
};

#endif
