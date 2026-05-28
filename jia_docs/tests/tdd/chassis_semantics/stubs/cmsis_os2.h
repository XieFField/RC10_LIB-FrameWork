#ifndef TEST_TDD_CMSIS_OS2_H
#define TEST_TDD_CMSIS_OS2_H

#include <cstddef>

typedef void *osThreadId_t;
typedef int osPriority_t;

struct osThreadAttr_t
{
    const char *name = nullptr;
    std::size_t stack_size = 0;
    osPriority_t priority = 0;
};

constexpr osPriority_t osPriorityAboveNormal7 = 0;

inline osThreadId_t osThreadNew(void (*)(void *), void *, const osThreadAttr_t *)
{
    return reinterpret_cast<osThreadId_t>(1);
}

#endif
