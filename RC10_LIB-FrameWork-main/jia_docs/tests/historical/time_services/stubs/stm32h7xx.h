#ifndef TEST_STM32H7XX_H
#define TEST_STM32H7XX_H

#include <cstdint>

typedef struct
{
    volatile std::uint32_t CTRL = 0;
    volatile std::uint32_t CYCCNT = 0;
    volatile std::uint32_t CPICNT = 0;
    volatile std::uint32_t EXCCNT = 0;
    volatile std::uint32_t SLEEPCNT = 0;
    volatile std::uint32_t LSUCNT = 0;
    volatile std::uint32_t FOLDCNT = 0;
    volatile std::uint32_t PCSR = 0;
    volatile std::uint32_t RESERVED0[864] = {0};
} DWT_Type;

typedef struct
{
    volatile std::uint32_t DEMCR = 0;
} CoreDebug_Type;

typedef struct
{
    volatile std::uint32_t CTRL = 0;
    volatile std::uint32_t LOAD = 0;
    volatile std::uint32_t VAL = 0;
} SysTick_Type;

inline DWT_Type test_dwt_instance{};
inline CoreDebug_Type test_coredebug_instance{};
inline SysTick_Type test_systick_instance{};

#define DWT (&test_dwt_instance)
#define CoreDebug (&test_coredebug_instance)
#define SysTick (&test_systick_instance)

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#define SysTick_VAL_CURRENT_Msk 0x00FFFFFFUL
#define SysTick_LOAD_RELOAD_Msk 0x00FFFFFFUL
#define SysTick_CTRL_COUNTFLAG_Msk (1UL << 16)

#endif
