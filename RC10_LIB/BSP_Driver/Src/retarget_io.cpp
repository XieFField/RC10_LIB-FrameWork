/**
 * @file retarget_io.cpp
 * @brief 重定向 C 库的输入输出到 UART（适用于 Arm Compiler）
 *        解决半主机模式下 _sys_open 未实现导致的断点问题。
 *        先前代碼無法正常運行，找了半天才找到的原因，特此記錄。
 * @author XieFField
 * 
 */

#include "stm32h7xx_hal.h"
#include <cstdio>
#include <cerrno>

// ！！！重要！！！
// 将 huart1 修改为你实际用于打印调试信息的 UART 句柄
// 可以任意指定，而且這個串口也不影響其餘的使用；
extern UART_HandleTypeDef huart1;

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
// Arm Compiler 6 (armclang)
__asm(".global __use_no_semihosting");
extern "C" __attribute__((used)) void __aeabi_assert(const char* expr, const char* file, int line) {
    (void)expr; (void)file; (void)line;
    while (1) { __NOP(); }
}
#else
// Arm Compiler 5 (armcc)
#pragma import(__use_no_semihosting)
#endif

extern "C" {

// FILEHANDLE 是 ARM C 库用于文件描述符的类型，通常是 int 的别名。
typedef int FILEHANDLE;

// --- 解决 _sys_open 问题的关键部分 ---
FILEHANDLE _sys_open(const char *name, int openmode) {
    (void)name;
    (void)openmode;
    return 1; // 返回一个虚拟句柄
}

// --- 最底层的 I/O 重定向 ---
int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned int len, int mode) {
    (void)fh;
    (void)mode;
    if (HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY) == HAL_OK) {
        return 0; // 返回 0 表示成功
    } else {
        return -1; // 返回错误
    }
}

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned int len, int mode) {
    (void)fh;
    (void)mode;
    if (HAL_UART_Receive(&huart1, buf, len, HAL_MAX_DELAY) == HAL_OK) {
        return len; // 返回成功读取的字节数
    } else {
        return -1; // 返回错误
    }
}

// --- 解决 _ttywrch 问题的关键部分 ---
/**
 * @brief 重定向 _ttywrch 函数
 * 用于处理无缓冲的单个字符输出。
 * @param ch 要发送的字符
 */
void _ttywrch(int ch) {
    char c = (char)ch;
    HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, HAL_MAX_DELAY);
}


// --- 其他必要的桩函数，防止链接半主机版本 ---

void _sys_exit(int return_code) {
    (void)return_code;
    while (1) { __NOP(); }
}

int _sys_close(FILEHANDLE fh) {
    (void)fh;
    return 0;
}

int _sys_istty(FILEHANDLE fh) {
    (void)fh;
    if (fh <= 2) {
        return 1;
    }
    return 0;
}

int _sys_seek(FILEHANDLE fh, long pos) {
    (void)fh;
    (void)pos;
    return -1;
}

long _sys_flen(FILEHANDLE fh) {
    (void)fh;
    return -1;
}

// C++ 纯虚函数和异常处理相关的桩
void __cxa_pure_virtual() { while (1) { __NOP(); } }
void __cxa_deleted_virtual() { while (1) { __NOP(); } }

} // extern "C"