#ifndef __uart_tune_h_
#define __uart_tune_h_
#include "headfile.h"

// ============================================================
// 取消注释以启用 UART3 实时 PID/速度 调试
// Uncomment to enable UART3 real-time PID/speed tuning
// ============================================================
#define USE_UART_TUNE

#ifdef USE_UART_TUNE

void uart_tune_init(uint32_t baud);
void uart_tune_process(void);
void uart_tune_rx_isr(uint8_t byte);

#endif // USE_UART_TUNE

#endif // __uart_tune_h_
