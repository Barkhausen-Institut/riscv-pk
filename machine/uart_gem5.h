// See LICENSE for license details.

#ifndef _RISCV_GEM5_H
#define _RISCV_GEM5_H

#include <stdint.h>

extern int uart_gem5;

void uart_gem5_putchar(uint8_t ch);
int uart_gem5_getchar();
void query_uart_gem5(uintptr_t dtb);

#endif
