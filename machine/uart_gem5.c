// See LICENSE for license details.

#include <string.h>
#include "uart_gem5.h"
#include "fdt.h"

int uart_gem5;

void uart_gem5_putchar(uint8_t c)
{
    char str[] = {c};
    register size_t a0 asm("a0") = (size_t)str;
    register size_t a1 asm("a1") = 1;
    register size_t a2 asm("a2") = 0;
    register size_t a3 asm("a3") = (size_t)"stdout";
    asm volatile (
        ".long 0x9E00007B"
        : : "r"(a0), "r"(a1), "r"(a2), "r"(a3) : "memory"
    );
}

int uart_gem5_getchar()
{
    return -1;
}

struct uart_gem5_scan
{
    int compat;
};

static void uart_gem5_open(const struct fdt_scan_node *node, void *extra)
{
    struct uart_gem5_scan *scan = (struct uart_gem5_scan *)extra;
    memset(scan, 0, sizeof(*scan));
}

static void uart_gem5_prop(const struct fdt_scan_prop *prop, void *extra)
{
    struct uart_gem5_scan *scan = (struct uart_gem5_scan *)extra;
    if (!strcmp(prop->name, "compatible") &&
        (fdt_string_list_index(prop, "gem5,uart0") != -1)) {
        scan->compat = 1;
    }
}

static void uart_gem5_done(const struct fdt_scan_node *node, void *extra)
{
    struct uart_gem5_scan *scan = (struct uart_gem5_scan *)extra;
    if (!scan->compat || uart_gem5)
        return;

    uart_gem5 = 1;
}

void query_uart_gem5(uintptr_t fdt)
{
    struct fdt_cb cb;
    struct uart_gem5_scan scan;

    memset(&cb, 0, sizeof(cb));
    cb.open = uart_gem5_open;
    cb.prop = uart_gem5_prop;
    cb.done = uart_gem5_done;
    cb.extra = &scan;

    fdt_scan(fdt, &cb);
}
