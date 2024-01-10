// See LICENSE for license details.

#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "fdt.h"

volatile uint32_t* uart;

#define ENV_START 0x10001000
#define MMIO_UNPRIV_ADDR 0xf0000000
#define EXT_REGS 5
#define UNPRIV_REGS 6
#define EP_REGS 3
#define PRINT_REGS 32
#define UNPRIV_REG_TIME 0x4
#define UNPRIV_REG_PRINT 0x5

typedef unsigned long long Reg;

static int buffering = 1;
static uint64_t last_putchar = 0;

static inline void write_unpriv_reg(unsigned int index, Reg val)
{
  *((volatile Reg*)MMIO_UNPRIV_ADDR + EXT_REGS + index) = val;
}

static inline Reg read_unpriv_reg(unsigned int index)
{
  return *((volatile Reg*)MMIO_UNPRIV_ADDR + EXT_REGS + index);
}

static inline int is_gem5()
{
  return *((uint64_t*)ENV_START) == 0;
}

static void tcu_puts(const char *str, size_t len)
{
  // make sure the string is aligned for the 8-byte accesses below
  static __attribute__((
    aligned(8))) char aligned_buf[PRINT_REGS * sizeof(Reg)];

  const char *aligned_str;
  size_t regCount;
  volatile Reg *buffer;
  const Reg *rstr, *end;

  len = len < PRINT_REGS * sizeof(Reg) - 1 ? len : PRINT_REGS * sizeof(Reg) - 1;

  aligned_str = str;
  if ((uintptr_t)aligned_str & 7) {
    memcpy(aligned_buf, str, len);
    aligned_str = aligned_buf;
  }

  regCount = EXT_REGS + UNPRIV_REGS;
  buffer = (volatile Reg *)MMIO_UNPRIV_ADDR + regCount;
  rstr = (const Reg *)(aligned_str);
  end = (const Reg *)(aligned_str + len);
  while (rstr < end) {
    *buffer = *rstr;
    buffer++;
    rstr++;
  }

  // limit the UDP packet rate a bit to avoid packet drops
  if(!is_gem5()) {
    while((read_unpriv_reg(UNPRIV_REG_TIME) - last_putchar) < 100000)
      ;
    last_putchar = read_unpriv_reg(UNPRIV_REG_TIME);
  }
  else {
    register size_t a0 asm("a0") = (size_t)str;
    register size_t a1 asm("a1") = len;
    register size_t a2 asm("a2") = 0;
    register size_t a3 asm("a3") = (size_t)"stdout";
    asm volatile (
        ".long 0x9E00007B"
        : : "r"(a0), "r"(a1), "r"(a2), "r"(a3) : "memory"
    );
  }

  write_unpriv_reg(UNPRIV_REG_PRINT, len);
  // wait until the print was carried out
  while (read_unpriv_reg(UNPRIV_REG_PRINT) != 0)
    ;
}

void tcu_putchar(uint8_t c)
{
  size_t regCount;
  volatile Reg *buffer;

  regCount = EXT_REGS + UNPRIV_REGS;
  buffer = (volatile Reg *)MMIO_UNPRIV_ADDR + regCount;
  *buffer = c;

  // limit the UDP packet rate a bit to avoid packet drops
  if(!is_gem5()) {
    while((read_unpriv_reg(UNPRIV_REG_TIME) - last_putchar) < 100000)
      ;
    last_putchar = read_unpriv_reg(UNPRIV_REG_TIME);
  }

  write_unpriv_reg(UNPRIV_REG_PRINT, 1);
  // wait until the print was carried out
  while(read_unpriv_reg(UNPRIV_REG_PRINT) != 0)
    ;
}

static void do_uart_putchar(uint8_t ch)
{
#ifdef TCU_PRINT
    tcu_putchar(ch);
#else
# ifdef __riscv_atomic
    int32_t r;
    do {
      __asm__ __volatile__ (
        "amoor.w %0, %2, %1\n"
        : "=r" (r), "+A" (uart[UART_REG_TXFIFO])
        : "r" (ch));
    } while (r < 0);
# else
    volatile uint32_t *tx = uart + UART_REG_TXFIFO;
    while ((int32_t)(*tx) < 0);
    *tx = ch;
# endif
#endif
}

void uart_putchar(uint8_t ch)
{
    static int is_new_line = 1;
    static int initialized = 0;
    static char prefix[32];
    static char buffer[256];
    static size_t bufpos = 0;
    if(!initialized) {
      uint64_t tile_id = ((uint64_t*)ENV_START)[1];
      int tile = tile_id & 0xFF;
      int chip = tile_id >> 8;
      // the 0 is hardcoded here due to the limitations of bbl's snprintf implementation; but we
      // mostly run it on the FPGA where we just have 8 tiles so that it should be fine.
      snprintf(prefix, sizeof(prefix), "(Lx@C%dT0%d) ", chip, tile);
      initialized = 1;
    }

    if(is_new_line) {
      const char *p = prefix;
      bufpos = 0;
      while(*p) {
        if(buffering)
          buffer[bufpos++] = *p;
        else
          do_uart_putchar(*p);
        p++;
      }
      is_new_line = 0;
    }

    if(buffering) {
      buffer[bufpos++] = ch;
      if(ch == '\n' || bufpos >= sizeof(buffer)) {
        tcu_puts(buffer, bufpos);
        bufpos = 0;
      }
    }
    else
      do_uart_putchar(ch);

    if(ch == '\n')
      is_new_line = 1;
}

int uart_getchar()
{
  int32_t ch = uart[UART_REG_RXFIFO];
  if (ch < 0) return -1;
  return ch;
}

struct uart_scan
{
  int compat;
  uint64_t reg;
};

static void uart_open(const struct fdt_scan_node *node, void *extra)
{
  struct uart_scan *scan = (struct uart_scan *)extra;
  memset(scan, 0, sizeof(*scan));
}

static void uart_prop(const struct fdt_scan_prop *prop, void *extra)
{
  struct uart_scan *scan = (struct uart_scan *)extra;
  if (!strcmp(prop->name, "compatible") && fdt_string_list_index(prop, "sifive,uart0") >= 0) {
    scan->compat = 1;
  } else if (!strcmp(prop->name, "reg")) {
    fdt_get_address(prop->node->parent, prop->value, &scan->reg);
  }
  if (!strcmp(prop->name, "nobuf"))
    buffering = 0;
}

static void uart_done(const struct fdt_scan_node *node, void *extra)
{
  struct uart_scan *scan = (struct uart_scan *)extra;
  if (!scan->compat || !scan->reg || uart) return;

  // Enable Rx/Tx channels
  uart = (void*)(uintptr_t)scan->reg;
  uart[UART_REG_TXCTRL] = UART_TXEN;
  uart[UART_REG_RXCTRL] = UART_RXEN;

  // set divisor for sifive uart
  if (scan->compat) {
    // core runs with 80 MHz and we use 115200 baud
    uint16_t div = ((80000000 + 115200 - 1) / 115200) - 1;
    uart[UART_REG_DIV] = div;
  }
}

void query_uart(uintptr_t fdt)
{
  struct fdt_cb cb;
  struct uart_scan scan;

  memset(&cb, 0, sizeof(cb));
  cb.open = uart_open;
  cb.prop = uart_prop;
  cb.done = uart_done;
  cb.extra = &scan;

  fdt_scan(fdt, &cb);
}
