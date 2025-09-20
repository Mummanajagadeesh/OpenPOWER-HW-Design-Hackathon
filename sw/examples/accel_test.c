// sw/examples/accel_test.c
// Small driver illustrating memory-mapped IO for the FIR peripheral.
// Adjust BASE to match final memory map (docs/design.md).
#include <stdint.h>

volatile uint32_t * const BASE = (uint32_t *)0x40000000;
#define REG(offset) (*(volatile uint32_t*)( (char*)BASE + (offset) ))

int main() {
  // write 4 coeffs (example)
  REG(0x10) = 1;
  REG(0x14) = 1;
  REG(0x18) = 1;
  REG(0x1C) = 1;

  // write input
  REG(0x04) = 123;

  // start
  REG(0x00) = 0x1;

  // poll
  while ((REG(0x0C) & 0x1) == 0) {;}

  uint32_t out = REG(0x08);
  // in real integration you'll print over serial or store result
  return (int)out;
}
