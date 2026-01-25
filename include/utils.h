#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdint.h>

static uint16_t bswap16(uint16_t x) {
#if defined(__clang__) || defined(__GNUC__)
  return __builtin_bswap16(x);
#else
  return (x << 8) | (x >> 8);
#endif
}

static uint16_t sextend(uint16_t x, uint16_t y) {
  uint16_t m = 1 << (y - 1);
  x &= ((1 << y) - 1);
  return (x ^ m) - m;
}

static bool test_bit(uint16_t x, uint16_t b) { return (bool)(x & (1 << b)); }

#endif // UTILS_H