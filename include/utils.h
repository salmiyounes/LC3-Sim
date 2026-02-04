#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static char *_strdup(const char *src) {
  if (src == NULL)
    return NULL;

  size_t len = strlen(src) + 1;
  void *new_s = malloc(len);

  if (new_s == NULL)
    return NULL;

  return (char *)memcpy(new_s, src, len);
}

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