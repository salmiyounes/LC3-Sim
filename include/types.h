#ifndef TYPES_H
#define TYPES_H

#include "utils.h"
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>

// Debug trace macro
#ifdef TRACE
#define DEBUG_TRACE(...) fprintf(stderr, __VA_ARGS__)
#else
#define DEBUG_TRACE(...)
#endif

#define msg(...) fprintf(stdout, __VA_ARGS__)
#define err(...) fprintf(stderr, __VA_ARGS__)

// Macros
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))

// Types
typedef uint16_t lc3_word;
typedef uint16_t lc3_addr;
typedef uint16_t lc3_reg;

// Enums
enum {
  MAX_MEM_SIZE = 1 << 16, // Memory address space 2 ** 16
  SIGN_FLAG_BIT = 1 << 15
};

typedef enum {
  VM_OPCODE_ADD = 0x1,
  VM_OPCODE_AND = 0x5,
  VM_OPCODE_BR = 0x0,
  VM_OPCODE_JMP = 0xc,
  VM_OPCODE_JSR = 0x4,
  VM_OPCODE_LD = 0x2,
  VM_OPCODE_LDI = 0xa,
  VM_OPCODE_LDR = 0x6,
  VM_OPCODE_LEA = 0xe,
  VM_OPCODE_NOT = 0x9,
  VM_OPCODE_RTI = 0x8,
  VM_OPCODE_ST = 0x3,
  VM_OPCODE_STI = 0xb,
  VM_OPCODE_STR = 0x7,
  VM_OPCODE_TRAP = 0xf,
  VM_OPCODE_RESERVED = 0xd,
} vm_opcode;

enum {
  R_R0 = 0,
  R_R1,
  R_R2,
  R_R3,
  R_R4,
  R_R5,
  R_R6,
  R_R7,
  R_PC,
  R_COND,
  R_COUNT,
};

typedef enum {
  FLAG_POS = 1 << 0,
  FLAG_ZRO = 1 << 1,
  FLAG_NEG = 1 << 2,
} vm_condition_codes;

typedef enum { OUT = 0x21, PUTS = 0X22, HALT = 0x25 } trap_vect;

enum { SUCCESS_CODE = 0, ERROR_CODE };

typedef enum { RUN_SUCCESS = 0, RUN_UNHANDLED_OPCODE, RUN_FAIL } vm_run_result;

// Structures
struct lc3_vm {
  lc3_word memory[MAX_MEM_SIZE];
#ifdef DEBUG_MODE
  bool breakpoints[MAX_MEM_SIZE];
#endif
  lc3_addr reg[R_COUNT];
  jmp_buf buf;
  vm_run_result last_result;
};

struct lc3_vm;

typedef struct lc3_vm *lc3_vm_p;

// Field Access inline functions
static inline uint16_t f_dr(const uint16_t instr) { return (instr >> 9) & 0x7; }

static inline uint16_t f_vect8(const uint16_t instr) { return instr & 0xFF; }

static inline uint16_t f_sr1(const uint16_t instr) {
  return (instr >> 6) & 0x7;
}

static inline uint16_t f_sr2(const uint16_t instr) {
  return (instr >> 0) & 0x7;
}

static inline uint16_t f_sr(const uint16_t instr) { return f_sr1(instr); }

static inline uint16_t f_imm5(const uint16_t instr) {
  return sextend(instr, 5);
}

static inline uint16_t f_base_r(const uint16_t instr) { return f_sr1(instr); }

static inline lc3_addr get_reg_val(lc3_vm_p vm, const uint16_t reg_idx) {
  assert(reg_idx < R_COUNT);
  return vm->reg[reg_idx];
}

static inline lc3_addr *get_reg_ptr(lc3_vm_p vm, const uint16_t reg_idx) {
  assert(reg_idx < R_COUNT);
  return &vm->reg[reg_idx];
}

#endif // TYPES_H