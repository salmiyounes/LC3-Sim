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

// Log Macros
#define msg(...) fprintf(stdout, __VA_ARGS__)
#define err(...) fprintf(stderr, __VA_ARGS__)
#define die(...)                                                               \
  do {                                                                         \
    err(__VA_ARGS__);                                                          \
    exit(ERROR_CODE);                                                          \
  } while (0)

// Macros
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define UNUSED(x) (void)(x);
#define CONCAT(a, b) a##b

// Types
typedef uint16_t lc3_word;
typedef uint16_t lc3_addr;
typedef uint16_t lc3_reg;

// Enums
enum {
  MAX_MEM_SIZE = 0x10000, // Memory address space 2 ** 16
  SIGN_FLAG_BIT = 0x8000,
  PC_START_POSITION = 0x3000
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

#ifdef DEBUG_MODE
typedef enum {
  VM_IS_RUNNING = 0,
  VM_HIT_BREAKPOINT,
  VM_STOP_RUNNING
} vm_execution_status;
#endif

typedef enum { 
  OUT = 0x21, 
  PUTS = 0x22, 
  HALT = 0x25 
} trap_vect;

enum { 
  SUCCESS_CODE = 0, 
  ERROR_CODE 
};

typedef enum {
  RUN_SUCCESS = 0,
  RUN_UNHANDLED_OPCODE,
  RUN_BREAKPOINT_STOP,
  RUN_FAIL
} vm_run_result;

// Structures
struct lc3_vm {
  lc3_word memory[MAX_MEM_SIZE];
#ifdef DEBUG_MODE
  int ic;
  size_t br_count;
  bool breakpoints[MAX_MEM_SIZE];
  vm_execution_status status;
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

#ifdef DEBUG_MODE
static inline vm_execution_status get_status_code(lc3_vm_p vm) {
  return vm->status;
}
#endif

static inline int get_instruction_count(lc3_vm_p vm) {
#ifdef DEBUG_MODE
  return vm->ic;
#else
  UNUSED(vm);
  return -1;
#endif
}

static inline bool is_breakpoint_hit(lc3_vm_p vm) {
#ifdef DEBUG_MODE
  lc3_addr index = get_reg_val(vm, R_PC);
  return vm->breakpoints[index];
#else
  UNUSED(vm);
  return false;
#endif
}

static inline void set_breakpoint(lc3_vm_p vm, const uint16_t index) {
#ifdef DEBUG_MODE
  if (!vm->breakpoints[index]) {
    msg("Breakpoint set at 0x%04X\n", (uint16_t)index);
    vm->br_count++;
  }
  vm->breakpoints[index] = true;
#else
  UNUSED(vm);
  UNUSED(index);
#endif
}

#endif // TYPES_H