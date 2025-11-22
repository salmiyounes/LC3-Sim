#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Debug trace macro
#ifdef TRACE
#define DEBUG_TRACE(...) fprintf(stderr, __VA_ARGS__)
#else
#define DEBUG_TRACE(...)
#endif

// Macros
#define TRAP_PUTS(vm)                                                          \
  do {                                                                         \
    lc3_addr *c = (vm)->memory + REG((vm), R_R0);                              \
    while (*(c)) {                                                             \
      putc((char)*(c), stdout);                                                \
      ++(c);                                                                   \
    }                                                                          \
    fflush(stdout);                                                            \
  } while (0)

// Field access macros; "i" is an instruction
#define F_DR(i) (((i) >> 9) & 0x7)

#define F_VECT8(i) ((i) & 0xFF)

#define F_SR1(i) (((i) >> 6) & 0x7)

#define F_SR2(i) (((i) >> 0) & 0x7)

#define F_imm5(i) sextend(i, 5)

#define F_BaseR(i) F_SR1(i)

#define REG_PC(vm) (vm)->reg[(R_PC)]

#define REG(vm, dr) (vm)->reg[(dr)]

// Types
typedef uint16_t lc3_word;
typedef uint16_t lc3_addr;
typedef uint16_t lc3_reg;

enum {
  MAX_MEM_SIZE = 1 << 16, // Memory address space 2 ** 16
  SIGN_FLAG_BIT = 1 << 15
};

typedef enum {
  VM_OPCODE_ADD = 0b0001,
  VM_OPCODE_AND = 0b0101,
  VM_OPCODE_BR = 0b0000,
  VM_OPCODE_JMP = 0b1100,
  VM_OPCODE_JSR = 0b0100,
  VM_OPCODE_LD = 0b0010,
  VM_OPCODE_LDI = 0b1010,
  VM_OPCODE_LDR = 0b0110,
  VM_OPCODE_LEA = 0b1110,
  VM_OPCODE_NOT = 0b1001,
  VM_OPCODE_RTI = 0b1000,
  VM_OPCODE_ST = 0b0011,
  VM_OPCODE_STI = 0b1011,
  VM_OPCODE_STR = 0b0111,
  VM_OPCODE_TRAP = 0b1111,
  VM_OPCODE_RESERVED = 0b1101,
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

enum { OUT = 0x21, PUTS = 0X22, HALT = 0x25 } trap_vect;

struct lc3_vm {
  volatile bool should_halt;
  lc3_word memory[MAX_MEM_SIZE];
  lc3_addr reg[R_COUNT];
};

typedef struct lc3_vm *lc3_vm_p;

enum { SUCCESS_CODE = 0, ERROR_CODE };

typedef enum { RUN_SUCCESS = 0, RUN_UNHANDLED_OPCODE, RUN_FAIL } vm_run_result;

// Helper functions
uint16_t bswap16(uint16_t x) {
#if defined(__clang__) || defined(__GNUC__)
  return __builtin_bswap16(x);
#else
  return (x << 8) | (x >> 8);
#endif
}

uint16_t sextend(uint16_t x, uint16_t y) {
  uint16_t m = 1 << (y - 1);
  x &= ((1 << y) - 1);
  return (x ^ m) - m;
}

bool test_bit(uint16_t x, uint16_t b) { return (bool)(x & (1 << b)); }

bool sign_bit(uint16_t x) { return (bool)(x & SIGN_FLAG_BIT); }

bool should_stop(lc3_vm_p vm) { return vm->should_halt; }

lc3_word bytes_to_lc3_word(unsigned char buf[2]) {
  union {
    unsigned char bytes[2];
    lc3_word word;
  } word_union;

  memcpy(word_union.bytes, buf, 2);
  return bswap16(word_union.word);
}

// Create
lc3_vm_p new_lc3_vm() {
  lc3_vm_p vm = calloc(1, sizeof(struct lc3_vm));
  if (vm == NULL) {
    fprintf(stderr, "%s: could not allocate memory for 'vm' \n", __func__);
    exit(ERROR_CODE);
  }
  vm->should_halt = false;

  return vm;
}

void vm_destroy(lc3_vm_p vm) { free(vm); }

vm_condition_codes vm_sign_flag(uint16_t value) {
  if (value == 0)
    return FLAG_ZRO;
  else if (sign_bit(value))
    return FLAG_NEG;
  else
    return FLAG_POS;
}

void vm_setcc(lc3_vm_p vm, lc3_reg reg_index) {
  REG(vm, R_COND) = vm_sign_flag(REG(vm, reg_index));
}

// read/write from memory
void vm_write_memory(lc3_vm_p vm, lc3_word addr, lc3_word value) {
  switch (addr) {
  case 0xFE06: // DDR
    putchar(value);
    fflush(stdout);
    break;
  }
  vm->memory[addr] = value;
}

lc3_word vm_read_memory(lc3_vm_p vm, lc3_addr addr) { return vm->memory[addr]; }

// Lc3 fetch/exucution logic
vm_run_result vm_run_instr(lc3_vm_p vm, lc3_word instr) {
  DEBUG_TRACE("DEBUG %s instr %x REG_PC %x\n", __func__, instr, REG_PC(vm));

  switch ((vm_opcode)(instr >> 12)) {
  case VM_OPCODE_ADD: {
    lc3_reg dr = F_DR(instr);
    if (!test_bit(instr, 5)) {
      DEBUG_TRACE("VM_OPCODE_ADD dr %x sr1 %x sr2 %x\n", dr, F_SR1(instr),
                  F_SR2(instr));
      REG(vm, dr) = REG(vm, F_SR1(instr)) + REG(vm, F_SR2(instr));
    } else {
      uint16_t imm5 = F_imm5(instr);
      DEBUG_TRACE("VM_OPCODE_ADD dr %x sr1 %x imm5 %x\n", dr, F_SR1(instr),
                  imm5);
      REG(vm, dr) = REG(vm, F_SR1(instr)) + imm5; // DR = SR1 + SEXT(imm5);
    }
    vm_setcc(vm, dr);
    break;
  }
  case VM_OPCODE_AND: {
    lc3_reg dr = F_DR(instr);
    if (!test_bit(instr, 5)) {
      DEBUG_TRACE("VM_OPCODE_AND dr %x sr1 %x sr2 %x\n", dr, F_SR1(instr),
                  F_SR2(instr));
      REG(vm, dr) = REG(vm, F_SR1(instr)) & REG(vm, F_SR2(instr));
    } else {
      uint16_t imm5 = F_imm5(instr);
      DEBUG_TRACE("VM_OPCODE_AND dr %x sr1 %x imm5 %x\n", dr, F_SR1(instr),
                  imm5);
      REG(vm, dr) = REG(vm, F_SR1(instr)) & imm5; // DR = SR1 & SEXT(imm5);
    }
    vm_setcc(vm, dr);
    break;
  }
  case VM_OPCODE_LD: {
    // TODO: if computed address is in privileged memory AND PSR[15] == 1
    //  Initiate ACV exception
    lc3_reg dr = F_DR(instr);
    lc3_addr pc_offset9 = sextend(instr, 9);
    REG(vm, dr) = vm_read_memory(vm, REG_PC(vm) + pc_offset9);
    vm_setcc(vm, dr);
    break;
  }
  case VM_OPCODE_LEA: {
    lc3_reg dr = F_DR(instr);
    lc3_addr pc_offset9 = sextend(instr, 9);
    REG(vm, dr) = REG_PC(vm) + pc_offset9;
    vm_setcc(vm, dr);
    break;
  }
  case VM_OPCODE_TRAP: {
    switch (F_VECT8(instr)) {
    case PUTS:
      TRAP_PUTS(vm);
      break;
    case HALT:
      vm->should_halt = true;
      break;
    }
    break;
  }
  default:
    return RUN_UNHANDLED_OPCODE;
  }
  return RUN_SUCCESS;
}

vm_run_result vm_fetch_execute(lc3_vm_p vm) {
  // Fetch instruction, increment PC, and execute.
  lc3_word instr = vm_read_memory(vm, REG_PC(vm)++);
  return vm_run_instr(vm, instr);
}

void vm_run(lc3_vm_p vm) {
  while (!should_stop(vm)) {
    vm_run_result res = vm_fetch_execute(vm);
    if (res != RUN_SUCCESS)
      break;
  }
}

// Load object file
int load_obj_file(lc3_vm_p vm, const char *filename, lc3_word *startp,
                  lc3_word *endp) {
  FILE *f;
  lc3_word addr, start;
  unsigned char buf[2];

  if ((f = fopen(filename, "rb")) == NULL)
    return ERROR_CODE;

  if (fread(buf, 2, 1, f) != 1) {
    fclose(f);
    return ERROR_CODE;
  }

  addr = start = bytes_to_lc3_word(buf);

  while (fread(buf, 2, 1, f) == 1) {
    vm_write_memory(vm, addr, bytes_to_lc3_word(buf));
    addr = (addr + 1) & 0xFFFF;
  }

  REG_PC(vm) = *startp = start;
  *endp = addr;

  fclose(f);
  return SUCCESS_CODE;
}

int main(int argc, char **argv) {
  lc3_vm_p vm = new_lc3_vm();
  lc3_word start, end;

  if (argc < 2) {
    fprintf(stderr, "Usage: %s <objfile>\n", argv[0]);
    return ERROR_CODE;
  }

  if (load_obj_file(vm, argv[1], &start, &end) != SUCCESS_CODE) {
    fprintf(stderr, "Failed to load %s\n", argv[1]);
    return ERROR_CODE;
  }

  vm_run(vm);

  vm_destroy(vm);
  return SUCCESS_CODE;
}
