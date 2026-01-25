#include "instructions.h"
#include "vm.h"
#include <setjmp.h>
#include <stdbool.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))

typedef void (*ins_handler_t)(lc3_vm_p vm, lc3_word instr);
typedef void (*trap_handler_t)(lc3_vm_p vm);

// Trap functions
void trap_puts(lc3_vm_p vm) {
  lc3_addr *c = vm->memory + get_reg_val(vm, R_R0);
  while (*c) {
    putc((char)*(c), stdout);
    ++c;
  }
}

void trap_out(lc3_vm_p vm) {
  fputc((char)get_reg_val(vm, R_R0) & 0XFF, stdout); // R0[7:0]
  fflush(stdout);
}

void trap_halt(lc3_vm_p vm) { longjmp(vm->buf, 1); }

static const trap_handler_t dispatch_trap_table[38] = {
    [PUTS] = trap_puts,
    [OUT] = trap_out,
    [HALT] = trap_halt,
};

// Opcode functions
void handle_add(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  if (!test_bit(instr, 5)) {
    DEBUG_TRACE("VM_OPCODE_ADD dr %x sr1 %x sr2 %x\n", dr, f_sr1(instr),
                f_sr2(instr));
    reg_write(vm, dr,
              get_reg_val(vm, f_sr1(instr)) + get_reg_val(vm, f_sr2(instr)),
              true);
  } else {
    uint16_t imm5 = f_imm5(instr);
    DEBUG_TRACE("VM_OPCODE_ADD dr %x sr1 %x imm5 %x\n", dr, f_sr1(instr), imm5);
    reg_write(vm, dr, get_reg_val(vm, f_sr1(instr)) + imm5,
              true); // DR = SR1 + SEXT(imm5);
  }
}

void handle_and(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  if (!test_bit(instr, 5)) {
    DEBUG_TRACE("VM_OPCODE_AND dr %x sr1 %x sr2 %x\n", dr, f_sr1(instr),
                f_sr2(instr));
    reg_write(vm, dr,
              get_reg_val(vm, f_sr1(instr)) & get_reg_val(vm, f_sr2(instr)),
              true);
  } else {
    uint16_t imm5 = f_imm5(instr);
    DEBUG_TRACE("VM_OPCODE_AND dr %x sr1 %x imm5 %x\n", dr, f_sr1(instr), imm5);
    reg_write(vm, dr, get_reg_val(vm, f_sr1(instr)) & imm5,
              true); // DR = SR1 & SEXT(imm5);
  }
}

void handle_br(lc3_vm_p vm, lc3_word instr) {
  lc3_addr pc_offset9 = sextend(instr, 9);
  lc3_word flag = f_dr(instr);
  if (flag & get_reg_val(vm, R_COND))
    reg_write(vm, R_PC, get_reg_val(vm, R_PC) + pc_offset9, false);
}

void handle_jmp(lc3_vm_p vm, lc3_word instr) {
  lc3_reg base_r = f_base_r(instr);
  reg_write(vm, R_PC, get_reg_val(vm, base_r), false);
}

void handle_jsr(lc3_vm_p vm, lc3_word instr) {
  lc3_reg temp = get_reg_val(vm, R_PC);
  if (test_bit(instr, 11)) {
    reg_write(vm, R_PC, f_base_r(instr), false);
  } else {
    lc3_addr pc_offset9 = sextend((instr & 0x7ff), 11);
    reg_write(vm, R_PC, get_reg_val(vm, R_PC) + pc_offset9, false);
  }
  reg_write(vm, R_R7, temp, false);
}

void handle_ld(lc3_vm_p vm, lc3_word instr) {
  // TODO: if computed address is in privileged memory AND PSR[15] == 1
  //  Initiate ACV exception
  lc3_reg dr = f_dr(instr);
  lc3_addr pc_offset9 = sextend(instr, 9);
  reg_write(vm, dr, vm_read_memory(vm, get_reg_val(vm, R_PC) + pc_offset9),
            true);
}

void handle_ldi(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode LDI\n");
}
void handle_ldr(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode LDR\n");
}

void handle_lea(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  lc3_addr pc_offset9 = sextend(instr, 9);
  reg_write(vm, dr, get_reg_val(vm, R_PC) + pc_offset9, true);
}

void handle_not(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  reg_write(vm, dr, ~get_reg_val(vm, f_sr(instr)), true);
}

void handle_rti(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode RTI\n");
}

void handle_st(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode ST\n");
}

void handle_sti(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode STI\n");
}

void handle_str(lc3_vm_p vm, lc3_word instr) {
  msg("Unimplemented opcode STR\n");
}

void handle_trap(lc3_vm_p vm, lc3_word instr) {
  uint16_t vect8 = f_vect8(instr);
  if (vect8 >= ARRAY_SIZE(dispatch_trap_table))
    return;
  trap_handler_t handler = dispatch_trap_table[vect8];
  if (!handler)
    return;
  handler(vm);
}

void handle_reserved(lc3_vm_p vm, lc3_word instr) { return; }

static const ins_handler_t dispatch_opcode_table[16] = {
    [VM_OPCODE_ADD] = handle_add,
    [VM_OPCODE_AND] = handle_and,
    [VM_OPCODE_BR] = handle_br,
    [VM_OPCODE_JMP] = handle_jmp,
    [VM_OPCODE_JSR] = handle_jsr,
    [VM_OPCODE_LD] = handle_ld,
    [VM_OPCODE_LDI] = handle_ldi,
    [VM_OPCODE_LEA] = handle_lea,
    [VM_OPCODE_NOT] = handle_not,
    [VM_OPCODE_RTI] = handle_rti,
    [VM_OPCODE_ST] = handle_st,
    [VM_OPCODE_STI] = handle_sti,
    [VM_OPCODE_STR] = handle_str,
    [VM_OPCODE_TRAP] = handle_trap,
    [VM_OPCODE_RESERVED] = handle_reserved,
};

// fetch/exucution logic
vm_run_result vm_fetch_execute(lc3_vm_p vm) {
  // Fetch instruction, increment PC, and execute.
  lc3_addr *pc = get_reg_ptr(vm, R_PC);
  lc3_word instr = vm_read_memory(vm, (*pc)++);
  return vm_run_instr(vm, instr);
}

void vm_call_handler(lc3_vm_p vm, lc3_word instr, const vm_opcode opcode) {
  ins_handler_t handler = dispatch_opcode_table[opcode];
  handler(vm, instr);
}

vm_run_result vm_run_instr(lc3_vm_p vm, lc3_word instr) {
  vm_opcode opcode = instr >> 12;

  if (opcode >= ARRAY_SIZE(dispatch_opcode_table))
    return RUN_UNHANDLED_OPCODE;

  vm_call_handler(vm, instr, opcode);
  return RUN_SUCCESS;
}