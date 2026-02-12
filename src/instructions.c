#include "instructions.h"
#include "vm.h"
#include <setjmp.h>
#include <stdbool.h>

typedef void (*ins_handler_t)(lc3_vm_p vm, lc3_word instr);
typedef void (*trap_handler_t)(lc3_vm_p vm);

#define CONCAT(a, b) a##b
#define trap(x) CONCAT(_trap_, x)
#define handle(x) CONCAT(_handle_, x)

// Trap functions
void _trap_puts(lc3_vm_p vm) {
  lc3_addr *c = vm->memory + get_reg_val(vm, R_R0);
  while (*c) {
    putc((char)*(c), stdout);
    ++c;
  }
}

void _trap_out(lc3_vm_p vm) {
  fputc((char)get_reg_val(vm, R_R0) & 0XFF, stdout); // R0[7:0]
  fflush(stdout);
}

void _trap_halt(lc3_vm_p vm) {
#ifdef DEBUG_MODE
  vm->status = VM_STOP_RUNNING;
#endif
  longjmp(vm->buf, 1);
}

static const trap_handler_t dispatch_trap_table[38] = {
    [PUTS] = trap(puts),
    [OUT] = trap(out),
    [HALT] = trap(halt),
};

// Opcode functions
void _handle_add(lc3_vm_p vm, lc3_word instr) {
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

void _handle_and(lc3_vm_p vm, lc3_word instr) {
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

void _handle_br(lc3_vm_p vm, lc3_word instr) {
  lc3_addr pc_offset9 = sextend(instr, 9);
  lc3_word flag = f_dr(instr);
  if (flag & get_reg_val(vm, R_COND))
    reg_write(vm, R_PC, get_reg_val(vm, R_PC) + pc_offset9, false);
}

void _handle_jmp(lc3_vm_p vm, lc3_word instr) {
  lc3_reg base_r = f_base_r(instr);
  reg_write(vm, R_PC, get_reg_val(vm, base_r), false);
}

void _handle_jsr(lc3_vm_p vm, lc3_word instr) {
  lc3_reg temp = get_reg_val(vm, R_PC);
  if (test_bit(instr, 11)) {
    reg_write(vm, R_PC, f_base_r(instr), false);
  } else {
    lc3_addr pc_offset9 = sextend((instr & 0x7ff), 11);
    reg_write(vm, R_PC, get_reg_val(vm, R_PC) + pc_offset9, false);
  }
  reg_write(vm, R_R7, temp, false);
}

void _handle_ld(lc3_vm_p vm, lc3_word instr) {
  // TODO: if computed address is in privileged memory AND PSR[15] == 1
  //  Initiate ACV exception
  lc3_reg dr = f_dr(instr);
  lc3_addr pc_offset9 = sextend(instr, 9);
  reg_write(vm, dr, vm_read_memory(vm, get_reg_val(vm, R_PC) + pc_offset9),
            true);
}

void _handle_ldi(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode LDI\n");
  assert(0);
}
void _handle_ldr(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode LDR\n");
  assert(0);
}

void _handle_lea(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  lc3_addr pc_offset9 = sextend(instr, 9);
  reg_write(vm, dr, get_reg_val(vm, R_PC) + pc_offset9, true);
}

void _handle_not(lc3_vm_p vm, lc3_word instr) {
  lc3_reg dr = f_dr(instr);
  reg_write(vm, dr, ~get_reg_val(vm, f_sr(instr)), true);
}

void _handle_rti(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode RTI\n");
  assert(0);
}

void _handle_st(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode ST\n");
  assert(0);
}

void _handle_sti(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode STI\n");
  assert(0);
}

void _handle_str(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  msg("Unimplemented opcode STR\n");
  assert(0);
}

void vm_call_trap_handler(lc3_vm_p vm, const uint16_t vect8) {
  trap_handler_t handler = dispatch_trap_table[vect8];
  if (!handler)
    return;
  handler(vm);
}

void _handle__trap(lc3_vm_p vm, lc3_word instr) {
  uint16_t vect8 = f_vect8(instr);
  if (vect8 >= ARRAY_SIZE(dispatch_trap_table))
    return;
  vm_call_trap_handler(vm, vect8);
}

void _handle_reserved(lc3_vm_p vm, lc3_word instr) {
  (void)vm;
  (void)instr;
  return;
}

static const ins_handler_t dispatch_opcode_table[16] = {
    [VM_OPCODE_ADD] = handle(add),
    [VM_OPCODE_AND] = handle(and),
    [VM_OPCODE_BR] = handle(br),
    [VM_OPCODE_JMP] = handle(jmp),
    [VM_OPCODE_JSR] = handle(jsr),
    [VM_OPCODE_LD] = handle(ld),
    [VM_OPCODE_LDI] = handle(ldi),
    [VM_OPCODE_LEA] = handle(lea),
    [VM_OPCODE_NOT] = handle(not ),
    [VM_OPCODE_RTI] = handle(rti),
    [VM_OPCODE_ST] = handle(st),
    [VM_OPCODE_STI] = handle(sti),
    [VM_OPCODE_STR] = handle(str),
    [VM_OPCODE_TRAP] = handle(_trap),
    [VM_OPCODE_RESERVED] = handle(reserved),
};

// fetch/exucution logic
vm_run_result vm_step(lc3_vm_p vm) {
  lc3_addr *pc = get_reg_ptr(vm, R_PC);
  lc3_word instr = vm_read_memory(vm, (*pc)++);
  return vm_run_instr(vm, instr);
}

void vm_step_over(lc3_vm_p vm) {
#ifdef DEBUG_MODE
  // Fix HALT problem
  if (setjmp(vm->buf) != 0)
    return;
  switch (vm->status) {
  case VM_IS_RUNNING:
  case VM_HIT_BREAKPOINT:
    vm->last_result = vm_step(vm);
    break;
  default:
    err("The program is not being run.\n");
    break;
  }
#endif
  (void)vm;
  return;
}

vm_run_result vm_fetch_execute(lc3_vm_p vm) {
  // Check for breakpoints
  if (is_breakpoint_hit(vm)) {
#ifdef DEBUG_MODE
    vm->status = VM_HIT_BREAKPOINT;
#endif
    vm->last_result = RUN_BREAKPOINT_STOP;
    longjmp(vm->buf, 1);
  }
  return vm_step(vm);
}

void vm_call_handler(lc3_vm_p vm, lc3_word instr, const vm_opcode opcode) {
  ins_handler_t handler = dispatch_opcode_table[opcode];
  handler(vm, instr);
}

vm_run_result vm_run_instr(lc3_vm_p vm, const lc3_word instr) {
  vm_opcode opcode = f_opcode(instr);

  if (opcode >= ARRAY_SIZE(dispatch_opcode_table))
    return RUN_UNHANDLED_OPCODE;

  vm_call_handler(vm, instr, opcode);
  return RUN_SUCCESS;
}