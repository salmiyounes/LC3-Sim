#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include "types.h"

static bool sign_bit(const uint16_t x) { return (bool)(x & SIGN_FLAG_BIT); }

static vm_condition_codes vm_sign_flag(const uint16_t value) {
  if (value == 0)
    return FLAG_ZRO;
  else if (sign_bit(value))
    return FLAG_NEG;
  else
    return FLAG_POS;
}

static void vm_setcc(lc3_vm_p vm, const lc3_reg reg_index) {
  lc3_addr *ptr = get_reg_ptr(vm, R_COND);
  *ptr = vm_sign_flag(get_reg_val(vm, reg_index));
}

__attribute__((unused)) static void reg_write(lc3_vm_p vm, const lc3_reg reg_index,
                      const lc3_word value, const bool is_setcc) {
  lc3_addr *ptr = get_reg_ptr(vm, reg_index);
  *ptr = value;
  if (is_setcc)
    vm_setcc(vm, reg_index);
}

static inline vm_opcode f_opcode(const lc3_word instr) {
  return (vm_opcode)(instr >> 12);
}

vm_run_result vm_fetch_execute(lc3_vm_p vm);

vm_run_result vm_run_instr(lc3_vm_p vm, lc3_word instr);

#endif // INSTRUCTIONS_H
