#ifndef VM_H
#define VM_H

#include "types.h"
#include <stdbool.h>

#ifdef DEBUG_MODE
static inline vm_exucution_status vm_current_status(lc3_vm_p vm) {
  return vm->status;
}
#endif

lc3_vm_p vm_create();

void vm_run(lc3_vm_p vm);

void vm_destroy(lc3_vm_p vm);

void vm_write_memory(lc3_vm_p vm, lc3_word addr, lc3_word value);

lc3_word vm_read_memory(lc3_vm_p vm, lc3_addr addr);

#endif // VM_H