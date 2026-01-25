#include "vm.h"
#include "instructions.h"
#include <setjmp.h>
#include <stdbool.h>
#include <stdlib.h>

// VM lifecycle
lc3_vm_p vm_create() {
  lc3_vm_p vm = calloc(1, sizeof(struct lc3_vm));
  if (vm == NULL) {
    err("%s: could not allocate memory for 'vm' \n", __func__);
    exit(ERROR_CODE);
  }

  return vm;
}

void vm_destroy(lc3_vm_p vm) { free(vm); }

void vm_run(lc3_vm_p vm) {
  if (setjmp(vm->buf) != 0) {
    msg("VM execution halted safely.\n");
    return;
  }

  while (true)
    vm_fetch_execute(vm);
}

// Read/Write
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
