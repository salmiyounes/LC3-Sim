#include "vm.h"
#include "instructions.h"
#include <setjmp.h>
#include <stdbool.h>
#include <stdlib.h>

static const char *vm_error_messages[] = {
    [RUN_SUCCESS] = "VM execution halted safely.\n",
    [RUN_FAIL] = "Fatal: General VM failure.\n",
    [RUN_UNHANDLED_OPCODE] = "Fatal: Unhandled or illegal opcode.\n"};

// VM lifecycle
lc3_vm_p vm_create() {
  lc3_vm_p vm = calloc(1, sizeof(struct lc3_vm));
  if (vm == NULL) {
    err("%s: could not allocate memory for 'vm' \n", __func__);
    exit(ERROR_CODE);
  }
  vm->last_result = RUN_SUCCESS;
  return vm;
}

void vm_destroy(lc3_vm_p vm) { free(vm); }

void vm_run(lc3_vm_p vm) {
  if (setjmp(vm->buf) != 0) {
    vm_run_result flag = vm->last_result;
    if (flag < ARRAY_SIZE(vm_error_messages)) {
      const char *log_msg = vm_error_messages[flag];
      if (flag != RUN_SUCCESS)
        err("%s", log_msg);
      else
        msg("%s", log_msg);
    }
    return;
  }

  for (;;) {
#ifdef DEBUG_MODE
    bool *ptr = vm->breakpoints + get_reg_val(vm, R_PC);
    if (*ptr) {
      // FIXME: i don't know if this the proper solution but it work
      *ptr = false;
      msg("Breakpoint hit at 0x%04X\n", get_reg_val(vm, R_PC));
      return;
    }
#endif
    if ((vm->last_result = vm_fetch_execute(vm)) != RUN_SUCCESS)
      longjmp(vm->buf, 1);
  }
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
