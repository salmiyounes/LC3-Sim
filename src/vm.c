#define _GNU_SOURCE
#include "vm.h"
#include "instructions.h"
#include <setjmp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>

static volatile sig_atomic_t keep_running;

// VM lifecycle
lc3_vm_p vm_create() {
  lc3_vm_p vm = calloc(1, sizeof(struct lc3_vm));
  if (vm == NULL)
    die("%s: could not allocate memory for 'vm' \n", __func__);

#ifdef DEBUG_MODE
  vm->status = VM_STOP_RUNNING;
#endif
  vm->last_result = RUN_SUCCESS;
  return vm;
}

void vm_destroy(lc3_vm_p vm) { free(vm); }

void vm_signal_handler(int sig_id) {
  UNUSED(sig_id);
  keep_running = 0;
}

const char *vm_error(lc3_vm_p vm) {
  static const char *vm_error_messages[] = {
      [RUN_SUCCESS] = "VM execution halted safely.\n",
      [RUN_FAIL] = "Fatal: General VM failure.\n",
      [RUN_UNHANDLED_OPCODE] = "Fatal: Unhandled or illegal opcode.\n",
  };

  vm_run_result flag = vm->last_result;

#ifdef DEBUG_MODE
  if (flag == RUN_BREAKPOINT_STOP) {
    static char buf[100];
    snprintf(buf, sizeof(buf), "Breakpoint %ld hit at PC: 0x%04X\n",
             vm_breakpoints_count(vm), vm->reg[R_PC]);
    return buf;
  }
#endif

  if (flag < ARRAY_SIZE(vm_error_messages))
    return vm_error_messages[flag];

  return "Fatal: Unknown VM error.\n";
}

void vm_run(lc3_vm_p vm) {
#ifdef DEBUG_MODE
  vm->status = VM_IS_RUNNING;
#endif
  if (setjmp(vm->buf) != 0) {
    vm_run_result flag = vm->last_result;
    const char *log_msg = vm_error(vm);

    if (flag == RUN_BREAKPOINT_STOP) {
      vm->last_result = RUN_SUCCESS;
      msg("%s", log_msg);
    } else if (flag != RUN_SUCCESS) {
      err("%s", log_msg);
    } else {
      msg("%s", log_msg);
    }

    return;
  }

  // Handle CTRL-C
  struct sigaction act;
  memset(&act, 0, sizeof(struct sigaction));
  sigemptyset(&act.sa_mask);

  act.sa_handler = vm_signal_handler;
  act.sa_flags = SA_SIGINFO;

  if (sigaction(SIGINT, &act, NULL) == -1)
    die("sigaction(): cannot handle SIGINT");

  keep_running = 1;

  while (keep_running) {
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
