#include "instructions.h"
#include "loader.h"

#ifdef DEBUG_MODE
#include "debugger.h"
#endif

#include "vm.h"

int main(int argc, char **argv) {
  lc3_vm_p vm;

  if (argc < 2) {
#ifdef DEBUG_MODE
    die("Usage: lc3-dbg <objfile>\n");
#else
    die("Usage: lc3 <objfile>\n");
#endif
  }

  vm = vm_create();
  if (load_obj_file(vm, argv[1]) != SUCCESS_CODE) {
    vm_destroy(vm);
    die("Failed to load %s\n", argv[1]);
  }
#ifdef DEBUG_MODE
  debugger_run(vm);
#else
  vm_run(vm);
#endif
  vm_destroy(vm);
  return SUCCESS_CODE;
}