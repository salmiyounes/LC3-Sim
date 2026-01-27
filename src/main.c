#include "instructions.h"
#include "vm.h"
#include "loader.h"

int main(int argc, char **argv) {
  lc3_vm_p vm = vm_create();

  if (argc < 2) {
    err("Usage: %s <objfile>\n", argv[0]);
    return ERROR_CODE;
  }
  if (load_obj_file(vm, argv[1]) != SUCCESS_CODE) {
    err("Failed to load %s\n", argv[1]);
    return ERROR_CODE;
  }

  vm_run(vm);

  vm_destroy(vm);
  return SUCCESS_CODE;
}