#include "instructions.h"
#include "loader.h"
#include "vm.h"

int main(int argc, char **argv) {
  lc3_vm_p vm;

  if (argc < 2)
    die("Usage: %s <objfile>\n", argv[0]);

  vm = vm_create();
  if (load_obj_file(vm, argv[1]) != SUCCESS_CODE) {
    vm_destroy(vm);
    die("Failed to load %s\n", argv[1]);
  }

  vm_run(vm);

  vm_destroy(vm);
  return SUCCESS_CODE;
}