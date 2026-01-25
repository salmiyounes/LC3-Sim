#include "instructions.h"
#include "vm.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

int load_data(lc3_vm_p vm, unsigned const char *data, size_t lenght) {
  size_t load_lenght = (lenght - sizeof(lc3_addr)) / sizeof(lc3_word);
  lc3_addr load_addr = (lc3_addr)bswap16(*((lc3_addr *)data));

  lc3_word *dst = vm->memory + load_addr;
  lc3_word *src = (lc3_word *)(data + sizeof(lc3_addr));

  while (load_lenght-- > 0)
    *(dst++) = (lc3_word)bswap16(*(src++));

  reg_write(vm, R_PC, load_addr, false);
  return SUCCESS_CODE;
}

int load_obj_file(lc3_vm_p vm, const char *filename) {
  int fd;
  unsigned char *data;
  struct stat sb;
  if ((fd = open(filename, O_RDONLY)) < 0)
    return ERROR_CODE;

  if (fstat(fd, &sb) < 0)
    return ERROR_CODE;

  if ((data = mmap(NULL, sb.st_size, PROT_READ, MAP_SHARED, fd, 0)) ==
      MAP_FAILED)
    return ERROR_CODE;

  int result = load_data(vm, data, sb.st_size);

  munmap(data, sb.st_size);
  close(fd);

  return result;
}

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