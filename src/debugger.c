#include "loader.h"
#include "vm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef enum { RUN = 0, QUIT, HELP, BREAK, CONTINUE, UNKNOWN } command_type;

command_type parse_line(char *line) {
  char *str = _strdup(line);
  command_type type = UNKNOWN;
  if (str == NULL)
    return UNKNOWN;

  str[strcspn(str, "\r\n")] = '\0';
  char *token = strtok(str, " ");
  if (token == NULL) {
    free(str);
    return UNKNOWN;
  }

  if (strcmp(token, "run") == 0) {
    type = RUN;
  } else if (strcmp(token, "quit") == 0) {
    type = QUIT;
  } else if (strcmp(token, "help") == 0) {
    type = HELP;
  } else if (strcmp(token, "break") == 0) {
    type = BREAK;
  } else if (strcmp(token, "continue") == 0) {
    type = CONTINUE;
  }

  free(str);
  return type;
}

void print_usage(const char *prog_name) {
  msg("LC-3 GDB-Style Debugger\n");
  msg("Usage: %s <objfile>\n", prog_name);
  msg("\nArguments:\n");
  msg("  <objfile>   The LC-3 .obj file to debug\n");
  msg("\nInteractive Commands (within lc3-dbg>):\n");
  msg("  run         Starts or resumes execution\n");
  msg("  quit        Exits the debugger and destroys the VM\n");
}

int main(int argc, char **argv) {
  lc3_vm_p vm;
  if (argc < 2) {
    err("Usage: %s <objfile>\n", argv[0]);
    return ERROR_CODE;
  }

  vm = vm_create();
  if (load_obj_file(vm, argv[1]) != SUCCESS_CODE) {
    err("Failed to load %s\n", argv[1]);
    return ERROR_CODE;
  }

  char line[256];
  bool keep_running = true;

  while (keep_running) {
    msg("lc3-dbg> ");
    if (fgets(line, sizeof(line), stdin) == NULL)
      break;

    switch (parse_line(line)) {
    case RUN:
      msg("Starting execution...\n");
      vm_run(vm);
      break;
    case BREAK:
      uint16_t addr;
      msg("%s\n", line);
      if (sscanf(line, "break %hd", &addr) == 1) {
        if (addr < MAX_MEM_SIZE) {
#ifdef DEBUG_MODE
          vm->breakpoints[addr] = !vm->breakpoints[addr];
          msg("Breakpoint %s at 0x%04X\n",
              vm->breakpoints[addr] ? "set" : "cleared", (uint16_t)addr);
#else
          err("Error: Debugger compiled without DEBUG_MODE.\n");
#endif
        } else {
          err("Error: Address 0x%X is out of range.\n", addr);
        }
      } else {
        err("Usage: break <addr> (e.g., break 12288)\n");
      }
      break;
    case CONTINUE:
      vm_run(vm);
      break;
    case HELP:
      print_usage(argv[0]);
      break;
    case QUIT:
      msg("Exiting debugger.\n");
      keep_running = false;
      break;
    default:
      err("Unknown command. Type 'help' for options.\n");
      break;
    }
  }
}