#include "../thirdparty/linenoise/linenoise.h"
#include "instructions.h"
#include "loader.h"
#include "vm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define HISTORY_MAX_LEN 100

typedef void (*cmd_handler_func)(lc3_vm_p vm, const char *args);

typedef enum { RUN = 0, QUIT, HELP, BREAK, CONTINUE, UNKNOWN } cmd_type;

typedef enum {
  PARSE_SUCCESS_CODE = 0,
  PARSE_ERROR_CODE,
  PARSE_EXIT_CODE,
  PARSE_UNKONWN_CODE
} parse_line_result;

typedef struct {
  char *name;
  char *alias;
  char *description;
  cmd_handler_func func;
} cmd_handler;

const char *register_names[] = {
    [R_R0] = "R0", [R_R1] = "R1",     [R_R2] = "R2", [R_R3] = "R3",
    [R_R4] = "R4", [R_R5] = "R5",     [R_R6] = "R6", [R_R7] = "R7",
    [R_PC] = "PC", [R_COND] = "COND",
};

void handle_break(lc3_vm_p vm, const char *args) {
  (void)vm;
  char *str, *s_addr;
  uint16_t addr;
  str = strdup(args);
  s_addr = strtok(str, " ");
  if (s_addr == NULL)
    goto cleanup;

  long value = strtol(s_addr, NULL, 16);
  if (value < PC_START_POSITION || value >= MAX_MEM_SIZE) {
    err("Error: Invalid breakpoint address 0x%04lX.\n", value);
    msg("Breakpoints must be within (0x3000 - 0xFFFF).\n");
    goto cleanup;
  }
  addr = (uint16_t)value;
  set_breakpoint(vm, addr);

cleanup:
  free(str);
  return;
}

void handle_run(lc3_vm_p vm, const char *args) {
  (void)args;
  vm_run(vm);
  return;
}

void handle_continue(lc3_vm_p vm, const char *args) {
  (void)args;
  vm_step(vm);
  vm_run(vm);
  return;
}

void handle_registers(lc3_vm_p vm, const char *args) {
  (void)args;
  for (uint16_t reg_index = 0; reg_index < R_COUNT; reg_index++)
    msg("\t%s   : 0x%04X\n", register_names[reg_index],
        get_reg_val(vm, reg_index));
  putc('\n', stdout);
}

void handle_next_instr(lc3_vm_p vm, const char *args) {
  (void)args;
  vm_step_over(vm);
}

const cmd_handler cmd_handler_table[] = {
    {"break", "br", "Set a breakpoint at a hex address", handle_break},
    {"run", "r", "Start execution of the program", handle_run},
    {"continue", "c", "Continue execution after a break", handle_continue},
    {"registers", "reg", "Print current register values", handle_registers},
    {"nexti", "ni", "Execute the next instruction", handle_next_instr}};

static bool cmd_handler_compare(cmd_handler c, const char *str) {
  return (strcmp(str, c.name) == 0) || (strcmp(str, c.alias) == 0);
}

static bool cmd_check_command(const char *str) {
  if (str == NULL)
    return false;

  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    cmd_handler c = cmd_handler_table[i];
    if (cmd_handler_compare(c, str))
      return true;
  }
  return false;
}

void print_help(void) {
  msg("Available commands:\n");
  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    msg("  %-12s (alias: %-3s)  - %s\n", cmd_handler_table[i].name,
        cmd_handler_table[i].alias, cmd_handler_table[i].description);
  }
  msg("  %-12s (alias: %-3s)  - Exit the debugger\n", "quit", "q");
}

parse_line_result handle_command(lc3_vm_p vm, const char *line) {
  char *token, *str, *args;
  str = strdup(line);
  if (str == NULL)
    goto error;

  token = strtok(str, " ");
  if (token == NULL)
    goto success;

  if ((strcmp(token, "quit") == 0) || (strcmp(token, "q") == 0))
    goto exit;

  if ((strcmp(token, "help") == 0) || (strcmp(token, "h") == 0)) {
    print_help();
    goto success;
  }

  if (!cmd_check_command(token)) {
    msg("Undefined command: '%s'.  Try 'help'.\n", token);
    goto unknown;
  }

  args = strtok(NULL, "");

  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    cmd_handler c = cmd_handler_table[i];
    if (cmd_handler_compare(c, token)) {
      c.func(vm, args);
      goto success;
    }
  }

error:
  free(str);
  return PARSE_ERROR_CODE;
exit:
  free(str);
  return PARSE_EXIT_CODE;
unknown:
  free(str);
  return PARSE_UNKONWN_CODE;
success:
  free(str);
  return PARSE_SUCCESS_CODE;
}

void completion(const char *buf, linenoiseCompletions *lc) {
  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    if (is_prefix(buf, cmd_handler_table[i].name))
      linenoiseAddCompletion(lc, cmd_handler_table[i].name);
  }
}

int main(int argc, char **argv) {
  lc3_vm_p vm;
  char *line;
  if (argc < 2) {
    err("Usage: %s <objfile>\n", argv[0]);
    return ERROR_CODE;
  }

  vm = vm_create();
  if (load_obj_file(vm, argv[1]) != SUCCESS_CODE) {
    err("Failed to load %s\n", argv[1]);
    return ERROR_CODE;
  }

  linenoiseHistorySetMaxLen(HISTORY_MAX_LEN);
  linenoiseSetCompletionCallback(completion);
  for (;;) {
    if ((line = linenoise("lc3-dbg> ")) == NULL)
      break;

    parse_line_result result = handle_command(vm, line);
    if (result == PARSE_UNKONWN_CODE) {
      linenoiseFree(line);
      continue;
    }
    else if ((result == PARSE_ERROR_CODE) || (result == PARSE_EXIT_CODE)) {
      linenoiseFree(line);
      break;
    }

    linenoiseHistoryAdd(line);
    linenoiseFree(line);
  }

  vm_destroy(vm);
  return SUCCESS_CODE;
}