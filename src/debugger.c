#include "debugger.h"
#include "instructions.h"
#include "linenoise.h"
#include "loader.h"
#include "vm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define DEBUGGER_PROMPT "(lc3-dbg) "
#define HISTORY_MAX_LEN 100

typedef void (*cmd_handler_func)(lc3_vm_p, const char *);

typedef enum {
  PARSE_SUCCESS_CODE = 0,
  PARSE_ERROR_CODE,
  PARSE_EXIT_CODE,
  PARSE_UNKNOWN_CODE
} parse_line_result;

typedef struct {
  char *name;
  char *alias;
  char *description;
  cmd_handler_func handler;
} cmd_handler;

const char *register_names[] = {
    [R_R0] = "R0", 
    [R_R1] = "R1",    
    [R_R2] = "R2", 
    [R_R3] = "R3",
    [R_R4] = "R4", 
    [R_R5] = "R5",    
    [R_R6] = "R6", 
    [R_R7] = "R7",
    [R_PC] = "PC", 
    [R_COND] = "COND"
};

void handle_break(lc3_vm_p vm, const char *args) {
  char *str, *s_addr;
  uint16_t addr;
  str = xstrdup(args);
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
  UNUSED(args);
  vm_run(vm);
  return;
}

void handle_continue(lc3_vm_p vm, const char *args) {
  UNUSED(args);
#ifdef DEBUG_MODE
  // Only run "continue" if the vm is running
  vm_single_step(vm);
  vm_execution_status status = vm_current_status(vm);
  if ((status == VM_HIT_BREAKPOINT) || (status == VM_IS_RUNNING)) {
    msg("Continuing.\n");
    vm_run(vm);
  }
  return;
#else
  UNUSED(vm);
  return;
#endif
}

void handle_registers(lc3_vm_p vm, const char *args) {
  UNUSED(args);
  for (int reg_index = 0; reg_index < R_COUNT; reg_index++)
    msg("\t%s   : 0x%04X\n", register_names[reg_index],
        get_reg_val(vm, reg_index));
  putc('\n', stdout);
}

void handle_next_instr(lc3_vm_p vm, const char *args) {
  // Default just execute one instruction
  if (!args || strlen(args) == 0) {
    vm_single_step(vm);
    return;
  }

  char *str = xstrdup(args);
  char *ptr = strstrip(str, ' ', '\t');
  if (strlen(ptr) == 0) {
    vm_single_step(vm);
    goto finish;
  }

  int count;
  if (sscanf(ptr, "%d", &count) == 0) {
    msg("No symbol '%s' in current context.\n", ptr);
    goto finish;
  }

  int ic = MIN(count, get_instruction_count(vm));
  while (ic-- > 0)
    vm_single_step(vm);

finish:
  free(str);
  return;
}

const cmd_handler cmd_handler_table[] = {
    {"break", "br", "Set a breakpoint at a hex address", handle_break},
    {"run", "r", "Start execution of the program", handle_run},
    {"continue", "c", "Continue execution after a break", handle_continue},
    {"registers", "reg", "Print current register values", handle_registers},
    {"nexti", "ni", "Execute the next instruction", handle_next_instr}};

static bool cmd_handler_compare(cmd_handler command, const char *str) {
  return (strcmp(str, command.name) == 0) || (strcmp(str, command.alias) == 0);
}

static bool cmd_check_command(const char *str) {
  if (str == NULL)
    return false;

  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    cmd_handler command = cmd_handler_table[i];
    if (cmd_handler_compare(command, str))
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
  parse_line_result result = PARSE_SUCCESS_CODE;
  if ((str = xstrdup(line)) == NULL) {
    result = PARSE_ERROR_CODE;
    goto finish;
  }

  if ((token = strtok(str, " ")) == NULL) {
    result = PARSE_SUCCESS_CODE;
    goto finish;
  }

  if ((strcmp(token, "help") == 0) || (strcmp(token, "h") == 0)) {
    result = PARSE_EXIT_CODE;
    goto finish;
  }

  if ((strcmp(token, "quit") == 0) || (strcmp(token, "q") == 0)) {
    print_help();
    result = PARSE_SUCCESS_CODE;
    goto finish;
  }

  if (!cmd_check_command(token)) {
    msg("Undefined command: '%s'.  Try 'help'.\n", token);
    result = PARSE_UNKNOWN_CODE;
    goto finish;
  }

  args = strtok(NULL, "");

  for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
    cmd_handler command = cmd_handler_table[i];
    if (cmd_handler_compare(command, token)) {
      command.handler(vm, args);
      result = PARSE_SUCCESS_CODE;
      goto finish;
    }
  }

finish:
  free(str);
  return result;
}

void completion(const char *buf, linenoiseCompletions *lc) {
  if (startwith(buf, "help")) {
    linenoiseAddCompletion(lc, "help");
  } else if (startwith(buf, "quit")) {
    linenoiseAddCompletion(lc, "quit");
  } else {
    for (size_t i = 0; i < ARRAY_SIZE(cmd_handler_table); i++) {
      if (startwith(buf, cmd_handler_table[i].name))
        linenoiseAddCompletion(lc, cmd_handler_table[i].name);
    }
  }
}

void repl_init(void) {
  linenoiseHistorySetMaxLen(HISTORY_MAX_LEN);
  linenoiseSetCompletionCallback(completion);
  print_help();
}

int debugger_run(lc3_vm_p vm) {
  repl_init();

  char *line;
  while ((line = linenoise(DEBUGGER_PROMPT)) != NULL) {

    parse_line_result result = handle_command(vm, line);
    if (result != PARSE_UNKNOWN_CODE)
      linenoiseHistoryAdd(line);
    linenoiseFree(line);

    if (result == PARSE_ERROR_CODE || result == PARSE_EXIT_CODE)
      break;
  }

  return SUCCESS_CODE;
}