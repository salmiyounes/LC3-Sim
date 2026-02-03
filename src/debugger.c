#include "vm.h"
#include "loader.h"
#include <string.h>
#include <stdbool.h>

typedef enum {
    RUN = 0,
    QUIT,
    HELP,
    UNKNOWN
} command_type;

command_type parse_line(char *line) {
    line[strcspn(line, "\r\n")] = '\0';
    char *token = strtok(line, " ");
    if (token == NULL) 
        return UNKNOWN;

    if (strcmp(token, "run") == 0) {
        return RUN;
    } else if (strcmp(token, "quit") == 0) {
        return QUIT;
    } else if (strcmp(token, "help") == 0) {
        return HELP;
    }

    return UNKNOWN;
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