# LC-3 Virtual Machine with Debugger

## Overview

This project implements a full **LC-3 Virtual Machine (VM)** — a simple 16-bit educational instruction set architecture widely used in computer architecture courses.

Unlike most LC-3 VM implementations, this project includes an **interactive debugger** that allows users to inspect and control the machine state during program execution. The debugger uses [linenoise](https://github.com/antirez/linenoise) to enhance the experience with features like **tab completion**, **command history**, and improved input editing.

The goal of this project is to bridge the gap between theoretical computer architecture concepts and hands-on, low-level programming experience.

---

## Build Instructions

Clone the repository and build the project:

```bash
git clone https://github.com/salmiyounes/LC3-Sim.git
cd LC3-Sim

mkdir build
cd build

cmake ..
make
```

This will generate two executables:

* `lc3` — Standard LC-3 virtual machine
* `lc3-dbg` — LC-3 VM with an interactive debugger

---

## Features

### LC-3 VM Core

#### Running a Simple Program

```bash
./lc3 ../tests/hello-world.obj
```

---

### Integrated Debugger

The debugger provides full control over program execution, including breakpoints, step-by-step execution, and inspection of registers. Features powered by **linenoise** include:

* **Tab completion** for commands and addresses
* **Command history**
* **Enhanced input editing**

#### Basic Commands

```text
  break        (alias: br )  - Set a breakpoint at a hex address
  run          (alias: r  )  - Start execution of the program
  continue     (alias: c  )  - Continue execution after hitting a breakpoint
  registers    (alias: reg)  - Print current register values
  nexti        (alias: ni )  - Execute the next instruction
  quit         (alias: q  )  - Exit the debugger
```

#### Debugger Demo

```bash
(lc3-dbg) registers
        R0   : 0x0000
        R1   : 0x0000
        R2   : 0x0000
        R3   : 0x0000
        R4   : 0x0000
        R5   : 0x0000
        R6   : 0x0000
        R7   : 0x0000
        PC   : 0x3000
        COND : 0x0000

(lc3-dbg) br 0x3000
Breakpoint set at 0x3000

(lc3-dbg) r
Breakpoint hit at PC: 0x3000

(lc3-dbg) nexti 4
hello world!

(lc3-dbg) reg
        R0   : 0x3008
        R1   : 0x0005
        R2   : 0x0000
        R3   : 0x0000
        R4   : 0x0000
        R5   : 0x0000
        R6   : 0x0000
        R7   : 0x0000
        PC   : 0x3004
        COND : 0x0001
```

---

## Roadmap / TODOs

* Implement remaining opcodes
* Complete all TRAP routines
* Implement memory-mapped I/O
* Add ability for the debugger to modify register values and inspect memory
* Add a `watch` command for observing specific memory or register changes
* Improve test suite

---

## Resources

* [Little Computer 3 (LC-3) - Wikipedia](https://en.wikipedia.org/wiki/Little_Computer_3)
* [Introduction to Computing Systems: From Bits and Gates to C and Beyond](https://icourse.club/uploads/files/96a2b94d4be48285f2605d843a1e6db37da9a944.pdf)
* [LC-3 Web Simulator](https://wchargin.github.io/lc3web/)
* [LC-3 Instruction Set Architecture (PDF)](https://icourse.club/uploads/files/a9710bf2454961912f79d89b25ba33c4841f6c24.pdf)

