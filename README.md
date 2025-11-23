# LC-3 Virtual Machine (LC-3 VM)

## Resources
- [Little Computer 3 (LC-3)](https://en.wikipedia.org/wiki/Little_Computer_3) 
- [Introduction to Computing Systems: From Bits and Gates to C and Beyond](https://icourse.club/uploads/files/96a2b94d4be48285f2605d843a1e6db37da9a944.pdf)
- [LC-3 Simulator](https://wchargin.github.io/lc3web/)
- [The LC-3 ISA](https://icourse.club/uploads/files/a9710bf2454961912f79d89b25ba33c4841f6c24.pdf)

### Build
```sh
gcc vm.c -o lc3
```

### Usage
```sh
./lc3 <program.obj>
#example
./lc3 tests/hello-world.obj
```