# Demo
This is a demo of wrapping a pointer to simple C++ class (MyCounter) in a C API & calling that C API from Go.

It provides equivolent C & Go main functions to produce C & Go executables which call into the C++ MyCounter object. The C one is useful for running static & dynamic tooling on (such as valgrind).

It includes a Makefile to compile the C, C++ & Go code with the appropriate configuration to link them together.

## Consulted references:

- https://renenyffenegger.ch/notes/development/languages/C-C-plus-plus/GCC/create-libraries/index
- https://github.com/ReneNyffenegger/gcc-create-library
- https://isocpp.org/wiki/faq/mixing-c-and-cpp#call-cpp
- https://pkg.go.dev/cmd/cgo
- https://stackoverflow.com/questions/1713214/how-to-use-c-in-go
- https://go.dev/doc/faq#Do%5FGo%5Fprograms%5Flink%5Fwith%5FCpp%5Fprograms

## Usage

```bash
make clean
make counter-go
make counter-c

bin/counter-go
From Go:
0 counter incremented 1
1 counter incremented 2
2 counter incremented 3
3 counter incremented 4
4 counter incremented 5


bin/counter-c
From C:
0 counter incremented 1
1 counter incremented 2
2 counter incremented 3
3 counter incremented 4
4 counter incremented 5


```

## TODO:
- [ ] Support compiling on different platforms without polluting build / bin directories
- [ ] Get counter-c to pass valgrind
- [ ] Add asan support
- [ ] Add C unit test
- [ ] Add C integration test
