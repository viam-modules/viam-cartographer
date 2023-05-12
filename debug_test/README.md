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

### Counter:
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


make valgrind
... bunch of errors

make valgrind
... bunch of errors


```

### Carto (mock only):
```bash
make carto-go
pre init viamCarto.initialized_flag 0
post init viamCarto.initialized_flag 5
response before main._Ctype_struct_viam_carto_get_position_response{x:0, y:0, z:0, o_x:0, o_y:0, o_z:0, theta:0, component_reference:(*main._Ctype_char)(nil)}
response after main._Ctype_struct_viam_carto_get_position_response{x:0, y:1, z:2, o_x:3, o_y:4, o_z:5, theta:6, component_reference:(*main._Ctype_char)(0x102a53e4f)}
component_reference: some_component_reference
calling viam_carto_WriteSensor  2023-05-12 17:42:26.189824 -0400 EDT m=+0.001652292
printing sensor_reading->sensor_reading
12345
called viam_carto_WriteSensor 2023-05-12 17:42:29.193109 -0400 EDT m=+3.004926042
calling viam_carto_WriteSensor  2023-05-12 17:42:29.193318 -0400 EDT m=+3.005135501
printing sensor_reading->sensor_reading
12345
called viam_carto_WriteSensor 2023-05-12 17:42:32.197735 -0400 EDT m=+6.009540792

```

## TODO:
- [ ] Support compiling on different platforms without polluting build / bin directories
- [ ] Get counter-c to pass valgrind
- [ ] Add asan support
- [ ] Add C unit test
- [ ] Add C integration test
- [X] Add carto C api stubs
- [X] Add valgrind support
