GCC_WARNINGS=-g -Wall -Wextra -std=c99 -pedantic -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes
GPP_WARNINGS=-g -Wall -Wextra -std=c++17 -pedantic -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes

counter-lib:
	mkdir -p c_cpp/build
	gcc $(GCC_WARNINGS) -c c_cpp/counter/mycounter.c -o c_cpp/build/mycounter.o
	gcc $(GPP_WARNINGS)  -c c_cpp/counter/my_cpp_counter.cpp -o c_cpp/build/my_cpp_counter.o
	ar rcs c_cpp/build/libcounter.a c_cpp/build/my_cpp_counter.o c_cpp/build/mycounter.o

counter-c: counter-lib
	mkdir -p bin
	gcc $(GCC_WARNINGS)  -c c_cpp/counter/main.c -o c_cpp/build/main.o
	g++ $(GPP_WARNINGS)  c_cpp/build/main.o -Lc_cpp/build -lcounter -o bin/counter-c

counter-go: counter-lib
	mkdir -p bin
	go build -o bin/counter-go go/counter/main.go

carto-go:
	mkdir -p bin
	go build -o bin/carto-go go/carto/main.go

clean:
	rm -rf bin
	rm -rf c_cpp/build

# NOTE: linux only
valgrind: counter-c
	valgrind --leak-check=full -s bin/counter-c
