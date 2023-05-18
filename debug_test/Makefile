# -Wstrict-prototypes  -pedantic
# -Werror
GCC_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -Wmissing-prototypes -std=c99 
GPP_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -Wmissing-prototypes -std=c++17 

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

carto-test-c-build:
	mkdir -p bin
	gcc $(GCC_WARNINGS)  c_cpp/slam/viam_carto.c c_cpp/slam/viam_carto_test.c  -o bin/viam-carto-test-c

carto-test-c: carto-test-c-build
	bin/viam-carto-test-c

carto-test-c-valgrind: carto-test-c-build
	valgrind --leak-check=full -s bin/viam-carto-test-c

carto-test-c-gdb: carto-test-c-build
	 bin/viam-carto-test-c

clean:
	rm -rf bin
	rm -rf c_cpp/build

# NOTE: linux only
valgrind: counter-c
	valgrind --leak-check=full -s bin/counter-c
