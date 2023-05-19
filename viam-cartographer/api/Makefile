# -Wstrict-prototypes  -pedantic
# -Werror
GCC_BSTRING_WARNINGS=-g -Wall -O2 -std=c99 
GCC_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -Wmissing-prototypes -std=c99 -O2
GPP_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -Wmissing-prototypes -std=c++17 -O2

clean:
	rm -rf bin
	rm -rf build

# BEGIN CARTO TARGETS
# TODO
carto-go:
	@mkdir -p bin
	@mkdir -p build
	go build -o bin/carto-go go/carto/main.go

carto-test-c-build:
	@mkdir -p bin
	@mkdir -p build
	gcc $(GCC_BSTRING_WARNINGS)  -c src/slam/bstrlib.c  -o build/bstrlib.o
	ar rcs build/libbstrlib.a build/bstrlib.o
	gcc $(GCC_WARNINGS)  src/slam/viam_carto.c src/slam/viam_carto_test.c -Lbuild -lbstrlib -o bin/viam-carto-test-c

carto-test-c: carto-test-c-build
	bin/viam-carto-test-c

# NOTE: linux only
carto-test-c-valgrind: carto-test-c-build
	valgrind --leak-check=full -s bin/viam-carto-test-c

# NOTE: linux only
carto-test-c-gdb: carto-test-c-build
	gdb --batch --ex run --ex bt --ex q --args bin/viam-carto-test-c


# BEGIN COUNTER TARGETS
counter-lib:
	@mkdir -p build
	gcc $(GCC_WARNINGS) -c src/counter/mycounter.c -o build/mycounter.o
	gcc $(GPP_WARNINGS)  -c src/counter/my_cpp_counter.cpp -o build/my_cpp_counter.o
	ar rcs build/libcounter.a build/my_cpp_counter.o build/mycounter.o

counter-c: counter-lib
	@mkdir -p bin
	gcc $(GCC_WARNINGS)  -c src/counter/main.c -o build/main.o
	g++ $(GPP_WARNINGS)  build/main.o -Lbuild -lcounter -o bin/counter-c

counter-go: counter-lib
	@mkdir -p bin
	go build -o bin/counter-go go/counter/main.go

# NOTE: linux only
counter-c-valgrind: counter-c
	valgrind --leak-check=full -s bin/counter-c
