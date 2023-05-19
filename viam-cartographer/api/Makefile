# -Wstrict-prototypes  -pedantic
# -Werror
GCC_BSTRING_WARNINGS=-g -Wall -O2 -std=c99 
GCC_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -Wmissing-prototypes -std=c99 -O2
GPP_WARNINGS=-g -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -std=c++17 -O2

carto-test-c: carto-test-c-build
	bin/viam-carto-test-c

# NOTE: linux only
carto-test-c-valgrind: carto-test-c-build
	valgrind --leak-check=full -s bin/viam-carto-test-c

# NOTE: linux only
carto-test-c-gdb: carto-test-c-build
	gdb --batch --ex run --ex bt --ex q --args bin/viam-carto-test-c


clean:
	rm -rf bin
	rm -rf build

# BEGIN CARTO TARGETS
# TODO
carto-go: build-setup
	go build -o bin/carto-go go/carto/main.go

build-setup:
	@mkdir -p bin
	@mkdir -p build

carto-test-c-build: build-setup
	# compile bstrlib
	gcc $(GCC_BSTRING_WARNINGS)  -c src/slam/bstrlib.c  -o build/bstrlib.o
	# archive bstrlib static library
	ar rcs build/libbstrlib.a build/bstrlib.o
	# compile carto_facade c interface
	gcc $(GCC_WARNINGS) -c src/slam/carto_facade.c -o build/carto_facade.o
	# compile carto_facade c++ interface
	gcc $(GPP_WARNINGS) -c src/slam/carto_facade_cpp.cpp -o build/carto_facade_cpp.o
	# make archive containing both C & CPP code
	ar rcs build/libcarto_facade.a build/carto_facade_cpp.o build/carto_facade.o
	# compile viam_carto
	gcc $(GCC_WARNINGS) -c src/slam/viam_carto.c -o build/viam_carto.o
	# compile viam_carto_test
	gcc $(GCC_WARNINGS) -c src/slam/viam_carto_test.c -o build/viam_carto_test.o
	# link viam_carto with viam_carto_test and both the bstrlib & carto_facade static libs
	g++ $(GPP_WARNINGS) build/viam_carto_test.o build/viam_carto.o -Lbuild -lbstrlib -lcarto_facade -o bin/viam-carto-test-c


# BEGIN COUNTER TARGETS
counter-lib: build-setup
	gcc $(GCC_WARNINGS) -c src/counter/mycounter.c -o build/mycounter.o
	gcc $(GPP_WARNINGS)  -c src/counter/my_cpp_counter.cpp -o build/my_cpp_counter.o
	ar rcs build/libcounter.a build/my_cpp_counter.o build/mycounter.o

counter-c: counter-lib 
	gcc $(GCC_WARNINGS)  -c src/counter/main.c -o build/main.o
	g++ $(GPP_WARNINGS)  build/main.o -Lbuild -lcounter -o bin/counter-c

counter-go: counter-lib
	go build -o bin/counter-go go/counter/main.go

# NOTE: linux only
counter-c-valgrind: counter-c
	valgrind --leak-check=full -s bin/counter-c
