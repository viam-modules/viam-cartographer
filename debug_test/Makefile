

static:
	mkdir -p bin/static
	gcc -c mycounter.c -o bin/static/mycounter.o
	gcc -c my_cpp_counter.cpp -o bin/static/my_cpp_counter.o
	ar rcs bin/static/libcounter.a bin/static/my_cpp_counter.o bin/static/mycounter.o

c: static
	mkdir -p bin
	gcc -c main.c -o bin/main.o
	g++ bin/main.o -Lbin/static -lcounter -o bin/main-c

go: static
	mkdir -p bin
	go build -o bin/main-go main.go


clean:
	rm -rf bin





