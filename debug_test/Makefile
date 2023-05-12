CGO_LDFLAGS="-v -Lbin/static -lcounter -lstdc++ -lpthread"
CGO_CFLAGS="-x c++"


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
	CGO_CFLAGS=${CGO_CFLAGS} CGO_LDFLAGS=${CGO_LDFLAGS} go build -o bin/main-go main.go


clean:
	rm -rf bin





