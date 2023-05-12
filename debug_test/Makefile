all:
	mkdir -p bin
	mkdir -p bin/static
	gcc -c main.c -o bin/main.o
	gcc -c mycounter.c -o bin/static/mycounter.o
	gcc -c my_cpp_counter.cpp -o bin/static/my_cpp_counter.o
	ar rcs bin/static/libcounter.a bin/static/my_cpp_counter.o bin/static/mycounter.o
	g++ bin/main.o -Lbin/static -lcounter -o bin/main

clean:
	rm -rf bin





