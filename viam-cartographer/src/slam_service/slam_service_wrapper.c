#include <string.h>
#include <stdio.h>

void Hello() {
    printf("FROM WITHIN C PROGRAM: Hello world\n");
}

int sum(int a, int b) {
    return a+b;
}

void reverse (char *x, int begin, int end) {
    char c;
    if (begin >= end)
        return;
    c = *(x + begin);
    *(x + begin) = *(x + end);
    *(x + end) = c;

    reverse(x, ++begin, --end);
    printf("FROM WITHIN C PROGRAM:\n");
    puts(x);
}
