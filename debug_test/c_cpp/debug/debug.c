#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void Hello() { printf("FROM WITHIN C PROGRAM: Hello world\n"); }

int sum(int a, int b) { return a + b; }

void reverse(char *x, int begin, int end) {
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

// https://valgrind.org/docs/manual/quick-start.html
void trip_valgrind(void) {
  int *x = malloc(10 * sizeof(int)); // problem 1: heap block overrun
  x[10] = 0;                         // problem 2: memory leak -- x not freed
}

// https://github.com/google/sanitizers/wiki/AddressSanitizerExampleUseAfterReturn
int *ptr;
__attribute__((noinline)) void FunctionThatEscapesLocalObject() {
  int local[100];
  ptr = &local[0];
}

int trip_address_sanitizer() {
  int x = 1;
  FunctionThatEscapesLocalObject();
  return ptr[x];
}
