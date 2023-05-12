#include "mycounter.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
  MyCounter *c = c_counter();
  printf("From C:\n");
  for (int i = 0; i < 5; i++) {
    printf("%d counter incremented %d\n", i, c_inc(c, 1));
  }

  return 0;
}
