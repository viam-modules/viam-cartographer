#include "mycounter.h"
#include <stdio.h>

int main(int argc, char *argv[]) {
  MyCounter *c = c_counter();
  for (int i = 0; i < 5; i++) {
    printf("%d counter incremented %d\n", i, c_inc(c, 1));
  }

  return 0;
}
