#include "mycounter.h"

MyCounter::MyCounter() : count(0) {}
int MyCounter::inc(int a) {
  count += a;
  return count;
}

int cpp_inc(MyCounter *fred, int i) { return fred->inc(i); }
MyCounter *cpp_counter() {
  MyCounter *fred = new MyCounter();
  return fred;
}
