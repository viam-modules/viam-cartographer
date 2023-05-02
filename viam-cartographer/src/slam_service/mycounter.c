#include "mycounter.h"
extern MyCounter *c_counter() { return cpp_counter(); }
extern int c_inc(MyCounter *fred, int i) { return cpp_inc(fred, i); }
