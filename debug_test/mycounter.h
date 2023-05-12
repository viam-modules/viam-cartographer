/* This header can be read by both C and C++ compilers */
#ifndef MYCOUNTER_H
#define MYCOUNTER_H
#ifdef __cplusplus
class MyCounter {
public:
  MyCounter();
  int inc(int);

private:
  int count;
};
#else
typedef struct MyCounter MyCounter;
#endif
#ifdef __cplusplus
extern "C" {
#endif
#if defined(__STDC__) || defined(__cplusplus)
extern int c_inc(MyCounter *, int i); /* ANSI C prototypes */
extern int cpp_inc(MyCounter *, int i);

extern MyCounter *c_counter(); /* ANSI C prototypes */
extern MyCounter *cpp_counter();
#else
extern int c_inc(); /* K&R style */
extern int cpp_inc();

extern MyCounter *c_counter(); /* ANSI C prototypes */
extern MyCounter *cpp_counter();
#endif
#ifdef __cplusplus
}
#endif
#endif /*MYCOUNTER_H*/
