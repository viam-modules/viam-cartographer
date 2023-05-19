/* This header can be read by both C and C++ compilers */
#ifndef CARTO_FACADE_H
#define CARTO_FACADE_H
#ifdef __cplusplus
class CartoFacade {
public:
  CartoFacade();
  int get_position();

private:
  int count;
};
#else
typedef struct CartoFacade {
} CartoFacade;
#endif
#ifdef __cplusplus
extern "C" {
#endif
#if defined(__STDC__) || defined(__cplusplus)
/* ANSI C prototypes */
extern int carto_facade_get_position(CartoFacade *);
extern int carto_facade_cpp_get_position(CartoFacade *);

extern CartoFacade *carto_facade_new();
extern CartoFacade *carto_facade_cpp_new();

extern void carto_facade_delete(CartoFacade *cf);
extern void carto_facade_cpp_delete(CartoFacade *cf);
#else
// UNIMPLEMENTED
/* extern int c_inc(); /1* K&R style *1/ */
/* extern int cpp_inc(); */

/* extern CartoFacade *carto_facade_new(); /1* ANSI C prototypes *1/ */
/* extern CartoFacade *carto_facade_cpp_new(); */
#endif
#ifdef __cplusplus
}
#endif
#endif /*CARTO_FACADE_H*/
