/* This header can be read by both C and C++ compilers */
#ifndef CARTO_FACADE_H
#define CARTO_FACADE_H
#include "viam_carto.h"
#ifdef __cplusplus
class CartoFacade {
public:
  CartoFacade();
  viam_carto_get_position_response get_position();

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
extern CartoFacade *carto_facade_new();
extern CartoFacade *carto_facade_cpp_new();

extern void carto_facade_delete(CartoFacade *cf);
extern void carto_facade_cpp_delete(CartoFacade *cf);

extern viam_carto_get_position_response
carto_facade_get_position(CartoFacade *);
extern viam_carto_get_position_response
carto_facade_cpp_get_position(CartoFacade *);

#else
// UNIMPLEMENTED
/* extern CartoFacade *carto_facade_new(); */
/* extern CartoFacade *carto_facade_cpp_new(); */

/* extern void carto_facade_delete(CartoFacade *cf); */
/* extern void carto_facade_cpp_delete(CartoFacade *cf); */

/* extern int carto_facade_get_position(CartoFacade *); */
/* extern int carto_facade_cpp_get_position(CartoFacade *); */
#endif
#ifdef __cplusplus
}
#endif
#endif /*CARTO_FACADE_H*/
