#include "carto_facade.h"
extern CartoFacade *carto_facade_new() { return carto_facade_cpp_new(); }

extern void carto_facade_delete(CartoFacade *cf) {
  return carto_facade_cpp_delete(cf);
}

extern int carto_facade_get_position(CartoFacade *cf) {
  return carto_facade_cpp_get_position(cf);
}
