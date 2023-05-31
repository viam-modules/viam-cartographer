#include "carto_facade.h"
#include "viam_carto.h"

extern CartoFacade *carto_facade_new() { return carto_facade_cpp_new(); }

extern void carto_facade_delete(CartoFacade *cf) {
  return carto_facade_cpp_delete(cf);
}

extern viam_carto_get_position_response
carto_facade_get_position(CartoFacade *cf) {
  return carto_facade_cpp_get_position(cf);
}
