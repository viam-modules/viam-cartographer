#include "carto_facade.h"

CartoFacade::CartoFacade() : count(1) {}
int CartoFacade::get_position() { return count; }

int carto_facade_cpp_get_position(CartoFacade *cf) {
  return cf->get_position();
}

CartoFacade *carto_facade_cpp_new() {
  // TODO: This needs error handling
  CartoFacade *cf = new CartoFacade();
  return cf;
}

// TODO: This needs error handling
void carto_facade_cpp_delete(CartoFacade *cf) { return delete cf; }
