#include "bstrlib.h"
#include "bstrwrap.h"
#include "carto_facade.h"
#include "viam_carto.h"
#include <string>

CartoFacade::CartoFacade() : count(1) {}
viam_carto_get_position_response CartoFacade::get_position() {
  bstring cr = bfromcstr("C++ component reference");
  viam_carto_get_position_response r;
  r.x = 100 * count;
  r.y = 200 * count;
  r.z = 300 * count;
  r.o_x = 400 * count;
  r.o_y = 500 * count;
  r.o_z = 600 * count;
  r.imag = 700 * count;
  r.jmag = 800 * count;
  r.kmag = 900 * count;
  r.theta = 1000 * count;
  r.real = 1100 * count;
  r.component_reference = cr;
  count++;

  return r;
}

viam_carto_get_position_response
carto_facade_cpp_get_position(CartoFacade *cf) {
  return cf->get_position();
}

CartoFacade *carto_facade_cpp_new() {
  // TODO: This needs error handling
  CartoFacade *cf = new CartoFacade();
  return cf;
}

// TODO: This needs error handling
void carto_facade_cpp_delete(CartoFacade *cf) { return delete cf; }
