#ifndef VIAM_CARTO_STRING_H
#define VIAM_CARTO_STRING_H
#include "viam_carto_string.h"
#include <stdlib.h>
#include <string.h>

viam_carto_str viam_carto_allocate_string(const char *p, size_t l) {
  viam_carto_str ret;
  ret.p = (char *)malloc(l);
  memcpy(ret.p, p, l);
  ret.n = l;
  return ret;
}
#endif /*VIAM_CARTO_STRING_H*/
