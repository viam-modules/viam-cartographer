#include <stdlib.h>
// NOTE: taken from swig generators & modified
typedef long long int64;
typedef struct viam_carto_str {
  char *p;
  int64 n;
} viam_carto_str;

viam_carto_str viam_carto_allocate_string(const char *p, size_t l);
