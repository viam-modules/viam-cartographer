#include "minunit.h"
#include "viam_carto.h"
#include <stdio.h>

int tests_run = 0;

static char *test_viam_carto_init();
static char *all_tests();

static char *test_viam_carto_init() {
  // setup
  viam_carto *vc;
  const char *sensors[] = {
      "sensor_1", "sensor_2", "sensor_3", "sensor_4", "sensor_5",
  };
  const viam_carto_config c = {.sensors_len = 5, .sensors = sensors};
  const viam_carto_algo_config ac = {};

  // init
  char *errmsg = NULL;
  int rc = viam_carto_init(&vc, c, ac, &errmsg);
  mu_assert("error, rc != 0", rc == 0);
  mu_assert("error, errmsg != NULL", errmsg == NULL);

  mu_assert("error, vc->sensors_len != 5", vc->sensors_len == 5);

  // terminate
  rc = viam_carto_terminate(&vc, &errmsg);
  mu_assert("error, rc != 0", rc == 0);
  mu_assert("error, errmsg != NULL", errmsg == NULL);
  return 0;
}
static char *all_tests() {
  mu_run_test(test_viam_carto_init);
  return 0;
}

int main(int argc, char **argv) {
  UNUSED(argc);
  UNUSED(argv);
  char *result = all_tests();
  if (result != 0) {
    printf("%s\n", result);
  } else {
    printf("ALL TESTS PASSED\n");
  }
  printf("Tests run: %d\n", tests_run);

  return result != 0;
}
