#include "bstrlib.h"
#include "minunit.h"
#include "viam_carto.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int tests_run = 0;

static char *test_viam_carto_init();
static char *all_tests();

static char *test_viam_carto_init() {
  // setup
  viam_carto *vc;

  // setup sensors list
  int sensors_len = 5;
  const char *sensors_arr[] = {
      "sensor_1", "sensor_2", "sensor_3", "sensor_4", "sensor_5",
  };
  int i = 0;
  // allocate space to hold the 5 pointers to the 5 sensor strings
  bstring *sensors = (bstring *)malloc(sizeof(bstring *) * sensors_len);
  mu_assert("error, sensors != NULL", sensors != NULL);
  mu_assert("error, &sensors[0] != sensors", &sensors[0] == sensors);
  for (i = 0; i < sensors_len; i++) {
    sensors[i] = bfromcstr(sensors_arr[i]);
  }

  // setup rest of structs
  const viam_carto_config c = {.sensors = sensors,
                               .sensors_len = sensors_len,
                               .map_rate_sec = 60,
                               .data_dir = bfromcstr("/tmp/some_tmp_file"),
                               .component_reference =
                                   bfromcstr("some_component_reference"),
                               .mode = VIAM_CARTO_LOCALIZING,
                               .lidar_config = TWO_D};

  const viam_carto_algo_config ac = {
      .optimize_every_n_nodes = 1,
      .num_range_data = 2,
      .missing_data_ray_length = 3.1,
      .max_range = 4.1,
      .min_range = 0.1,
      .max_submaps_to_keep = 6,
      .fresh_submaps_count = 7,
      .min_covered_area = 8.1,
      .min_added_submaps_count = 9,
      .occupied_space_weight = 10.1,
      .translation_weight = 11.1,
      .rotation_weight = 12.1,
  };

  // init
  char *errmsg = NULL;
  int rc = viam_carto_init(&vc, c, ac, &errmsg);
  mu_assert("error, rc != 0", rc == 0);
  mu_assert("error, errmsg != NULL", errmsg == NULL);
  mu_assert("error, vc->sensors_len != 5", vc->sensors_len == c.sensors_len);
  for (i = 0; i < vc->sensors_len; i++) {
    mu_assert("error, biseq(vc->sensors[i], c.sensors[i]) != 1",
              biseq(vc->sensors[i], c.sensors[i]) == 1);
  }

  // free all parameters to init
  rc = bdestroy(c.component_reference);
  mu_assert("error, rc != BSTR_OK", rc == BSTR_OK);

  rc = bdestroy(c.data_dir);
  mu_assert("error, rc != BSTR_OK", rc == BSTR_OK);

  for (i = 0; i < sensors_len; i++) {
    rc = bdestroy(sensors[i]);
    mu_assert("error, rc != BSTR_OK", rc == BSTR_OK);
  }
  free(sensors);

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
