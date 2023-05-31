#include "carto_facade.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace carto_facade {

BOOST_AUTO_TEST_SUITE(CartoFacadeCPPAPI)

BOOST_AUTO_TEST_CASE(CartoFacade_demo) {
    CartoFacade c;

    BOOST_TEST(c.Start() == 0);

    viam_carto_get_position_response pr;
    BOOST_TEST(c.GetPosition(&pr) == 0);

    viam_carto_sensor_reading sr;
    BOOST_TEST(c.AddSensorReading(&sr) == 0);

    viam_carto_get_point_cloud_map_response mr;
    BOOST_TEST(c.GetPointCloudMap(&mr) == 0);

    viam_carto_get_internal_state_response isr;
    BOOST_TEST(c.GetInternalState(&isr) == 0);

    BOOST_TEST(c.Stop() == 0);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace carto_facade
}  // namespace viam
