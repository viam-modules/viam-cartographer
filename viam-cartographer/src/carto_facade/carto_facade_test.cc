#include "carto_facade.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace carto_facade {

BOOST_AUTO_TEST_SUITE(CartoFacadeCPPAPI)

BOOST_AUTO_TEST_CASE(CartoFacade_demo) {
    CartoFacade c;
    viam_carto_get_position_response r;
    BOOST_TEST(c.GetPosition(&r) == 0);
    // should leak
    new CartoFacade;
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace carto_facade
}  // namespace viam
