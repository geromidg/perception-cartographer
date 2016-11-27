#include <boost/test/unit_test.hpp>
#include <cartographer/Dummy.hpp>

using namespace cartographer;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    cartographer::DummyClass dummy;
    dummy.welcome();
}
