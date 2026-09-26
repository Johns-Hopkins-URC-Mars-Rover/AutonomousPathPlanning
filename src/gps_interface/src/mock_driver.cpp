#include "gps_interface/mock_driver.hpp"
#include <cstdlib>

namespace gps_interface
{
  bool MockGpsDriver::connect() {
    return true;
  }

  GpsFix MockGpsDriver::readPosition() {
    double garbage = ((std::rand() % 200) - 100) / 1000000.0;
    return GpsFix{38.4180 + garbage, -110.7790 + garbage, 1400.0, 4};
  }

  void MockGpsDriver::close() {
    return;
  }
}
