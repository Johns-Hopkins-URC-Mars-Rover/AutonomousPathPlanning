#pragma once
#include "gps_interface/gps_driver_base.hpp"

namespace gps_interface
{
  class MockGpsDriver: public GpsDriverBase
  {
  public:
    bool connect() override;
    GpsFix readPosition() override;
    void close() override;
  };
}
