#pragma once

namespace gps_interface
{
  struct GpsFix
  {
    double latitude;
    double longitude;
    double altitude;
    int fix_quality;
  };

  class GpsDriverBase
  {
  public:
    virtual ~GpsDriverBase() = default;
    virtual bool connect() = 0;
    virtual GpsFix readPosition() = 0;
    virtual void close() = 0;
  };
}
