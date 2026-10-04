#pragma once

namespace waypoint_nav {
  struct TargetStatus {
    double distance_meters;
    double bearing_degrees; // compass bearing: 0 = North, 90 = East
    bool reached;
  };

  class WayPointCalculator {
  public:
    WayPointCalculator(double target_latitude, double target_longitude, double arrival_threshold);
    TargetStatus ComputeStatus(double current_latitude, double current_longitude) const;

  private:
    double target_latitude;
    double target_longitude;
    double arrival_threshold;

    static double HaversineDistance(double lat1, double long1, double lat2, double long2); // the shortest "as-the-crow-flies" surface distance between two points on a sphere, using their latitude and longitude
    static double BearingTo(double lat1, double long1, double lat2, double long2);
  };
}
