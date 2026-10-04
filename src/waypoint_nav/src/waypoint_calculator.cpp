#include "waypoint_nav/waypoint_calculator.hpp"
#include <cmath>


namespace waypoint_nav {
  namespace {
    constexpr double EARTH_RADIUS_METERS = 6371000.0;
    double ToRadians(double degrees) { return degrees * M_PI / 180.0; }
    double ToDegrees(double radians) { return radians * 180.0 / M_PI; }
  }

  WayPointCalculator::WayPointCalculator(double target_latitude, double target_longitude, double arrival_threshold): target_latitude(target_latitude), target_longitude(target_longitude), arrival_threshold(arrival_threshold) {}

  double WayPointCalculator::HaversineDistance(double lat1, double long1, double lat2, double long2) {
    double phi1 = ToRadians(lat1), phi2 = ToRadians(lat2);
    double d_phi = ToRadians(lat2 - lat1), d_lambda = ToRadians(long2 - long1);
    
    double a = std::sin(d_phi / 2) * std::sin(d_phi / 2) + std::cos(phi1) * std::cos(phi2) * std::sin(d_lambda / 2) * std::sin(d_lambda / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return EARTH_RADIUS_METERS * c;
  }

  double WayPointCalculator::BearingTo(double lat1, double long1, double lat2, double long2) {
    double phi1 = ToRadians(lat1), phi2 = ToRadians(lat2);
    double d_lambda = ToRadians(long2 - long1);

    double y = std::sin(d_lambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(d_lambda);
    double bearing = ToDegrees(std::atan2(y, x));

    return std::fmod(bearing + 360.0, 360.0);  // normalize to 0-360
  }

  TargetStatus WayPointCalculator::ComputeStatus(double current_latitude, double current_longitude) const {
    TargetStatus status;
    status.distance_meters = HaversineDistance(current_latitude, current_longitude, target_latitude, target_longitude);
    status.bearing_degrees = BearingTo(current_latitude, current_longitude, target_latitude, target_longitude);
    status.reached = status.distance_meters <= arrival_threshold;
    return status;
  }
}
