#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

using namespace std::chrono_literals;

class GpsSubscriber : public rclcpp::Node
{
public:
    GpsSubscriber() : Node("gps_subscriber"), last_received_(std::chrono::steady_clock::now()){
        subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr message) {handle_position(*message);});
        watchdog_ = create_wall_timer(500ms, [this]() { check_timeout(); });
        RCLCPP_INFO(get_logger(), "Waiting for GPS messages...");
    }

private:
    void handle_position(const sensor_msgs::msg::NavSatFix & message) {
        last_received_ = std::chrono::steady_clock::now();
        received_any_ = true;

        if (timeout_reported_) {
            RCLCPP_INFO(get_logger(), "GPS messages resumed");
            timeout_reported_ = false;
        }

        const bool valid = message.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX && std::isfinite(message.latitude) &&  std::isfinite(message.longitude) && message.latitude >= -90.0 &&
            message.latitude <= 90.0 && message.longitude >= -180.0 && message.longitude <= 180.0;

        if (!valid) {
            RCLCPP_WARN(get_logger(), "GPS message received, but fix is invalid");
            return;
        }

        RCLCPP_INFO(get_logger(), "Latitude: %.7f | Longitude: %.7f | Altitude: %.2f m", message.latitude, message.longitude, message.altitude);
    }

    void check_timeout() {
        const auto elapsed = std::chrono::steady_clock::now() - last_received_;

        if (elapsed > 3s && !timeout_reported_) {
            RCLCPP_WARN(get_logger(), "%s", received_any_ ? "GPS stream stale: no updates for over 3 seconds" : "No GPS messages received after 3 seconds");
            timeout_reported_ = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr watchdog_;

    std::chrono::steady_clock::time_point last_received_;
    bool received_any_ = false;
    bool timeout_reported_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsSubscriber>());
    rclcpp::shutdown();
    return 0;
}