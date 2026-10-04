#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"


using namespace std::chrono_literals;

class GpsPublisher : public rclcpp::Node {
    public:
        GpsPublisher() : Node("gps_publisher") {
            publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", rclcpp::SensorDataQoS());
            timer_ = create_wall_timer(200ms, [this]() {publish_position(); });
            RCLCPP_INFO(get_logger(), "Publishing GPS Coordinates.");
        }
    
    private:
        void publish_position() {
            sensor_msgs::msg::NavSatFix message;
            message.header.stamp = now();
            message.header.frame_id = "gps_link";

            message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            // Sample simulated data for the time being
            message.latitude = 38.41801;
            message.longitude = -110.77901;
            message.altitude = 0.0;
            message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

            publisher_->publish(message);
        }

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisher>());
    rclcpp::shutdown();
    return 0;
}
