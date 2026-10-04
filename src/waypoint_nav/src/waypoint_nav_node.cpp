#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "waypoint_nav/waypoint_calculator.hpp"

class WayPointNavNode : public rclcpp::Node {
public:
  WayPointNavNode() : Node("waypoint_nav_node")
  {
    declare_parameter("target_latitude", 38.4180);
    declare_parameter("target_longitude", -110.7790);
    declare_parameter("arrival_threshold", 3.0);

    double target_latitude = get_parameter("target_latitude").as_double();
    double target_longitude = get_parameter("target_longitude").as_double();
    double arrival_threshold = get_parameter("arrival_threshold").as_double();

    calculator_ = std::make_unique<waypoint_nav::WayPointCalculator>(target_latitude, target_longitude, arrival_threshold);
    distance_pub_ = create_publisher<std_msgs::msg::Float64>("nav/distance_to_target", 10); // Publisher for distance to target
    bearing_pub_ = create_publisher<std_msgs::msg::Float64>("nav/bearing_to_target", 10); // Publisher for bearing to target
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("nav/target_reached", 10); // Publisher to signal arrival

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", rclcpp::SensorDataQoS(), std::bind(&WayPointNavNode::OnGpsUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Navigating to (%.6f, %.6f), arrival threshold %.1fm", target_latitude, target_longitude, arrival_threshold);
  }

private:
  void OnGpsUpdate(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto status = calculator_->ComputeStatus(msg->latitude, msg->longitude);

    std_msgs::msg::Float64 distance_msg;
    distance_msg.data = status.distance_meters;
    distance_pub_->publish(distance_msg);

    std_msgs::msg::Float64 bearing_msg;
    bearing_msg.data = status.bearing_degrees;
    bearing_pub_->publish(bearing_msg);

    std_msgs::msg::Bool reached_msg;
    reached_msg.data = status.reached;
    reached_pub_->publish(reached_msg);

    if (status.reached) {
      RCLCPP_INFO(get_logger(), "Target reached! Distance: %.2fm", status.distance_meters);
    }
  }

  std::unique_ptr<waypoint_nav::WayPointCalculator> calculator_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WayPointNavNode>());
  rclcpp::shutdown();
  return 0;
}
