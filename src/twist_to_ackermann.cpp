#include "twist_to_ackermann/tta.hpp"
#include <memory>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tta {
TwistToAckermann::TwistToAckermann(rclcpp::NodeOptions options)
    : Node("twist_to_ackermann", options) {
  _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/nav_vel", 10,
      std::bind(&TwistToAckermann::twist_cb, this, std::placeholders::_1));
      _ack_pub = this->create_publisher<>()

}

void TwistToAckermann::twist_cb(geometry_msgs::msg::Twist::SharedPtr msg) {

}
} // namespace tta

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto tta_node = std::make_shared<tta::TwistToAckermann>(options);
  exec.add_node(tta_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
