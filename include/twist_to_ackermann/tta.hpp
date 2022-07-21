#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/subscription.hpp>

namespace tta {
class TwistToAckermann : public rclcpp::Node {
public:
  explicit TwistToAckermann(rclcpp::NodeOptions options);

private:
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _twist_sub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _ack_pub;
  
  void twist_cb(geometry_msgs::msg::Twist::SharedPtr msg);
};
} // namespace tta