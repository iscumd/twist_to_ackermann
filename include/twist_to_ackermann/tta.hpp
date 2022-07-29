#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optional"
#include "rclcpp/rclcpp.hpp"

namespace tta
{
class TwistToAckermann : public rclcpp::Node
{
public:
  explicit TwistToAckermann(rclcpp::NodeOptions options);

private:
  // Without stamps
  std::optional<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>> _twist_sub =
    std::nullopt;
  std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>> _ack_pub =
    std::nullopt;

  // With stamps
  std::optional<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>>>
    _twists_sub = std::nullopt;
  std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>>>
    _acks_pub = std::nullopt;

  /// Wheelbase in meters.
  double _wheelbase{};

  /// Whether to use stamped messages.
  bool _use_stamps;

  void twist_cb(geometry_msgs::msg::Twist::SharedPtr msg);
  void twists_cb(geometry_msgs::msg::TwistStamped::SharedPtr msg);
};
}  // namespace tta