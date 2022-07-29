#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "twist_to_ackermann/tta.hpp"

namespace tta
{
/// Finds the appropriate steering angle for some turning radius, which is defined by the
/// transient velocity of the vehicle along that curve (just the current velocity in this instance),
/// and the rate that curve changes (angular z rotation). This angle is effected by vehicle wheelbase.
///
/// All units are in m/s or r/s.
double convert_trans_rot_vel_to_steering_angle(double vel, double omega, double wheelbase)
{
  if (omega == 0 || vel == 0) {
    return 0;
  }

  // Remove negative so steering doesn't reverse when reversing.
  vel = std::abs(vel);

  auto rad = vel / omega;
  return std::atan(wheelbase / rad);
}

TwistToAckermann::TwistToAckermann(rclcpp::NodeOptions options)
: Node("twist_to_ackermann", options)
{
  _use_stamps = this->declare_parameter("use_stamps", false);

  if (_use_stamps) {
    _twists_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/nav_vel", 10, std::bind(&TwistToAckermann::twists_cb, this, std::placeholders::_1));

    _acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ack_vel", 10);
  } else {
    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/nav_vel", 10, std::bind(&TwistToAckermann::twist_cb, this, std::placeholders::_1));

    _ack_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 10);
  }

  _wheelbase = this->declare_parameter("wheelbase", 1.0);
}

void TwistToAckermann::twist_cb(geometry_msgs::msg::Twist::SharedPtr msg)
{
  ackermann_msgs::msg::AckermannDrive out{};
  out.speed = msg->linear.x;
  out.steering_angle =
    convert_trans_rot_vel_to_steering_angle(msg->linear.x, msg->angular.z, this->_wheelbase);

  this->_ack_pub->get()->publish(out);
}

void TwistToAckermann::twists_cb(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  ackermann_msgs::msg::AckermannDriveStamped out{};
  out.drive.speed = msg->twist.linear.x;
  out.drive.steering_angle = convert_trans_rot_vel_to_steering_angle(
    msg->twist.linear.x, msg->twist.angular.z, this->_wheelbase);

  this->_acks_pub->get()->publish(out);
}

}  // namespace tta

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto tta_node = std::make_shared<tta::TwistToAckermann>(options);
  exec.add_node(tta_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
