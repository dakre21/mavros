/*
 * TODO - BPRL CU Boulder
 */
/**
 * @brief JoystickToAttitude Plugin
 * @file joystick_to_attitude.cpp
 * @author David Akre
 *
 * @addtogroup plugin
 * @{
 */

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace mavros {
namespace extra_plugins {
class JoystickToAttitudePlugin : public plugin::Plugin {
 public:
  explicit JoystickToAttitudePlugin(plugin::UASPtr uas_)
      : Plugin(uas_, "joystick_to_attitude") {
    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "~/joy", 10,
        std::bind(&JoystickToAttitudePlugin::joy_cb, this,
                  std::placeholders::_1));
    attitude_pub_ = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);

    enable_node_watch_parameters();
    node_declare_and_watch_parameter(
        "max_thrust", 1.0,
        [&](const rclcpp::Parameter& p) { max_thrust_ = p.as_double(); });

    node_declare_and_watch_parameter(
        "max_roll", M_PI / 4,
        [&](const rclcpp::Parameter& p) { max_roll_ = p.as_double(); });

    node_declare_and_watch_parameter(
        "max_pitch", M_PI / 4,
        [&](const rclcpp::Parameter& p) { max_pitch_ = p.as_double(); });

    node_declare_and_watch_parameter(
        "max_yaw", M_PI,
        [&](const rclcpp::Parameter& p) { max_yaw_ = p.as_double(); });
  }

  Subscriptions get_subscriptions() override { return {/* Rx disabled */}; }

 private:
  double max_thrust_, max_yaw_, max_pitch_, max_roll_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
  std::vector<double> euler_to_quat(const double& phi, const double& theta,
                                    const double& psi) {
    const auto cr = cos(phi / 2);
    const auto sr = sin(phi / 2);
    const auto cp = cos(theta / 2);
    const auto sp = sin(theta / 2);
    const auto cy = cos(psi / 2);
    const auto sy = sin(psi / 2);
    const auto w = cr * cp * cy + sr * sp * sy;
    const auto x = sr * cp * cy - cr * sp * sy;
    const auto y = cr * sp * cy + sr * cp * sy;
    const auto z = cr * cp * sy - sr * sp * cy;
    return std::vector<double>{w, x, y, z};
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto joy_to_angle = [](const auto& input, const auto& max_angle) {
      return max_angle * input;
    };
    auto joy_to_thrust = [this](const auto& input) {
      return input < 0 ? 0 : input * max_thrust_;
    };

    const auto left_thumb_horizontal = msg->axes[0];
    const auto left_thumb_vertical = msg->axes[1];
    const auto right_thumb_horizontal = msg->axes[3];
    const auto right_thumb_vertical = msg->axes[4];

    const auto roll_angle = joy_to_angle(left_thumb_horizontal, max_roll_);
    const auto pitch_angle = joy_to_angle(left_thumb_vertical, max_pitch_);
    const auto thrust = joy_to_thrust(right_thumb_vertical);
    const auto yaw_angle = joy_to_angle(right_thumb_horizontal, max_yaw_);

    const auto quat = euler_to_quat(roll_angle, pitch_angle, yaw_angle);
    mavros_msgs::msg::AttitudeTarget att_msg;
    att_msg.header.stamp = this->get_clock()->now();
    att_msg.type_mask = 7;
    att_msg.orientation.w = quat[0];
    att_msg.orientation.x = quat[1];
    att_msg.orientation.y = quat[2];
    att_msg.orientation.z = quat[3];
    att_msg.thrust = thrust;

    attitude_pub_->publish(att_msg);
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::JoystickToAttitudePlugin)
