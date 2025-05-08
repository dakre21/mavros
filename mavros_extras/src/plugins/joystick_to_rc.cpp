/*
 * TODO - BPRL CU Boulder
 */
/**
 * @brief JoystickToRC Plugin
 * @file joystick_to_rc.cpp
 * @author David Akre
 *
 * @addtogroup plugin
 * @{
 */

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace mavros {
namespace extra_plugins {
class JoystickToRCPlugin : public plugin::Plugin {
 public:
  explicit JoystickToRCPlugin(plugin::UASPtr uas_)
      : Plugin(uas_, "joystick_to_rc") {
    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "~/joy", 10,
        std::bind(&JoystickToRCPlugin::joy_cb, this, std::placeholders::_1));
    rc_override_pub_ = node->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);
  }

  Subscriptions get_subscriptions() override { return {/* Rx disabled */}; }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto joy_to_pwm = [](const auto& input) { return int(1500 + input * 400); };

    const auto left_thumb_horizontal = msg->axes[0];
    const auto left_thumb_vertical = msg->axes[1];
    const auto right_thumb_horizontal = msg->axes[3];
    const auto right_thumb_vertical = msg->axes[4];

    const auto roll_pwm = joy_to_pwm(left_thumb_horizontal);
    const auto pitch_pwm = joy_to_pwm(left_thumb_vertical);
    const auto throttle_pwm = joy_to_pwm(right_thumb_vertical);
    const auto yaw_pwm = joy_to_pwm(right_thumb_horizontal);

    mavros_msgs::msg::OverrideRCIn out_msg;

    out_msg.channels[0] = roll_pwm;
    out_msg.channels[1] = pitch_pwm;
    out_msg.channels[2] = throttle_pwm;
    out_msg.channels[3] = yaw_pwm;
    out_msg.channels[4] = 0;
    out_msg.channels[5] = 0;
    out_msg.channels[6] = 0;
    out_msg.channels[7] = 0;

    rc_override_pub_->publish(out_msg);
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::JoystickToRCPlugin)
