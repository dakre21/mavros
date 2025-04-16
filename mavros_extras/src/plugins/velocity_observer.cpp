/*
 * TODO - BPRL CU Boulder
 */
/**
 * @brief Velocity Observer plugin
 * @file bprl_velocity_observer.cpp
 * @author David Akre
 *
 * @addtogroup plugin
 * @{
 */

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

namespace mavros {
namespace extra_plugins {
class VelocityObserverPlugin : public plugin::Plugin {
 public:
  explicit VelocityObserverPlugin(plugin::UASPtr uas_)
      : Plugin(uas_, "velocity_observer") {
    velocity_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/velocity", 10,
        std::bind(&VelocityObserverPlugin::velocity_cb, this,
                  std::placeholders::_1));
  }

  Subscriptions get_subscriptions() override { return {/* Rx disabled */}; }

 private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      velocity_sub_;

  void send_velocity_estimate(const uint64_t usec, const Eigen::VectorXd& v,
                              const Eigen::Vector3d w) {
    mavlink::common::msg::ODOMETRY odom{};

    // (dakre) make this unique from odom plugin
    odom.frame_id = 99;
    odom.time_usec = usec;
    odom.x = v.x();
    odom.y = v.y();
    odom.z = v.z();

    // (dakre) bastardizing quaternion with ncams
    const int ncams = w.z();
    odom.q[0] = ncams;

    uas->send_message(odom);
  }

  void velocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    send_velocity_estimate(get_time_usec(msg->header.stamp),
                           ftf::to_eigen(msg->twist.linear),
                           ftf::to_eigen(msg->twist.angular));
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VelocityObserverPlugin)
