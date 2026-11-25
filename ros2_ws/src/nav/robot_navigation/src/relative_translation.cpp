// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_navigation
{
using nav2_behaviors::ResultStatus;
using nav2_behaviors::Status;
using Odom = nav_msgs::msg::Odometry;

// Separate action names keep this distance profile local to the relative-motion client.
template<class Action, int Direction>
class RelativeTranslation : public nav2_behaviors::DriveOnHeading<Action>
{
  using Base = nav2_behaviors::DriveOnHeading<Action>;
  using Result = typename Action::Result;

public:
  void onConfigure() override
  {
    Base::onConfigure();
    auto node = this->node_.lock();
    auto setting = [&](const char * key, double initial) {
        const auto name = std::string("relative_translation.") + key;
        nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(initial));
        const double value = node->get_parameter(name).as_double();
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::invalid_argument(name + " must be positive and finite");
        }
        return value;
      };
    approach_distance_ = setting("approach_distance_m", 0.15);
    approach_speed_ = setting("approach_speed_mps", 0.10);
    tolerance_ = setting("position_tolerance_m", 0.015);
    coast_time_ = setting("coast_time_s", 0.20);
    if (approach_distance_ <= tolerance_) {
      throw std::invalid_argument("Approach distance must exceed position tolerance");
    }
    subscription_ = node->template create_subscription<Odom>(
      "odometry/filtered", rclcpp::SensorDataQoS(),
      [this](const Odom::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        odom_ = msg;
      });
  }

  void onCleanup() override
  {
    subscription_.reset();
    std::lock_guard<std::mutex> guard(mutex_);
    odom_.reset();
  }

  ResultStatus onRun(const std::shared_ptr<const typename Action::Goal> goal) override
  {
    if (!std::isfinite(goal->target.x) || !std::isfinite(goal->speed) ||
      Direction * goal->target.x <= 0 || Direction * goal->speed <= 0 ||
      goal->time_allowance.sec < 0 ||
      (goal->time_allowance.sec == 0 && goal->time_allowance.nanosec == 0))
    {
      return {Status::FAILED, Result::INVALID_INPUT};
    }
    braking_ = false;
    still_since_ = -1;
    last_stamp_ = -1;
    return Base::onRun(goal);
  }

  ResultStatus onCycleUpdate() override
  {
    const auto now = this->clock_->now();
    if (now > this->end_time_) {
      return fail(Result::TIMEOUT, "Relative translation timed out");
    }
    Odom::ConstSharedPtr obs;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      obs = odom_;
    }
    if (!obs || obs->header.frame_id != this->local_frame_ ||
      obs->child_frame_id != this->robot_base_frame_)
    {
      return fail(Result::TF_ERROR, "Missing odometry in the behavior frames");
    }
    const rclcpp::Time stamp(obs->header.stamp, now.get_clock_type());
    const double age = (now - stamp).seconds();
    if (age < -0.05 || age > 0.25 || stamp.nanoseconds() < last_stamp_) {
      return fail(Result::TF_ERROR, "Stale or out-of-order relative-motion odometry");
    }
    if (last_stamp_ >= 0 && stamp.nanoseconds() - last_stamp_ > 250000000) {
      still_since_ = -1;
    }
    last_stamp_ = stamp.nanoseconds();
    const auto & pose = obs->pose.pose;
    const auto & start = this->initial_pose_.pose;
    const auto & twist = obs->twist.twist;
    const double yaw = tf2::getYaw(start.orientation);
    const double dx = pose.position.x - start.position.x;
    const double dy = pose.position.y - start.position.y;
    const double progress = Direction * (std::cos(yaw) * dx + std::sin(yaw) * dy);
    const double cross_track = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double traveled = std::hypot(dx, dy);
    const double remaining = std::abs(this->command_x_) - progress;
    const double speed = std::hypot(twist.linear.x, twist.linear.y);
    const double heading = tf2::getYaw(pose.orientation);
    if (!std::isfinite(traveled) || !std::isfinite(remaining) || !std::isfinite(heading) ||
      !std::isfinite(speed) || !std::isfinite(twist.angular.z))
    {
      return fail(Result::TF_ERROR, "Invalid relative-motion odometry");
    }
    this->feedback_->distance_traveled = traveled;
    this->action_server_->publish_feedback(this->feedback_);

    const double tolerance = std::min(tolerance_, std::abs(this->command_x_) * 0.15);
    const double coast = std::max(0.0, Direction * twist.linear.x) * coast_time_;
    if (remaining <= std::max(tolerance * 0.5, coast)) {
      braking_ = true;
    }
    if (braking_) {
      this->stopRobot();
      if (speed > 0.02 || std::abs(twist.angular.z) > 0.05) {
        still_since_ = -1;
      } else if (still_since_ < 0) {
        still_since_ = last_stamp_;
      } else if (last_stamp_ - still_since_ >= 200000000) {
        if (std::hypot(remaining, cross_track) <= tolerance) {
          return {Status::SUCCEEDED, Result::NONE};
        }
        if (remaining <= 0 || std::abs(cross_track) >= tolerance) {
          return fail(Result::UNKNOWN, "Stopped outside the target tolerance");
        }
        // Stopped short. Re-approach only in the requested direction.
        braking_ = false;
        still_since_ = -1;
      }
      return {Status::RUNNING, Result::NONE};
    }

    const double cruise = std::min(0.30, std::abs(this->command_speed_));
    const double terminal = std::min(cruise, approach_speed_);
    // Blend into the approach band before reaching it; short moves start in that band.
    const double blend = std::clamp((remaining - approach_distance_) / approach_distance_, 0.0, 1.0);
    const double requested = Direction * (terminal + blend * (cruise - terminal));
    auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd->header.stamp = now;
    cmd->header.frame_id = this->robot_base_frame_;
    cmd->twist.linear.x = requested;
    geometry_msgs::msg::Pose2D local_pose;
    local_pose.x = pose.position.x;
    local_pose.y = pose.position.y;
    local_pose.theta = heading;
    if (!this->isCollisionFree(progress, cmd->twist, local_pose)) {
      return fail(Result::COLLISION_AHEAD, "Collision ahead during relative translation");
    }
    this->vel_pub_->publish(std::move(cmd));
    return {Status::RUNNING, Result::NONE};
  }

private:
  ResultStatus fail(uint16_t code, const char * reason)
  {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "%s", reason);
    return {Status::FAILED, code};
  }

  double approach_distance_{}, approach_speed_{}, tolerance_{}, coast_time_{};
  bool braking_{false};
  int64_t still_since_{-1}, last_stamp_{-1};
  std::mutex mutex_;
  Odom::ConstSharedPtr odom_;
  rclcpp::Subscription<Odom>::SharedPtr subscription_;
};

using RelativeForward = RelativeTranslation<nav2_msgs::action::DriveOnHeading, 1>;
using RelativeBackward = RelativeTranslation<nav2_msgs::action::BackUp, -1>;
}  // namespace robot_navigation

PLUGINLIB_EXPORT_CLASS(robot_navigation::RelativeForward, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(robot_navigation::RelativeBackward, nav2_core::Behavior)
