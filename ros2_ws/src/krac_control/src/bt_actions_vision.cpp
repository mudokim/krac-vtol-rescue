#include "krac_control/bt/bt_actions_vision.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include <algorithm>
#include <cmath>

namespace krac_control::bt
{
namespace
{
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

ExecuteSearchPattern::ExecuteSearchPattern(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList ExecuteSearchPattern::providedPorts()
{
  return {BT::InputPort<std::string>("target", std::string(""), ""), BT::InputPort<std::string>("pattern", "spiral_or_lawnmower", ""), BT::InputPort<double>("max_duration_sec", 45.0, "")};
}
BT::NodeStatus ExecuteSearchPattern::onStart()
{
  getInput("max_duration_sec", max_duration_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteSearchPattern::onRunning()
{
  if (ctx_->objectDetected(0.5)) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  const double t = (ctx_->node()->now() - start_time_).seconds();
  if (t > max_duration_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  geometry_msgs::msg::Twist cmd;
  const double speed = 0.2;
  const int phase = static_cast<int>(t / 4.0) % 4;
  if (phase == 0) cmd.linear.y = speed;
  else if (phase == 1) cmd.linear.x = speed;
  else if (phase == 2) cmd.linear.y = -speed;
  else cmd.linear.x = -speed;
  ctx_->publishVelocity(cmd);
  return BT::NodeStatus::RUNNING;
}
void ExecuteSearchPattern::onHalted() { ctx_->publishZeroVelocity(); }

AlignToTarget::AlignToTarget(const std::string& name, const BT::NodeConfiguration& config, const std::string& target_name)
: BT::StatefulActionNode(name, config), ctx_(globalContext()), target_name_(target_name) {}
BT::PortsList AlignToTarget::providedPorts()
{
  return {BT::InputPort<double>("pixel_tolerance_px", 25.0, ""), BT::InputPort<double>("yaw_tolerance_rad", 0.1, ""), BT::InputPort<double>("stable_duration_sec", 1.0, ""), BT::InputPort<double>("timeout_sec", 35.0, "")};
}
BT::NodeStatus AlignToTarget::onStart()
{
  getInput("pixel_tolerance_px", pixel_tolerance_px_);
  getInput("stable_duration_sec", stable_duration_sec_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignToTarget::onRunning()
{
  const auto now = ctx_->node()->now();
  const double elapsed = (now - start_time_).seconds();

  // IMPORTANT: use the same TargetError stream as WaitForPrecisionTarget and
  // precision_lander. The old implementation read legacy camera/target_error
  // (geometry_msgs/Point), while vision_tracker publishes /vision/target_error.
  // That mismatch made WaitForPrecisionTarget succeed and AlignToBasket wait
  // forever with "no sufficiently fresh target".
  constexpr double kFreshAgeSec = 3.0;
  if (!ctx_->targetErrorFresh(kFreshAgeSec)) {
    ctx_->publishZeroVelocity();
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
    if (elapsed > timeout_sec_) {
      RCLCPP_WARN(ctx_->node()->get_logger(),
        "%s timed out: /vision/target_error not fresh for %.1f s",
        name().c_str(), timeout_sec_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto target = ctx_->targetError();
  if (!target.is_detected || target.confidence < 0.20) {
    ctx_->publishZeroVelocity();
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
    return BT::NodeStatus::RUNNING;
  }

  const double err_x = static_cast<double>(target.pixel_err_x);
  const double err_y = static_cast<double>(target.pixel_err_y);
  const double pixel_error = std::hypot(err_x, err_y);

  if (pixel_error <= pixel_tolerance_px_) {
    ctx_->publishZeroVelocity();
    if (stable_start_.nanoseconds() == 0) {
      stable_start_ = now;
      RCLCPP_INFO(ctx_->node()->get_logger(),
        "%s entered deadband: err=(%.1f, %.1f), norm=%.1f px, source=%s, conf=%.2f",
        name().c_str(), err_x, err_y, pixel_error,
        target.source.c_str(), static_cast<double>(target.confidence));
    }
    if ((now - stable_start_).seconds() >= stable_duration_sec_) {
      RCLCPP_INFO(ctx_->node()->get_logger(),
        "%s aligned: norm=%.1f px stable=%.2f s",
        name().c_str(), pixel_error, (now - stable_start_).seconds());
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());

    const double gain_scale = (pixel_error < 140.0) ? 0.40 : 0.75;
    const double max_speed = (pixel_error < 140.0)
      ? std::min(0.18, ctx_->maxAlignSpeed())
      : std::min(0.35, ctx_->maxAlignSpeed());

    geometry_msgs::msg::Twist cmd;
    // Preserve the project's established image-to-body mapping, but use the
    // live TargetError stream. Near the center the smaller gain prevents
    // repeated overshoot.
    cmd.linear.x = clamp(-err_y * ctx_->alignPGain() * gain_scale, -max_speed, max_speed);
    cmd.linear.y = clamp(-err_x * ctx_->alignPGain() * gain_scale, -max_speed, max_speed);
    cmd.linear.z = 0.0;
    ctx_->publishVelocity(cmd);
  }

  RCLCPP_INFO_THROTTLE(ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
    "%s running: err=(%.1f, %.1f) norm=%.1f px tol=%.1f source=%s conf=%.2f elapsed=%.1f/%.1f",
    name().c_str(), err_x, err_y, pixel_error, pixel_tolerance_px_,
    target.source.c_str(), static_cast<double>(target.confidence), elapsed, timeout_sec_);

  if (elapsed > timeout_sec_) {
    ctx_->publishZeroVelocity();
    RCLCPP_WARN(ctx_->node()->get_logger(),
      "%s timed out: final error=%.1f px tolerance=%.1f px",
      name().c_str(), pixel_error, pixel_tolerance_px_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void AlignToTarget::onHalted() { ctx_->publishZeroVelocity(); }

DescendWithAlignment::DescendWithAlignment(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList DescendWithAlignment::providedPorts()
{
  return {BT::InputPort<std::string>("target", "basket", ""), BT::InputPort<double>("descent_step_m", 0.3, ""), BT::InputPort<double>("min_confidence", 0.55, ""), BT::InputPort<double>("max_lost_time_sec", 0.7, ""), BT::InputPort<double>("timeout_sec", 60.0, ""), BT::InputPort<double>("target_altitude_m", 0.8, "")};
}
BT::NodeStatus DescendWithAlignment::onStart()
{
  getInput("target", target_); getInput("min_confidence", min_confidence_);
  getInput("max_lost_time_sec", max_lost_time_sec_); getInput("timeout_sec", timeout_sec_);
  getInput("target_altitude_m", target_altitude_m_);
  start_time_ = ctx_->node()->now();
  last_seen_time_ = start_time_;
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DescendWithAlignment::onRunning()
{
  const bool seen = ctx_->objectDetected(min_confidence_);
  if (seen) last_seen_time_ = ctx_->node()->now();
  if (!seen && (ctx_->node()->now() - last_seen_time_).seconds() > max_lost_time_sec_) {
    ctx_->publishZeroVelocity();
    return BT::NodeStatus::FAILURE;
  }
  if (ctx_->relativeAltitude() <= target_altitude_m_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  auto err = ctx_->visionError();
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(-err.y * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  cmd.linear.y = clamp(-err.x * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  cmd.linear.z = -std::abs(ctx_->descendSpeed());
  ctx_->publishVelocity(cmd);
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::RUNNING;
}
void DescendWithAlignment::onHalted() { ctx_->publishZeroVelocity(); }

LandOnZone::LandOnZone(const std::string& name, const BT::NodeConfiguration& config, bool basket)
: BT::StatefulActionNode(name, config), ctx_(globalContext()), basket_(basket) {}
BT::PortsList LandOnZone::providedPorts()
{
  return {BT::InputPort<double>("target_altitude_m", 0.35, ""), BT::InputPort<double>("handoff_altitude_m", 1.0, ""), BT::InputPort<double>("max_horizontal_error_m", 0.25, ""), BT::InputPort<double>("timeout_sec", 30.0, "")};
}
BT::NodeStatus LandOnZone::onStart()
{
  if (basket_) getInput("target_altitude_m", target_altitude_m_);
  else getInput("handoff_altitude_m", target_altitude_m_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus LandOnZone::onRunning()
{
  if (ctx_->relativeAltitude() <= target_altitude_m_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  geometry_msgs::msg::Twist cmd;
  if (ctx_->objectDetected(0.4)) {
    auto err = ctx_->visionError();
    cmd.linear.x = clamp(-err.y * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
    cmd.linear.y = clamp(-err.x * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  }
  cmd.linear.z = -std::abs(ctx_->descendSpeed()) * 0.6;
  ctx_->publishVelocity(cmd);
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::RUNNING;
}
void LandOnZone::onHalted() { ctx_->publishZeroVelocity(); }

VerifyBasketPicked::VerifyBasketPicked(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList VerifyBasketPicked::providedPorts()
{
  return {
    BT::InputPort<std::string>("method", "lift_contact_visual", ""),
    BT::InputPort<double>("lift_test_height_m", 0.5, ""),
    BT::InputPort<double>("timeout_sec", 12.0, ""),
    BT::InputPort<double>("stable_duration_sec", 1.5, ""),
    BT::InputPort<double>("max_center_drift_px", 60.0, ""),
    BT::InputPort<double>("max_area_change_ratio", 0.35, ""),
    BT::InputPort<bool>("require_contact", false, ""),
    BT::InputPort<bool>("require_visual", true, "")
  };
}
BT::NodeStatus VerifyBasketPicked::onStart()
{
  getInput("lift_test_height_m", lift_test_height_m_);
  getInput("timeout_sec", timeout_sec_);
  getInput("stable_duration_sec", stable_duration_sec_);
  getInput("max_center_drift_px", max_center_drift_px_);
  getInput("max_area_change_ratio", max_area_change_ratio_);
  getInput("require_contact", require_contact_);
  getInput("require_visual", require_visual_);
  start_time_ = ctx_->node()->now();
  evidence_stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  if (!ctx_->gripperClosed() && !ctx_->gripperStubSuccess()) {
    RCLCPP_WARN(ctx_->node()->get_logger(), "VerifyBasketPicked: gripper close command was not confirmed.");
    return BT::NodeStatus::FAILURE;
  }
  const auto target = ctx_->targetError();
  start_center_x_ = target.pixel_err_x;
  start_center_y_ = target.pixel_err_y;
  start_bbox_area_ = target.bbox_area;
  start_altitude_m_ = ctx_->relativeAltitude();
  target_altitude_m_ = start_altitude_m_ + std::max(0.0, lift_test_height_m_);
  ctx_->setHoldCurrentPosition();
  ctx_->setHoldAltitude(target_altitude_m_);
  RCLCPP_INFO(ctx_->node()->get_logger(),
    "VerifyBasketPicked: lift %.2f->%.2fm, contact_required=%d visual_required=%d",
    start_altitude_m_, target_altitude_m_, require_contact_, require_visual_);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus VerifyBasketPicked::onRunning()
{
  if (!ctx_->gripperClosed() && !ctx_->gripperStubSuccess()) return BT::NodeStatus::FAILURE;
  ctx_->setHoldAltitude(target_altitude_m_);
  const bool lift_ok = ctx_->relativeAltitude() >= target_altitude_m_ - 0.15;
  const bool contact_feedback = ctx_->hasGripperContactFeedback();
  const bool contact_ok =
    !require_contact_ || (contact_feedback && ctx_->gripperContact());

  bool visual_ok = !require_visual_;
  const auto target = ctx_->targetError();
  if (require_visual_ && target.is_detected && start_bbox_area_ > 1.0 && target.bbox_area > 1.0) {
    const double center_drift = std::hypot(target.pixel_err_x - start_center_x_, target.pixel_err_y - start_center_y_);
    const double area_change = std::abs(target.bbox_area - start_bbox_area_) / start_bbox_area_;
    visual_ok = center_drift <= max_center_drift_px_ && area_change <= max_area_change_ratio_;
    RCLCPP_INFO_THROTTLE(ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 500,
      "Verify payload: lift=%d contact=%d visual=%d center_drift=%.1fpx area_change=%.2f",
      lift_ok, contact_ok, visual_ok, center_drift, area_change);
  }

  RCLCPP_INFO_THROTTLE(
    ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 500,
    "Verify payload evidence: lift=%d contact_feedback=%d contact=%d visual=%d alt=%.2f target_alt=%.2f",
    lift_ok, contact_feedback, contact_ok, visual_ok,
    ctx_->relativeAltitude(), target_altitude_m_);

  if (require_contact_ && !contact_feedback) {
    RCLCPP_WARN_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
      "VerifyBasketPicked requires contact, but neither /gripper/contact nor /survivor_tray_rep/gripper_state has published.");
  }

  if (lift_ok && contact_ok && visual_ok) {
    if (evidence_stable_start_.nanoseconds() == 0) evidence_stable_start_ = ctx_->node()->now();
    if ((ctx_->node()->now() - evidence_stable_start_).seconds() >= stable_duration_sec_) {
      ctx_->markRescueCompleted();
      RCLCPP_INFO(ctx_->node()->get_logger(), "Payload verification PASSED.");
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    evidence_stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(ctx_->node()->get_logger(), "Payload verification FAILED by timeout.");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void VerifyBasketPicked::onHalted() {}

}  // namespace krac_control::bt
