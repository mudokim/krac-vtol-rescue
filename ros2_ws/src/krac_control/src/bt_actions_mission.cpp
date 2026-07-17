#include "krac_control/bt/bt_actions_mission.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include <cmath>

namespace krac_control::bt
{
namespace
{
constexpr int MAV_VTOL_STATE_FW = 4;
constexpr int MAV_LANDED_STATE_ON_GROUND = 1;
}

FlyToAltitude::FlyToAltitude(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList FlyToAltitude::providedPorts()
{
  return {
    BT::InputPort<double>("target_altitude_m", 10.0, ""),
    BT::InputPort<double>("tolerance_m", 1.0, ""),
    BT::InputPort<double>("max_climb_rate_mps", 1.5, ""),
    BT::InputPort<bool>("hold_latlon", true, ""),
    BT::InputPort<double>("timeout_sec", 60.0, "")
  };
}
BT::NodeStatus FlyToAltitude::onStart()
{
  getInput("target_altitude_m", target_altitude_m_);
  getInput("tolerance_m", tolerance_m_);
  getInput("timeout_sec", timeout_sec_);
  getInput("mode", mode_);
  start_time_ = ctx_->node()->now();
  ctx_->setHoldCurrentPosition();
  ctx_->setHoldAltitude(target_altitude_m_);
  ctx_->startOffboardSetpointStream(20.0, "hold_current_pose");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus FlyToAltitude::onRunning()
{
  ctx_->setHoldAltitude(target_altitude_m_);
  const double alt = ctx_->relativeAltitude();
  const bool altitude_ok =
      (mode_ == "at_or_below" || mode_ == "below_or_equal") ? (alt <= target_altitude_m_ + tolerance_m_) :
      (mode_ == "at_or_above" || mode_ == "above_or_equal") ? (alt >= target_altitude_m_ - tolerance_m_) :
      (std::abs(alt - target_altitude_m_) <= tolerance_m_);
  if (altitude_ok) return BT::NodeStatus::SUCCESS;
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void FlyToAltitude::onHalted() {}

WaitForHoverStable::WaitForHoverStable(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList WaitForHoverStable::providedPorts()
{
  return {
    BT::InputPort<double>("altitude_tolerance_m", 0.5, ""),
    BT::InputPort<double>("max_vertical_speed_mps", 0.3, ""),
    BT::InputPort<double>("max_horizontal_speed_mps", 0.5, ""),
    BT::InputPort<double>("stable_duration_sec", 2.0, ""),
    BT::InputPort<double>("timeout_sec", 15.0, "")
  };
}
BT::NodeStatus WaitForHoverStable::onStart()
{
  getInput("altitude_tolerance_m", altitude_tolerance_m_);
  getInput("max_vertical_speed_mps", max_vertical_speed_mps_);
  getInput("max_horizontal_speed_mps", max_horizontal_speed_mps_);
  getInput("stable_duration_sec", stable_duration_sec_);
  getInput("timeout_sec", timeout_sec_);
  reference_altitude_m_ = ctx_->relativeAltitude();
  start_time_ = ctx_->node()->now();
  stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForHoverStable::onRunning()
{
  const bool stable = std::abs(ctx_->relativeAltitude() - reference_altitude_m_) <= altitude_tolerance_m_ &&
                      std::abs(ctx_->verticalSpeed()) <= max_vertical_speed_mps_ &&
                      ctx_->horizontalSpeed() <= max_horizontal_speed_mps_;
  if (!stable) {
    if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
      RCLCPP_WARN(
        ctx_->node()->get_logger(),
        "WaitForHoverStable timed out: alt=%.2f ref=%.2f vz=%.2f hspeed=%.2f",
        ctx_->relativeAltitude(),
        reference_altitude_m_,
        ctx_->verticalSpeed(),
        ctx_->horizontalSpeed());
      return BT::NodeStatus::FAILURE;
    }
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
    return BT::NodeStatus::RUNNING;
  }
  if (stable_start_.nanoseconds() == 0) stable_start_ = ctx_->node()->now();
  return (ctx_->node()->now() - stable_start_).seconds() >= stable_duration_sec_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}
void WaitForHoverStable::onHalted() {}

EnsureFixedWingCruise::EnsureFixedWingCruise(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList EnsureFixedWingCruise::providedPorts()
{
  return {BT::InputPort<std::string>("transition_source", "mission_plan_or_command", ""), BT::InputPort<double>("timeout_sec", 45.0, "")};
}
BT::NodeStatus EnsureFixedWingCruise::onStart()
{
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus EnsureFixedWingCruise::onRunning()
{
  if (ctx_->vtolState() == MAV_VTOL_STATE_FW) return BT::NodeStatus::SUCCESS;
  if (ctx_->simBypass() && (ctx_->node()->now() - start_time_).seconds() > 2.0) {
    RCLCPP_WARN(ctx_->node()->get_logger(), "EnsureFixedWingCruise sim_bypass=true; accepting FW cruise without FW state.");
    return BT::NodeStatus::SUCCESS;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void EnsureFixedWingCruise::onHalted() {}

EnterStation::EnterStation(const std::string& name, const BT::NodeConfiguration& config, bool home)
: BT::StatefulActionNode(name, config), ctx_(globalContext()), home_(home) {}
BT::PortsList EnterStation::providedPorts()
{
  return {BT::InputPort<double>("approach_radius_m", 120.0, ""), BT::InputPort<double>("station_radius_m", 30.0, ""), BT::InputPort<double>("max_cross_track_m", 60.0, ""), BT::InputPort<double>("timeout_sec", 300.0, "")};
}
BT::NodeStatus EnterStation::onStart()
{
  getInput("approach_radius_m", approach_radius_m_);
  getInput("station_radius_m", station_radius_m_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus EnterStation::onRunning()
{
  const auto target = home_ ? ctx_->homePoint() : ctx_->repPoint();
  if (!target.valid) {
    if (ctx_->simBypass()) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }
  const double d = ctx_->distanceTo(target);
  if (d <= station_radius_m_ || d <= approach_radius_m_) return BT::NodeStatus::SUCCESS;
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void EnterStation::onHalted() {}

DetectLanding::DetectLanding(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList DetectLanding::providedPorts()
{
  return {
    BT::InputPort<double>("max_relative_altitude_m", 0.3, ""),
    BT::InputPort<double>("max_vertical_speed_mps", 0.2, ""),
    BT::InputPort<double>("timeout_sec", 45.0, ""),
    BT::InputPort<bool>("require_px4_landed_state", false,
      "Require MAVROS/PX4 landed_state=ON_GROUND"),
    BT::InputPort<double>("stable_duration_sec", 0.0,
      "How long the landing condition must remain true")
  };
}
BT::NodeStatus DetectLanding::onStart()
{
  getInput("max_relative_altitude_m", max_relative_altitude_m_);
  getInput("max_vertical_speed_mps", max_vertical_speed_mps_);
  getInput("timeout_sec", timeout_sec_);
  getInput("require_px4_landed_state", require_px4_landed_state_);
  getInput("stable_duration_sec", stable_duration_sec_);
  start_time_ = ctx_->node()->now();
  stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DetectLanding::onRunning()
{
  const bool px4_grounded =
    ctx_->landedState() == MAV_LANDED_STATE_ON_GROUND;
  const bool kinematic_landing =
    ctx_->relativeAltitude() <= max_relative_altitude_m_ &&
    std::abs(ctx_->verticalSpeed()) <= max_vertical_speed_mps_;
  const bool landed_ok =
    require_px4_landed_state_ ? px4_grounded : (px4_grounded || kinematic_landing);

  if (landed_ok) {
    if (stable_start_.nanoseconds() == 0) stable_start_ = ctx_->node()->now();
    if ((ctx_->node()->now() - stable_start_).seconds() >= stable_duration_sec_) {
      RCLCPP_INFO(ctx_->node()->get_logger(),
        "DetectLanding confirmed: px4_grounded=%s alt=%.2f vz=%.2f stable=%.2fs",
        px4_grounded ? "true" : "false",
        ctx_->relativeAltitude(), ctx_->verticalSpeed(), stable_duration_sec_);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  }

  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(ctx_->node()->get_logger(),
      "DetectLanding timeout: px4_landed_state=%d alt=%.2f vz=%.2f require_px4=%s",
      ctx_->landedState(), ctx_->relativeAltitude(), ctx_->verticalSpeed(),
      require_px4_landed_state_ ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void DetectLanding::onHalted() {}

VerifyLandingComplete::VerifyLandingComplete(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList VerifyLandingComplete::providedPorts()
{
  return {BT::InputPort<bool>("require_disarmed", true, ""), BT::InputPort<double>("timeout_sec", 20.0, "")};
}
BT::NodeStatus VerifyLandingComplete::onStart()
{
  getInput("require_disarmed", require_disarmed_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus VerifyLandingComplete::onRunning()
{
  const bool landed = ctx_->landedState() == MAV_LANDED_STATE_ON_GROUND || ctx_->relativeAltitude() < 0.4;
  const bool disarmed_ok = !require_disarmed_ || !ctx_->armed();
  if (landed && disarmed_ok) return BT::NodeStatus::SUCCESS;
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void VerifyLandingComplete::onHalted() {}

VerifyPayloadSecured::VerifyPayloadSecured(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList VerifyPayloadSecured::providedPorts() { return {BT::InputPort<std::string>("method", "gripper_feedback_or_visual_invariant", "")}; }
BT::NodeStatus VerifyPayloadSecured::tick()
{
  return (ctx_->gripperClosed() || ctx_->gripperStubSuccess()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace krac_control::bt
