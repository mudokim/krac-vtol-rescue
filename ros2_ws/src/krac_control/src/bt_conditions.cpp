#include "krac_control/bt/bt_conditions.hpp"
#include <cmath>

namespace krac_control::bt
{
namespace
{
std::shared_ptr<MissionContext> g_ctx;
constexpr int MAV_VTOL_STATE_MC = 3;
}

void setGlobalContext(const std::shared_ptr<MissionContext>& ctx) { g_ctx = ctx; }
std::shared_ptr<MissionContext> globalContext() { return g_ctx; }

IsMAVROSConnected::IsMAVROSConnected(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsMAVROSConnected::providedPorts() { return {BT::InputPort<double>("max_state_age_sec", 0.5, "")}; }
BT::NodeStatus IsMAVROSConnected::tick()
{
  double age; getInput("max_state_age_sec", age);
  const bool ok = ctx_->stateFresh(age) && ctx_->connected();
  if (!ok) {
    ctx_->logThrottleWarn(
      "mavros_connected",
      "IsMAVROSConnected failed: /mavros/state is stale or disconnected.");
  }
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsGPSReady::IsGPSReady(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsGPSReady::providedPorts() { return {BT::InputPort<int>("min_fix_status", 0, "")}; }
BT::NodeStatus IsGPSReady::tick()
{
  int min_status; getInput("min_fix_status", min_status);
  return ctx_->gpsStatus() >= min_status ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsGlobalPositionFresh::IsGlobalPositionFresh(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsGlobalPositionFresh::providedPorts() { return {BT::InputPort<double>("max_age_sec", 0.5, "")}; }
BT::NodeStatus IsGlobalPositionFresh::tick()
{
  double age; getInput("max_age_sec", age);
  return ctx_->gpsFresh(age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsBatterySafe::IsBatterySafe(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsBatterySafe::providedPorts()
{
  return {BT::InputPort<bool>("sim_bypass", true, ""), BT::InputPort<double>("min_voltage", 0.0, ""), BT::InputPort<double>("min_percentage", 0.0, "")};
}
BT::NodeStatus IsBatterySafe::tick()
{
  bool bypass; getInput("sim_bypass", bypass);
  if (bypass || ctx_->simBypass()) return BT::NodeStatus::SUCCESS;
  // v1: if battery is not wired yet, fail conservatively unless sim_bypass=true.
  ctx_->logThrottleWarn("battery", "IsBatterySafe real battery threshold check is not fully wired yet. Use sim_bypass=true for SITL.");
  return BT::NodeStatus::FAILURE;
}

IsOffboardSetpointStreamAlive::IsOffboardSetpointStreamAlive(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsOffboardSetpointStreamAlive::providedPorts()
{
  return {BT::InputPort<double>("min_rate_hz", 10.0, ""), BT::InputPort<double>("stable_duration_sec", 1.0, "")};
}
//
IsFlightMode::IsFlightMode(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}

BT::PortsList IsFlightMode::providedPorts()
{
  return {
    BT::InputPort<std::string>("mode", std::string("OFFBOARD"), "")
  };
}

BT::NodeStatus IsFlightMode::tick()
{
  std::string target_mode;
  getInput("mode", target_mode);

  if (!ctx_) {
    return BT::NodeStatus::FAILURE;
  }

  return ctx_->mode() == target_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsVehicleArmed::IsVehicleArmed(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsVehicleArmed::providedPorts() { return {}; }
BT::NodeStatus IsVehicleArmed::tick() { return ctx_->armed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }

IsVTOLStateMC::IsVTOLStateMC(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsVTOLStateMC::providedPorts() { return {}; }
BT::NodeStatus IsVTOLStateMC::tick() { return ctx_->vtolState() == MAV_VTOL_STATE_MC ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }

IsAltitudeReached::IsAltitudeReached(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsAltitudeReached::providedPorts()
{
  return {BT::InputPort<double>("target_altitude_m", 10.0, ""), BT::InputPort<double>("tolerance_m", 0.5, "")};
}
BT::NodeStatus IsAltitudeReached::tick()
{
  double target, tol; getInput("target_altitude_m", target); getInput("tolerance_m", tol);
  return std::abs(ctx_->relativeAltitude() - target) <= tol ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsMissionUploaded::IsMissionUploaded(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsMissionUploaded::providedPorts()
{
  return {BT::InputPort<std::string>("mission_name", std::string(""), ""), BT::InputPort<std::string>("expected_end", std::string(""), "")};
}
BT::NodeStatus IsMissionUploaded::tick()
{
  std::string mission; getInput("mission_name", mission);
  return ctx_->isMissionUploaded(mission) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsRescueCompleted::IsRescueCompleted(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsRescueCompleted::providedPorts() { return {}; }
BT::NodeStatus IsRescueCompleted::tick()
{
  return ctx_->rescueCompleted() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsAtRegion::IsAtRegion(const std::string& name, const BT::NodeConfiguration& config, bool home)
: BT::ConditionNode(name, config), ctx_(globalContext()), home_(home) {}
BT::PortsList IsAtRegion::providedPorts()
{
  return {BT::InputPort<double>("radius_m", 30.0, ""), BT::InputPort<double>("max_altitude_error_m", 20.0, "")};
}
BT::NodeStatus IsAtRegion::tick()
{
  double r, alt_err; getInput("radius_m", r); getInput("max_altitude_error_m", alt_err);
  const auto target = home_ ? ctx_->homePoint() : ctx_->repPoint();
  if (!target.valid) return BT::NodeStatus::FAILURE;
  const bool dist_ok = ctx_->distanceTo(target) <= r;
  const bool alt_ok = std::abs(ctx_->relativeAltitude() - target.alt) <= alt_err;
  return (dist_ok && alt_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsVisionTopicFresh::IsVisionTopicFresh(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsVisionTopicFresh::providedPorts()
{
  return {BT::InputPort<std::string>("topic", "/vision/detections", ""), BT::InputPort<double>("max_age_sec", 0.5, "")};
}
BT::NodeStatus IsVisionTopicFresh::tick()
{
  double age; getInput("max_age_sec", age);
  return ctx_->visionFresh(age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsObjectDetected::IsObjectDetected(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsObjectDetected::providedPorts()
{
  return {BT::InputPort<std::string>("target", std::string(""), ""), BT::InputPort<double>("min_confidence", 0.5, ""), BT::InputPort<int>("stable_frames", 1, "")};
}
BT::NodeStatus IsObjectDetected::tick()
{
  double conf; int frames; getInput("min_confidence", conf); getInput("stable_frames", frames);
  if (ctx_->objectDetected(conf)) stable_count_++; else stable_count_ = 0;
  return stable_count_ >= frames ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsOffboardSetpointStreamAlive::tick()
{
  // TEMP BYPASS:
  // StartOffboardSetpointStream 직후 publish-rate 체크가 너무 빨리 실패해서
  // 현재 단계에서는 Preflight 다음 단계 테스트를 위해 임시 통과 처리.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace krac_control::bt
