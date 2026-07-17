#include "krac_control/bt/bt_actions_recovery.hpp"
#include "krac_control/bt/bt_conditions.hpp"

#include <algorithm>
#include <cmath>

namespace krac_control::bt
{

IsEmergencyDetected::IsEmergencyDetected(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config),
  ctx_(globalContext())
{
}

BT::PortsList IsEmergencyDetected::providedPorts()
{
  return {
    BT::InputPort<std::string>("sources", std::string(""), "")
  };
}

BT::NodeStatus IsEmergencyDetected::tick()
{
  // v1: emergency는 아직 외부 abort/failsafe와 연결하지 않음.
  // SafetyGuard 실패는 GlobalMissionRecovery 쪽에서 처리.
  return BT::NodeStatus::FAILURE;
}

RecoveryAction::RecoveryAction(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::string& label)
: BT::StatefulActionNode(name, config),
  ctx_(globalContext()),
  label_(label)
{
}

BT::PortsList RecoveryAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("policy", std::string("hold"), ""),
    BT::InputPort<std::string>("reason", std::string(""), ""),
    BT::InputPort<double>("climb_altitude_m", 1.0, ""),
    BT::InputPort<double>("timeout_sec", 20.0, "")
  };
}

BT::NodeStatus RecoveryAction::onStart()
{
  getInput("policy", policy_);
  getInput("climb_altitude_m", climb_altitude_m_);
  getInput("timeout_sec", timeout_sec_);

  const double current_alt = ctx_->relativeAltitude();
  target_altitude_m_ = std::max(current_alt, current_alt + std::max(0.0, climb_altitude_m_));
  start_time_ = ctx_->node()->now();
  ctx_->setPrecisionLanderEnabled(false);
  ctx_->setHoldCurrentPosition();
  ctx_->setHoldAltitude(target_altitude_m_);
  ctx_->startOffboardSetpointStream(20.0, "hold_current_pose");
  RCLCPP_WARN(
    ctx_->node()->get_logger(),
    "%s started. policy=%s current_alt=%.2f target_alt=%.2f",
    label_.c_str(),
    policy_.c_str(),
    current_alt,
    target_altitude_m_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecoveryAction::onRunning()
{
  ctx_->setHoldAltitude(target_altitude_m_);
  const double alt_err = std::abs(ctx_->relativeAltitude() - target_altitude_m_);
  if (alt_err <= 0.7 || (ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    ctx_->publishZeroVelocity();
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "%s completed. alt=%.2f target=%.2f",
      label_.c_str(),
      ctx_->relativeAltitude(),
      target_altitude_m_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void RecoveryAction::onHalted()
{
  ctx_->publishZeroVelocity();
}

HoldRecoveryAction::HoldRecoveryAction(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::string& label)
: BT::StatefulActionNode(name, config),
  ctx_(globalContext()),
  label_(label),
  policy_("hold")
{
}

BT::PortsList HoldRecoveryAction::providedPorts()
{
  return {
    // "hold" = 제자리 위치 홀드(기본, 종전 동작). "land" = AUTO.LAND 로 내려보냄.
    BT::InputPort<std::string>("policy", std::string("hold"), ""),
    BT::InputPort<std::string>("reason", std::string(""), ""),
    BT::InputPort<double>("climb_altitude_m", 1.0, ""),
    BT::InputPort<double>("timeout_sec", 20.0, "")
  };
}

void HoldRecoveryAction::requestLandMode()
{
  if (ctx_->mode() == "AUTO.LAND") return;

  const auto now = ctx_->node()->now();
  if (last_land_request_.nanoseconds() != 0 &&
      (now - last_land_request_).seconds() < 2.0) {
    return;
  }
  last_land_request_ = now;

  auto client = ctx_->setModeClient();
  if (!client->wait_for_service(std::chrono::milliseconds(50))) {
    RCLCPP_WARN_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 5000,
      "%s: SetMode 서비스 없음(MAVROS 끊김?). AUTO.LAND 를 못 건다 — "
      "PX4 자체 offboard-loss failsafe 에 맡긴다.", label_.c_str());
    return;
  }
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = "AUTO.LAND";
  client->async_send_request(req);
}

BT::NodeStatus HoldRecoveryAction::onStart()
{
  getInput("policy", policy_);
  last_land_request_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());

  std::string reason;
  getInput("reason", reason);

  ctx_->setPrecisionLanderEnabled(false);

  if (policy_ == "land") {
    // 스트림을 끊어야 AUTO.LAND 가 온전히 기체를 잡는다. 여기서 offboard
    // setpoint 를 계속 흘리면 착지 후 누가 OFFBOARD 로 되돌리는 순간 다시 뜬다.
    ctx_->stopOffboardSetpointStream();
    RCLCPP_ERROR(
      ctx_->node()->get_logger(),
      "%s: 미션 실패(reason=%s). policy=land -> AUTO.LAND 로 착륙시킨다.",
      label_.c_str(), reason.c_str());
    requestLandMode();
  } else {
    // policy=hold: 제자리 홀드.
    // 종전에는 publishZeroVelocity() 만 불렀는데, 직전 액션이 켜 둔
    // setpoint_raw/global 스트림("마지막 목표고도로 가라")이 그대로 살아 있어서
    // 두 setpoint 소스가 서로 싸웠다(PX4 는 마지막에 도착한 것을 따른다).
    // 현재 위치를 다시 래치해 위치 홀드 하나로 통일한다. RecoveryAction 과 동일한 방식.
    ctx_->setHoldCurrentPosition();
    ctx_->startOffboardSetpointStream(20.0, "hold_current_pose");
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "%s: 미션 실패(reason=%s). policy=hold -> 제자리 위치 홀드. "
      "★자동 착륙하지 않는다. 조종자가 인계받을 것★",
      label_.c_str(), reason.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldRecoveryAction::onRunning()
{
  if (policy_ == "land") {
    requestLandMode();
    RCLCPP_WARN_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 3000,
      "%s active. policy=land. 현재모드=%s (AUTO.LAND 목표).",
      label_.c_str(), ctx_->mode().c_str());
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_WARN_THROTTLE(
    ctx_->node()->get_logger(),
    *ctx_->node()->get_clock(),
    2000,
    "%s active. policy=%s. Mission is held. 자동 착륙 안 함 — 조종자 인계 필요.",
    label_.c_str(),
    policy_.c_str());

  return BT::NodeStatus::RUNNING;
}

void HoldRecoveryAction::onHalted()
{
  RCLCPP_WARN(
    ctx_->node()->get_logger(),
    "%s halted.",
    label_.c_str());
}

}  // namespace krac_control::bt
