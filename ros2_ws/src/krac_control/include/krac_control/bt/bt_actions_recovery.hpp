#pragma once

#include "krac_control/bt/mission_context.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

namespace krac_control::bt
{

class IsEmergencyDetected : public BT::ConditionNode
{
public:
  IsEmergencyDetected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionContext> ctx_;
};

// Local recovery용.
// RecoverVisionLanding / RecoverFinalLanding처럼 현재 위치에서 짧게 상승/정지 복구를 수행하고
// SUCCESS로 끝나는 노드. XML에서 바로 뒤에 FailAfterRecovery가 붙으면 RetryUntilSuccessful이
// 원래 landing attempt를 다시 시작한다.
class RecoveryAction : public BT::StatefulActionNode
{
public:
  RecoveryAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::string& label);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  std::shared_ptr<MissionContext> ctx_;
  std::string label_;
  std::string policy_;
  double climb_altitude_m_{1.0};
  double timeout_sec_{20.0};
  double target_altitude_m_{0.0};
  rclcpp::Time start_time_;
};

// Global/Emergency hold용.
// 이 노드들은 RUNNING을 반환해야 하므로 SyncActionNode가 아니라 StatefulActionNode여야 함.
class HoldRecoveryAction : public BT::StatefulActionNode
{
public:
  HoldRecoveryAction(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::string& label);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  void requestLandMode();

  std::shared_ptr<MissionContext> ctx_;
  std::string label_;
  // "hold" = 제자리 위치 홀드(기본). "land" = AUTO.LAND 로 내려보냄.
  std::string policy_;
  rclcpp::Time last_land_request_;
};

class EmergencyRecovery : public HoldRecoveryAction
{
public:
  EmergencyRecovery(const std::string& name, const BT::NodeConfiguration& config)
  : HoldRecoveryAction(name, config, "EmergencyRecovery") {}
};

class GlobalMissionRecovery : public HoldRecoveryAction
{
public:
  GlobalMissionRecovery(const std::string& name, const BT::NodeConfiguration& config)
  : HoldRecoveryAction(name, config, "GlobalMissionRecovery") {}
};

class RecoverVisionLanding : public RecoveryAction
{
public:
  RecoverVisionLanding(const std::string& name, const BT::NodeConfiguration& config)
  : RecoveryAction(name, config, "RecoverVisionLanding") {}
};

class RecoverFinalLanding : public RecoveryAction
{
public:
  RecoverFinalLanding(const std::string& name, const BT::NodeConfiguration& config)
  : RecoveryAction(name, config, "RecoverFinalLanding") {}
};

class FailAfterRecovery : public BT::SyncActionNode
{
public:
  FailAfterRecovery(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override { return BT::NodeStatus::FAILURE; }
};

}  // namespace krac_control::bt
