#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

namespace krac_control::bt
{

// krac24 vtol_fsm.cpp(프록시 FSM)에서 이식: 단일 AUTO.MISSION 플랜 비행 중
// 특정 웨이포인트에서만 OFFBOARD로 개입해 precision_lander에 위임하고,
// 자동 헤딩 계산 후 강제 VTOL 천이로 복귀하는 흐름을 BT 노드로 표현한다.

class IsWaypointReached : public BT::ConditionNode
{
public:
  IsWaypointReached(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class WaitForWaypointReached : public BT::StatefulActionNode
{
public:
  WaitForWaypointReached(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  int seq_{0};
  double timeout_sec_{300.0};
  std::string match_{"at_or_after"};
  uint64_t start_event_count_{0};
  rclcpp::Time start_time_;
};

class SetMissionCurrentWaypoint : public BT::StatefulActionNode
{
public:
  SetMissionCurrentWaypoint(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  int seq_{0};
  double timeout_sec_{5.0};
  rclcpp::Time start_time_;
  rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future_;
  bool request_sent_{false};
};

class EnablePrecisionLander : public BT::SyncActionNode
{
public:
  EnablePrecisionLander(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsPrecisionTargetDetected : public BT::ConditionNode
{
public:
  IsPrecisionTargetDetected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class WaitForPrecisionTarget : public BT::StatefulActionNode
{
public:
  WaitForPrecisionTarget(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double max_age_sec_{2.0};
  double timeout_sec_{30.0};
  rclcpp::Time start_time_;
};

class PrecisionLandOnTarget : public BT::StatefulActionNode
{
public:
  PrecisionLandOnTarget(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double target_altitude_m_{0.3};
  double timeout_sec_{60.0};
  double target_fresh_max_age_sec_{3.0};
  double close_range_lock_alt_m_{2.0};
  bool require_landed_state_{false};
  rclcpp::Time start_time_;
};

class FlyToLocalPoint : public BT::StatefulActionNode
{
public:
  FlyToLocalPoint(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double x_m_{0.0};
  double y_m_{0.0};
  double z_m_{10.0};
  double xy_tolerance_m_{2.0};
  double z_tolerance_m_{2.0};
  double max_xy_speed_mps_{4.0};
  double max_z_speed_mps_{1.0};
  double timeout_sec_{45.0};
  rclcpp::Time start_time_;
  GPSPoint target_gps_;
};

class AlignHeadingToWaypoint : public BT::StatefulActionNode
{
public:
  AlignHeadingToWaypoint(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  int seq_{0};
  double target_yaw_rad_{0.0};
  double tolerance_rad_{0.1};
  double timeout_sec_{8.0};
  rclcpp::Time start_time_;
};

class SetMissionPhase : public BT::SyncActionNode
{
public:
  SetMissionPhase(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class OpenGripper : public BT::StatefulActionNode
{
public:
  OpenGripper(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  std::string left_topic_;
  std::string right_topic_;
  std::string detach_topic_;
  double timeout_sec_{5.0};
  double settle_sec_{1.0};
  double publish_rate_hz_{20.0};
  double left_position_{-1.2};
  double right_position_{-1.2};
  bool detached_{false};
  rclcpp::Time start_time_;
  rclcpp::Time last_publish_time_;
};

}  // namespace krac_control::bt
