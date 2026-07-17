#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/action_node.h>

namespace krac_control::bt
{

class FlyToAltitude : public BT::StatefulActionNode
{
public:
  FlyToAltitude(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double target_altitude_m_{10.0};
  double tolerance_m_{1.0};
  double timeout_sec_{60.0};
  std::string mode_{"near"};
  rclcpp::Time start_time_;
};

class WaitForHoverStable : public BT::StatefulActionNode
{
public:
  WaitForHoverStable(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double altitude_tolerance_m_{0.5};
  double max_vertical_speed_mps_{0.3};
  double max_horizontal_speed_mps_{0.5};
  double stable_duration_sec_{2.0};
  double timeout_sec_{15.0};
  double reference_altitude_m_{0.0};
  rclcpp::Time start_time_;
  rclcpp::Time stable_start_;
};

class EnsureFixedWingCruise : public BT::StatefulActionNode
{
public:
  EnsureFixedWingCruise(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double timeout_sec_{45.0};
  rclcpp::Time start_time_;
};

class EnterStation : public BT::StatefulActionNode
{
public:
  EnterStation(const std::string& name, const BT::NodeConfiguration& config, bool home);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool home_;
  double approach_radius_m_{120.0};
  double station_radius_m_{30.0};
  double timeout_sec_{300.0};
  rclcpp::Time start_time_;
};

class EnterREPStation : public EnterStation
{
public:
  EnterREPStation(const std::string& name, const BT::NodeConfiguration& config) : EnterStation(name, config, false) {}
};

class EnterHomeStation : public EnterStation
{
public:
  EnterHomeStation(const std::string& name, const BT::NodeConfiguration& config) : EnterStation(name, config, true) {}
};

class DetectLanding : public BT::StatefulActionNode
{
public:
  DetectLanding(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double max_relative_altitude_m_{0.3};
  double max_vertical_speed_mps_{0.2};
  double timeout_sec_{45.0};
  bool require_px4_landed_state_{false};
  double stable_duration_sec_{0.0};
  rclcpp::Time start_time_;
  rclcpp::Time stable_start_;
};

class VerifyLandingComplete : public BT::StatefulActionNode
{
public:
  VerifyLandingComplete(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool require_disarmed_{true};
  double timeout_sec_{20.0};
  rclcpp::Time start_time_;
};

class VerifyPayloadSecured : public BT::SyncActionNode
{
public:
  VerifyPayloadSecured(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

}  // namespace krac_control::bt
