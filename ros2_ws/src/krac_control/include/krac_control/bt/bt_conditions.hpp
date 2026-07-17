#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

namespace krac_control::bt
{

void setGlobalContext(const std::shared_ptr<MissionContext>& ctx);
std::shared_ptr<MissionContext> globalContext();

class IsMAVROSConnected : public BT::ConditionNode
{
public:
  IsMAVROSConnected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsGPSReady : public BT::ConditionNode
{
public:
  IsGPSReady(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsGlobalPositionFresh : public BT::ConditionNode
{
public:
  IsGlobalPositionFresh(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsBatterySafe : public BT::ConditionNode
{
public:
  IsBatterySafe(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsOffboardSetpointStreamAlive : public BT::ConditionNode
{
public:
  IsOffboardSetpointStreamAlive(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsFlightMode : public BT::ConditionNode
{
public:
  IsFlightMode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsVehicleArmed : public BT::ConditionNode
{
public:
  IsVehicleArmed(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsVTOLStateMC : public BT::ConditionNode
{
public:
  IsVTOLStateMC(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsAltitudeReached : public BT::ConditionNode
{
public:
  IsAltitudeReached(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsMissionUploaded : public BT::ConditionNode
{
public:
  IsMissionUploaded(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsRescueCompleted : public BT::ConditionNode
{
public:
  IsRescueCompleted(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsAtRegion : public BT::ConditionNode
{
public:
  IsAtRegion(const std::string& name, const BT::NodeConfiguration& config, bool home);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool home_;
};
class IsAtREPRegion : public IsAtRegion
{
public:
  IsAtREPRegion(const std::string& name, const BT::NodeConfiguration& config) : IsAtRegion(name, config, false) {}
};
class IsAtHomeRegion : public IsAtRegion
{
public:
  IsAtHomeRegion(const std::string& name, const BT::NodeConfiguration& config) : IsAtRegion(name, config, true) {}
};

class IsVisionTopicFresh : public BT::ConditionNode
{
public:
  IsVisionTopicFresh(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class IsObjectDetected : public BT::ConditionNode
{
public:
  IsObjectDetected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  int stable_count_{0};
};

}  // namespace krac_control::bt
