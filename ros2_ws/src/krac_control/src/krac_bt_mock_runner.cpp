#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

struct MockRuntime
{
  int tick_count{0};
  bool verbose{false};
  int default_action_ticks{1};
  std::string fail_node{""};
  int fail_after_tick{0};
  bool fail_once{true};
  bool fail_already_used{false};
  int emergency_at_tick{0};
  rclcpp::Logger logger{rclcpp::get_logger("krac_bt_mock_runner")};

  std::unordered_map<std::string, int> default_ticks_by_node{
    {"EmergencyRecovery", 5},
    {"GlobalMissionRecovery", 5},
    {"StartOffboardSetpointStream", 2},
    {"SetFlightMode", 3},
    {"ArmVehicle", 3},
    {"CommandVTOLTransition", 8},
    {"FlyToAltitude", 20},
    {"WaitForHoverStable", 10},
    {"ClearMissionPlan", 2},
    {"UploadMissionPlan", 5},
    {"EnsureFixedWingCruise", 20},
    {"EnterREPStation", 35},
    {"EnterHomeStation", 35},
    {"ActivateYOLO", 2},
    {"DeactivateYOLO", 1},
    {"ExecuteSearchPattern", 12},
    {"AlignToBasket", 18},
    {"AlignToLandingMarker", 18},
    {"DescendWithAlignment", 25},
    {"LandOnBasketZone", 10},
    {"LandOnLandingZone", 10},
    {"CloseGripper", 5},
    {"VerifyBasketPicked", 12},
    {"DetectLanding", 8},
    {"VerifyLandingComplete", 5},
    {"RecoverVisionLanding", 8},
    {"RecoverFinalLanding", 8}
  };

  bool shouldFail(const std::string& id)
  {
    if (fail_node.empty() || id != fail_node) {
      return false;
    }
    if (fail_after_tick > 0 && tick_count < fail_after_tick) {
      return false;
    }
    if (fail_once && fail_already_used) {
      return false;
    }
    fail_already_used = true;
    return true;
  }

  int durationFor(const std::string& id) const
  {
    const auto it = default_ticks_by_node.find(id);
    if (it != default_ticks_by_node.end()) {
      return it->second;
    }
    return std::max(0, default_action_ticks);
  }
};

static std::shared_ptr<MockRuntime> g_runtime;

static BT::PortsList allPorts()
{
  return {
    BT::InputPort<int>("mock_duration_ticks", -1, "mock에서 RUNNING을 유지할 tick 수"),
    BT::InputPort<std::string>("altitude_tolerance_m"),
    BT::InputPort<std::string>("approach_radius_m"),
    BT::InputPort<std::string>("arm"),
    BT::InputPort<std::string>("attach_topic"),
    BT::InputPort<std::string>("climb_altitude_m"),
    BT::InputPort<std::string>("descent_step_m"),
    BT::InputPort<std::string>("detach_topic"),
    BT::InputPort<std::string>("enable"),
    BT::InputPort<std::string>("expected_end"),
    BT::InputPort<std::string>("handoff_altitude_m"),
    BT::InputPort<std::string>("hold_latlon"),
    BT::InputPort<std::string>("left_position"),
    BT::InputPort<std::string>("left_topic"),
    BT::InputPort<std::string>("lift_test_height_m"),
    BT::InputPort<std::string>("max_age_sec"),
    BT::InputPort<std::string>("max_altitude_error_m"),
    BT::InputPort<std::string>("max_climb_rate_mps"),
    BT::InputPort<std::string>("max_cross_track_m"),
    BT::InputPort<std::string>("max_duration_sec"),
    BT::InputPort<std::string>("max_horizontal_error_m"),
    BT::InputPort<std::string>("max_horizontal_speed_mps"),
    BT::InputPort<std::string>("max_lost_time_sec"),
    BT::InputPort<std::string>("max_relative_altitude_m"),
    BT::InputPort<std::string>("max_state_age_sec"),
    BT::InputPort<std::string>("max_vertical_speed_mps"),
    BT::InputPort<std::string>("match"),
    BT::InputPort<std::string>("method"),
    BT::InputPort<std::string>("min_confidence"),
    BT::InputPort<std::string>("min_fix_status"),
    BT::InputPort<std::string>("min_percentage"),
    BT::InputPort<std::string>("min_rate_hz"),
    BT::InputPort<std::string>("min_voltage"),
    BT::InputPort<std::string>("mission_name"),
    BT::InputPort<std::string>("mode"),
    BT::InputPort<std::string>("optional"),
    BT::InputPort<std::string>("pattern"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("pixel_tolerance_px"),
    BT::InputPort<std::string>("plan_path"),
    BT::InputPort<std::string>("policy"),
    BT::InputPort<std::string>("radius_m"),
    BT::InputPort<std::string>("rate_hz"),
    BT::InputPort<std::string>("reason"),
    BT::InputPort<std::string>("require_disarmed"),
    BT::InputPort<std::string>("right_position"),
    BT::InputPort<std::string>("right_topic"),
    BT::InputPort<std::string>("service"),
    BT::InputPort<std::string>("seq"),
    BT::InputPort<std::string>("sim_bypass"),
    BT::InputPort<std::string>("sources"),
    BT::InputPort<std::string>("stable_duration_sec"),
    BT::InputPort<std::string>("stable_frames"),
    BT::InputPort<std::string>("station_radius_m"),
    BT::InputPort<std::string>("target"),
    BT::InputPort<std::string>("target_fresh_max_age_sec"),
    BT::InputPort<std::string>("target_altitude_m"),
    BT::InputPort<std::string>("target_state"),
    BT::InputPort<std::string>("timeout_sec"),
    BT::InputPort<std::string>("tolerance_m"),
    BT::InputPort<std::string>("tolerance_rad"),
    BT::InputPort<std::string>("topic"),
    BT::InputPort<std::string>("transition_source"),
    BT::InputPort<std::string>("yaw_tolerance_rad")
  };
}

class MockCondition : public BT::ConditionNode
{
public:
  MockCondition(const std::string& name,
                const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config), runtime_(g_runtime) {}

  static BT::PortsList providedPorts() { return allPorts(); }

  BT::NodeStatus tick() override
  {
    const std::string id = name();

    if (id == "IsEmergencyDetected") {
      const bool emergency = runtime_->emergency_at_tick > 0 &&
                             runtime_->tick_count >= runtime_->emergency_at_tick;
      if (runtime_->verbose) {
        RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] %s -> %s",
                    runtime_->tick_count, id.c_str(), emergency ? "SUCCESS" : "FAILURE");
      }
      return emergency ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    if (runtime_->shouldFail(id)) {
      RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] injected FAILURE at condition %s",
                  runtime_->tick_count, id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (runtime_->verbose) {
      RCLCPP_INFO(runtime_->logger, "[MOCK][tick %d] %s -> SUCCESS",
                  runtime_->tick_count, id.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<MockRuntime> runtime_;
};

class MockAction : public BT::StatefulActionNode
{
public:
  MockAction(const std::string& name,
             const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config), runtime_(g_runtime) {}

  static BT::PortsList providedPorts() { return allPorts(); }

  BT::NodeStatus onStart() override
  {
    id_ = name();
    start_tick_ = runtime_->tick_count;
    int override_ticks = -1;
    getInput("mock_duration_ticks", override_ticks);
    duration_ticks_ = (override_ticks >= 0) ? override_ticks : runtime_->durationFor(id_);

    if (id_ == "FailAfterRecovery") {
      if (runtime_->verbose) {
        RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] %s -> FAILURE", runtime_->tick_count, id_.c_str());
      }
      return BT::NodeStatus::FAILURE;
    }

    if (runtime_->shouldFail(id_)) {
      RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] injected FAILURE at action %s",
                  runtime_->tick_count, id_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (runtime_->verbose) {
      RCLCPP_INFO(runtime_->logger, "[MOCK][tick %d] %s START duration=%d ticks",
                  runtime_->tick_count, id_.c_str(), duration_ticks_);
    }

    if (duration_ticks_ <= 0) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (runtime_->shouldFail(id_)) {
      RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] injected FAILURE while running action %s",
                  runtime_->tick_count, id_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const int elapsed = runtime_->tick_count - start_tick_;
    if (runtime_->verbose) {
      RCLCPP_INFO(runtime_->logger, "[MOCK][tick %d] %s RUNNING elapsed=%d/%d",
                  runtime_->tick_count, id_.c_str(), elapsed, duration_ticks_);
    }

    if (elapsed >= duration_ticks_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (runtime_->verbose) {
      RCLCPP_WARN(runtime_->logger, "[MOCK][tick %d] %s HALTED", runtime_->tick_count, id_.c_str());
    }
  }

private:
  std::shared_ptr<MockRuntime> runtime_;
  std::string id_;
  int start_tick_{0};
  int duration_ticks_{0};
};

class KracBtMockRunner : public rclcpp::Node
{
public:
  KracBtMockRunner() : Node("krac_bt_mock_runner")
  {
    declare_parameter<std::string>("bt_xml_path", "");
    declare_parameter<double>("tick_rate_hz", 10.0);
    declare_parameter<int>("max_ticks", 0);
    declare_parameter<bool>("verbose_mock_ticks", false);

    declare_parameter<int>("default_action_ticks", 1);
    declare_parameter<std::string>("fail_node", "");
    declare_parameter<int>("fail_after_tick", 0);
    declare_parameter<bool>("fail_once", true);
    declare_parameter<int>("emergency_at_tick", 0);
  }

  int run()
  {
    const auto xml_path = get_parameter("bt_xml_path").as_string();
    const double tick_rate_hz = get_parameter("tick_rate_hz").as_double();
    const int max_ticks = get_parameter("max_ticks").as_int();

    if (xml_path.empty()) {
      RCLCPP_ERROR(get_logger(), "bt_xml_path parameter is empty");
      return 1;
    }

    auto runtime = std::make_shared<MockRuntime>();
    runtime->verbose = get_parameter("verbose_mock_ticks").as_bool();
    runtime->default_action_ticks = get_parameter("default_action_ticks").as_int();
    runtime->fail_node = get_parameter("fail_node").as_string();
    runtime->fail_after_tick = get_parameter("fail_after_tick").as_int();
    runtime->fail_once = get_parameter("fail_once").as_bool();
    runtime->emergency_at_tick = get_parameter("emergency_at_tick").as_int();
    runtime->logger = get_logger();

    BT::BehaviorTreeFactory factory;
    registerMockNodes(factory, runtime);

    RCLCPP_INFO(get_logger(), "Loading BT XML: %s", xml_path.c_str());
    RCLCPP_INFO(get_logger(), "Mock options: fail_node='%s', fail_after_tick=%d, fail_once=%s, emergency_at_tick=%d",
                runtime->fail_node.c_str(), runtime->fail_after_tick,
                runtime->fail_once ? "true" : "false", runtime->emergency_at_tick);

    auto tree = factory.createTreeFromFile(xml_path);
    BT::StdCoutLogger logger_cout(tree);

    rclcpp::Rate rate(tick_rate_hz > 0.0 ? tick_rate_hz : 10.0);

    while (rclcpp::ok()) {
      runtime->tick_count++;
      const auto status = tree.tickRoot();

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(get_logger(), "BT finished with SUCCESS at tick %d", runtime->tick_count);
        return 0;
      }
      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(get_logger(), "BT finished with FAILURE at tick %d", runtime->tick_count);
        return 2;
      }
      if (max_ticks > 0 && runtime->tick_count >= max_ticks) {
        RCLCPP_WARN(get_logger(), "Reached max_ticks=%d while BT is still RUNNING", max_ticks);
        return 3;
      }

      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    return 0;
  }

private:
  void registerMockNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<MockRuntime> runtime)
  {
    const std::vector<std::string> conditions = {
      "IsEmergencyDetected", "IsMAVROSConnected", "IsGPSReady", "IsGlobalPositionFresh",
      "IsBatterySafe", "IsOffboardSetpointStreamAlive", "IsFlightMode", "IsVehicleArmed",
      "IsVTOLStateMC", "IsAltitudeReached", "IsMissionUploaded", "IsAtREPRegion",
      "IsAtHomeRegion", "IsVisionTopicFresh", "IsObjectDetected", "VerifyPayloadSecured",
      "IsRescueCompleted", "IsWaypointReached", "IsPrecisionTargetDetected"
    };

    const std::vector<std::string> actions = {
      "EmergencyRecovery", "GlobalMissionRecovery", "StartOffboardSetpointStream", "SetFlightMode",
      "ArmVehicle", "EnsureFixedWingCruise", "CommandVTOLTransition", "FlyToAltitude",
      "WaitForHoverStable", "ClearMissionPlan", "UploadMissionPlan", "EnterREPStation",
      "EnterHomeStation", "ActivateYOLO", "DeactivateYOLO", "ExecuteSearchPattern",
      "AlignToBasket", "AlignToLandingMarker", "DescendWithAlignment", "LandOnBasketZone",
      "CloseGripper", "VerifyBasketPicked", "RecoverVisionLanding", "LandOnLandingZone",
      "DetectLanding", "VerifyLandingComplete", "RecoverFinalLanding", "FailAfterRecovery",
      "WaitForWaypointReached", "SetMissionCurrentWaypoint", "EnablePrecisionLander",
      "WaitForPrecisionTarget", "PrecisionLandOnTarget", "FlyToLocalPoint",
      "AlignHeadingToWaypoint", "SetMissionPhase", "OpenGripper"
    };

    g_runtime = runtime;

    for (const auto& id : conditions) {
      factory.registerNodeType<MockCondition>(id);
    }
    for (const auto& id : actions) {
      factory.registerNodeType<MockAction>(id);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KracBtMockRunner>();
  const int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
