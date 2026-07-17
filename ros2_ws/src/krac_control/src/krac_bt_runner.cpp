#include "krac_control/bt/mission_context.hpp"
#include "krac_control/bt/bt_register.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include "krac_control/bt/bt_viewer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("krac_bt_runner");

  node->declare_parameter<std::string>("bt_xml_path", "");
  node->declare_parameter<double>("tick_rate_hz", 10.0);
  node->declare_parameter<int>("max_ticks", 0);
  node->declare_parameter<bool>("print_bt_transitions", true);
  node->declare_parameter<bool>("enable_bt_viewer", false);
  node->declare_parameter<std::string>("bt_viewer_direction", "Vertical");

  const std::string xml_path = node->get_parameter("bt_xml_path").as_string();
  const double tick_rate_hz = node->get_parameter("tick_rate_hz").as_double();
  const int max_ticks = node->get_parameter("max_ticks").as_int();
  const bool print_transitions = node->get_parameter("print_bt_transitions").as_bool();
  const bool enable_bt_viewer = node->get_parameter("enable_bt_viewer").as_bool();
  const std::string bt_viewer_direction = node->get_parameter("bt_viewer_direction").as_string();

  if (xml_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "bt_xml_path parameter is empty");
    rclcpp::shutdown();
    return 1;
  }

  auto ctx = std::make_shared<krac_control::bt::MissionContext>(node);

  {
    rclcpp::Rate wait_rate(20.0);
    const auto wait_start = node->now();
    while (rclcpp::ok() && (node->now() - wait_start).seconds() < 5.0) {
      rclcpp::spin_some(node);
      if (ctx->stateFresh(1.0) && ctx->connected()) {
        break;
      }
      wait_rate.sleep();
    }
    if (!ctx->connected()) {
      RCLCPP_WARN(node->get_logger(), "Starting BT before local MissionContext has a connected /mavros/state sample.");
    }
  }

  BT::BehaviorTreeFactory factory;
  krac_control::bt::registerKracBtNodes(factory, ctx);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_path);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to create BT from [%s]: %s", xml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  std::unique_ptr<BT::StdCoutLogger> cout_logger;
  if (print_transitions) {
    cout_logger = std::make_unique<BT::StdCoutLogger>(tree);
  }

  std::unique_ptr<krac_control::bt::BTViewer> bt_viewer;
  if (enable_bt_viewer) {
    if (std::getenv("DISPLAY") == nullptr) {
      RCLCPP_WARN(node->get_logger(), "enable_bt_viewer=true but DISPLAY is not set; skipping BT viewer.");
    } else {
      bt_viewer = std::make_unique<krac_control::bt::BTViewer>(bt_viewer_direction);
      RCLCPP_INFO(node->get_logger(), "BT viewer window enabled (direction=%s).", bt_viewer_direction.c_str());
    }
  }

  RCLCPP_INFO(node->get_logger(), "KRAC real BT runner started. XML=%s tick_rate=%.2f", xml_path.c_str(), tick_rate_hz);

  rclcpp::Rate rate(tick_rate_hz > 0.1 ? tick_rate_hz : 10.0);
  int tick_count = 0;
  BT::NodeStatus status = BT::NodeStatus::IDLE;

  while (rclcpp::ok()) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    tick_count++;

    if (bt_viewer) {
      if (!bt_viewer->pollEvents()) {
        RCLCPP_INFO(node->get_logger(), "BT viewer window closed by user; continuing BT without it.");
        bt_viewer.reset();
      } else {
        bt_viewer->render(tree);
      }
    }

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "BT finished with SUCCESS at tick %d", tick_count);
      break;
    }
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "BT finished with FAILURE at tick %d", tick_count);
      break;
    }
    if (max_ticks > 0 && tick_count >= max_ticks) {
      RCLCPP_WARN(node->get_logger(), "BT stopped by max_ticks=%d", max_ticks);
      break;
    }
    rate.sleep();
  }

  tree.haltTree();
  if (ctx && rclcpp::ok()) {
    ctx->stopOffboardSetpointStream();
    ctx->publishZeroVelocity();
    rclcpp::spin_some(node);
  }
  cout_logger.reset();
  tree = BT::Tree();
  krac_control::bt::setGlobalContext(nullptr);
  ctx.reset();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return status == BT::NodeStatus::FAILURE ? 2 : 0;
}
