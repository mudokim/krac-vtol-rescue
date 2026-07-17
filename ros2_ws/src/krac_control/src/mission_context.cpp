#include "krac_control/bt/mission_context.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>

namespace krac_control::bt
{
namespace
{
constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
}

MissionContext::MissionContext(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  load_parameters();
  init_ros_interfaces();
}

void MissionContext::load_parameters()
{
  node_->declare_parameter<std::string>("mavros_ns", "/mavros");
  node_->declare_parameter<std::string>("mission_dir", "");
  node_->declare_parameter<bool>("sim_bypass", true);
  node_->declare_parameter<bool>("mission_upload_stub_success", true);
  node_->declare_parameter<bool>("gripper_stub_success", true);
  node_->declare_parameter<double>("align_p_gain", 0.005);
  node_->declare_parameter<double>("max_align_speed", 0.5);
  node_->declare_parameter<double>("descend_speed", 0.25);
  node_->declare_parameter<std::vector<double>>("rep_point", std::vector<double>{0.0, 0.0, 10.0});
  node_->declare_parameter<std::vector<double>>("home_point", std::vector<double>{0.0, 0.0, 10.0});
  node_->declare_parameter<int>("rescue_wp_seq", 6);
  node_->declare_parameter<int>("resume_wp_seq", 7);
  node_->declare_parameter<int>("drop_wp_seq", 10);
  node_->declare_parameter<int>("landing_wp_seq", 13);
  node_->declare_parameter<double>("safe_ascend_alt", 30.0);

  mavros_ns_ = node_->get_parameter("mavros_ns").as_string();
  mission_dir_ = node_->get_parameter("mission_dir").as_string();
  sim_bypass_ = node_->get_parameter("sim_bypass").as_bool();
  mission_upload_stub_success_ = node_->get_parameter("mission_upload_stub_success").as_bool();
  gripper_stub_success_ = node_->get_parameter("gripper_stub_success").as_bool();
  align_p_gain_ = node_->get_parameter("align_p_gain").as_double();
  max_align_speed_ = node_->get_parameter("max_align_speed").as_double();
  descend_speed_ = node_->get_parameter("descend_speed").as_double();

  auto rep = node_->get_parameter("rep_point").as_double_array();
  if (rep.size() >= 3 && (std::abs(rep[0]) > 1e-9 || std::abs(rep[1]) > 1e-9)) {
    rep_point_ = GPSPoint{rep[0], rep[1], rep[2], true};
  }
  auto home = node_->get_parameter("home_point").as_double_array();
  if (home.size() >= 3 && (std::abs(home[0]) > 1e-9 || std::abs(home[1]) > 1e-9)) {
    home_point_ = GPSPoint{home[0], home[1], home[2], true};
  }

  rescue_wp_seq_ = node_->get_parameter("rescue_wp_seq").as_int();
  resume_wp_seq_ = node_->get_parameter("resume_wp_seq").as_int();
  drop_wp_seq_ = node_->get_parameter("drop_wp_seq").as_int();
  landing_wp_seq_ = node_->get_parameter("landing_wp_seq").as_int();
  safe_ascend_alt_ = node_->get_parameter("safe_ascend_alt").as_double();
}

void MissionContext::init_ros_interfaces()
{
  auto qos = rclcpp::SensorDataQoS();
  auto topic = [this](const std::string& suffix) {
    return mavros_ns_ + suffix;
  };

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    topic("/state"), 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = *msg;
      has_state_ = true;
      last_state_time_ = node_->now();
    });

  ext_state_sub_ = node_->create_subscription<mavros_msgs::msg::ExtendedState>(
    topic("/extended_state"), 10,
    [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      extended_state_ = *msg;
      has_ext_state_ = true;
    });

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    topic("/global_position/global"), qos,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      gps_ = *msg;
      has_gps_ = true;
      last_gps_time_ = node_->now();
      if (!home_point_.valid && msg->status.status >= 0) {
        double alt = 0.0;
        if (has_rel_alt_) alt = rel_alt_.data;
        else if (has_local_pose_) alt = local_pose_.pose.position.z;
        home_point_ = GPSPoint{msg->latitude, msg->longitude, alt, true};
      }
    });

  rel_alt_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    topic("/global_position/rel_alt"), qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      rel_alt_ = *msg;
      has_rel_alt_ = true;
    });

  local_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic("/local_position/pose"), qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      local_pose_ = *msg;
      has_local_pose_ = true;
    });

  local_vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    topic("/local_position/velocity_local"), qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      local_vel_ = *msg;
      has_local_vel_ = true;
    });

  vision_error_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
    "camera/target_error", 10,
    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      vision_error_ = *msg;
      has_vision_ = true;
      if (msg->z > 0.0) {
        last_vision_time_ = node_->now();
      }
    });

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    topic("/battery"), qos,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      battery_ = *msg;
      has_battery_ = true;
      last_battery_time_ = node_->now();
    });

  wp_reached_sub_ = node_->create_subscription<mavros_msgs::msg::WaypointReached>(
    topic("/mission/reached"), 10,
    [this](const mavros_msgs::msg::WaypointReached::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      wp_reached_ = *msg;
      has_wp_reached_ = true;
      max_reached_wp_seq_ = std::max(max_reached_wp_seq_, static_cast<int>(msg->wp_seq));
      latest_reached_wp_seq_ = static_cast<int>(msg->wp_seq);
      ++wp_reached_event_count_;
    });

  wp_list_sub_ = node_->create_subscription<mavros_msgs::msg::WaypointList>(
    topic("/mission/waypoints"), qos,
    [this](const mavros_msgs::msg::WaypointList::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      wp_list_ = msg->waypoints;
    });

  target_error_sub_ = node_->create_subscription<krac_interfaces::msg::TargetError>(
    "/vision/target_error", 10,
    [this](const krac_interfaces::msg::TargetError::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      target_error_ = *msg;
      has_target_error_ = true;
      if (msg->is_detected) {
        last_target_error_time_ = node_->now();
      }
    });

  gripper_contact_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/gripper/contact", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      gripper_contact_ = msg->data;
      has_gripper_contact_ = true;
    });

  // Simulator adapter: gz detachable-joint state. Real hardware continues
  // to publish /gripper/contact, so the BT uses one common contact API.
  sim_gripper_contact_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/survivor_tray_rep/gripper_state", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      gripper_contact_ = msg->data;
      has_gripper_contact_ = true;
      RCLCPP_INFO(
        node_->get_logger(), "Simulator payload joint state: %s",
        msg->data ? "ATTACHED" : "DETACHED");
    });

  precision_lander_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/precision_lander/cmd_vel", 10,
    std::bind(&MissionContext::precisionLanderVelCb, this, std::placeholders::_1));

  global_setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(
    topic("/setpoint_raw/global"), 10);
  velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    topic("/setpoint_velocity/cmd_vel_unstamped"), 10);
  vision_target_pub_ = node_->create_publisher<std_msgs::msg::String>("camera/set_target", 10);
  mission_phase_pub_ = node_->create_publisher<krac_interfaces::msg::FlightPhase>("/krac/mission_phase", 10);

  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(topic("/set_mode"));
  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(topic("/cmd/arming"));
  command_client_ = node_->create_client<mavros_msgs::srv::CommandLong>(topic("/cmd/command"));
  wp_clear_client_ = node_->create_client<mavros_msgs::srv::WaypointClear>(topic("/mission/clear"));
  mission_set_current_client_ = node_->create_client<mavros_msgs::srv::WaypointSetCurrent>(topic("/mission/set_current"));
  precision_lander_enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/precision_lander/enable");
}

bool MissionContext::stateFresh(double max_age_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_state_ && (node_->now() - last_state_time_).seconds() <= max_age_sec;
}

bool MissionContext::gpsFresh(double max_age_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_gps_ && (node_->now() - last_gps_time_).seconds() <= max_age_sec;
}

bool MissionContext::visionFresh(double max_age_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_vision_ && last_vision_time_.nanoseconds() > 0 &&
         (node_->now() - last_vision_time_).seconds() <= max_age_sec;
}

bool MissionContext::connected() const { std::lock_guard<std::mutex> lock(mutex_); return has_state_ && state_.connected; }
bool MissionContext::armed() const { std::lock_guard<std::mutex> lock(mutex_); return has_state_ && state_.armed; }
std::string MissionContext::mode() const { std::lock_guard<std::mutex> lock(mutex_); return has_state_ ? state_.mode : std::string(); }
int MissionContext::gpsStatus() const { std::lock_guard<std::mutex> lock(mutex_); return has_gps_ ? gps_.status.status : -99; }
int MissionContext::vtolState() const { std::lock_guard<std::mutex> lock(mutex_); return has_ext_state_ ? static_cast<int>(extended_state_.vtol_state) : 0; }
int MissionContext::landedState() const { std::lock_guard<std::mutex> lock(mutex_); return has_ext_state_ ? static_cast<int>(extended_state_.landed_state) : 0; }
bool MissionContext::hasLocalPose() const { std::lock_guard<std::mutex> lock(mutex_); return has_local_pose_; }
bool MissionContext::hasRelAlt() const { std::lock_guard<std::mutex> lock(mutex_); return has_rel_alt_; }

geometry_msgs::msg::PoseStamped MissionContext::localPose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return local_pose_;
}

double MissionContext::relativeAltitude() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_rel_alt_) return rel_alt_.data;
  if (has_local_pose_) return local_pose_.pose.position.z;
  return 0.0;
}

double MissionContext::verticalSpeed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_local_vel_ ? local_vel_.twist.linear.z : 0.0;
}

double MissionContext::horizontalSpeed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_local_vel_) return 0.0;
  const auto& v = local_vel_.twist.linear;
  return std::sqrt(v.x * v.x + v.y * v.y);
}

bool MissionContext::objectDetected(double min_confidence) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_vision_ && last_vision_time_.nanoseconds() > 0 &&
         (node_->now() - last_vision_time_).seconds() < 2.5 &&
         vision_error_.z >= min_confidence;
}

geometry_msgs::msg::Point MissionContext::visionError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return vision_error_;
}

GPSPoint MissionContext::currentGps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_gps_) return GPSPoint{};
  double alt = 0.0;
  if (has_rel_alt_) alt = rel_alt_.data;
  else if (has_local_pose_) alt = local_pose_.pose.position.z;
  return GPSPoint{gps_.latitude, gps_.longitude, alt, true};
}

double MissionContext::haversineMeters(double lat1, double lon1, double lat2, double lon2)
{
  const double dlat = (lat2 - lat1) * DEG2RAD;
  const double dlon = (lon2 - lon1) * DEG2RAD;
  const double rlat1 = lat1 * DEG2RAD;
  const double rlat2 = lat2 * DEG2RAD;
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(rlat1) * std::cos(rlat2) *
                   std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  return EARTH_RADIUS_M * 2.0 * std::asin(std::sqrt(a));
}

double MissionContext::distanceTo(const GPSPoint& target) const
{
  if (!target.valid) return 1e9;
  auto cur = currentGps();
  if (!cur.valid) return 1e9;
  return haversineMeters(cur.lat, cur.lon, target.lat, target.lon);
}

bool MissionContext::targetErrorFresh(double max_age_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_target_error_ && target_error_.is_detected &&
         last_target_error_time_.nanoseconds() > 0 &&
         (node_->now() - last_target_error_time_).seconds() <= max_age_sec;
}

krac_interfaces::msg::TargetError MissionContext::targetError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_error_;
}

int MissionContext::lastReachedWaypointSeq() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_wp_reached_ ? max_reached_wp_seq_ : -1;
}

int MissionContext::latestReachedWaypointSeq() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_wp_reached_ ? latest_reached_wp_seq_ : -1;
}

uint64_t MissionContext::waypointReachedEventCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return wp_reached_event_count_;
}

double MissionContext::currentYaw() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_local_pose_) return 0.0;
  const auto& q = local_pose_.pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double MissionContext::computeBearingToWaypoint(int seq) const
{
  const double fallback_yaw = currentYaw();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_gps_ || seq < 0 || static_cast<size_t>(seq) >= wp_list_.size()) return fallback_yaw;

  const auto& wp = wp_list_[static_cast<size_t>(seq)];
  if (wp.command == 3000 || (std::abs(wp.x_lat) < 1e-9 && std::abs(wp.y_long) < 1e-9)) return fallback_yaw;

  const double lat1 = gps_.latitude * DEG2RAD;
  const double lon1 = gps_.longitude * DEG2RAD;
  const double lat2 = wp.x_lat * DEG2RAD;
  const double lon2 = wp.y_long * DEG2RAD;

  const double dlon = lon2 - lon1;
  const double y = std::sin(dlon) * std::cos(lat2);
  const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
  const double bearing = std::atan2(y, x);

  // MAVROS local frame (ENU: East=0, CCW) 로 변환
  double target_yaw = -bearing + (M_PI / 2.0);
  while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
  while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
  return target_yaw;
}

void MissionContext::precisionLanderVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  bool active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active = precision_lander_active_;
  }
  if (active) {
    velocity_pub_->publish(*msg);
  }
}

void MissionContext::setPrecisionLanderEnabled(bool enable)
{
  // Do not publish the old 3 m global hold while precision_lander publishes
  // downward velocity. Those two controllers previously fought each other.
  if (enable) {
    stopOffboardSetpointStream();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    precision_lander_active_ = enable;
  }

  if (!enable) {
    publishZeroVelocity();
  }

  if (!precision_lander_enable_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(node_->get_logger(), "/precision_lander/enable service unavailable");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;
  precision_lander_enable_client_->async_send_request(req);
}

void MissionContext::publishMissionPhase(uint8_t phase)
{
  krac_interfaces::msg::FlightPhase msg;
  msg.current_phase = phase;
  mission_phase_pub_->publish(msg);
}

void MissionContext::markRescueCompleted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rescue_completed_ = true;
}

bool MissionContext::rescueCompleted() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rescue_completed_;
}

void MissionContext::startOffboardSetpointStream(double rate_hz, const std::string& mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  offboard_stream_active_ = true;
  offboard_stream_mode_ = mode;
  offboard_rate_hz_ = std::max(2.0, rate_hz);
  setpoint_pub_times_.clear();

  if (has_gps_) {
    double alt = 0.0;
    if (has_rel_alt_) alt = rel_alt_.data;
    else if (has_local_pose_) alt = local_pose_.pose.position.z;
    hold_gps_ = GPSPoint{gps_.latitude, gps_.longitude, alt, true};
    hold_alt_m_ = alt;
  }

  const auto period = std::chrono::duration<double>(1.0 / offboard_rate_hz_);
  offboard_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MissionContext::offboardTimerCb, this));
}

void MissionContext::stopOffboardSetpointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  offboard_stream_active_ = false;
  offboard_timer_.reset();
  setpoint_pub_times_.clear();
}

void MissionContext::setHoldCurrentPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_gps_) {
    double alt = 0.0;
    if (has_rel_alt_) alt = rel_alt_.data;
    else if (has_local_pose_) alt = local_pose_.pose.position.z;
    hold_gps_ = GPSPoint{gps_.latitude, gps_.longitude, alt, true};
    hold_alt_m_ = alt;
  }
}

void MissionContext::setHoldAltitude(double alt_m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hold_alt_m_ = alt_m;
  // 위경도는 아직 래치된 적이 없을 때만 잡는다. 이미 유효하면 유지해야 한다.
  // 이 함수는 onRunning 에서 매 tick 호출되므로(예: FlyToAltitude), 여기서
  // 현재 GPS 로 재래치하면 위치 setpoint 가 기체를 계속 따라다녀 위치오차가
  // 항상 0 이 되고 PX4 가 수평 드리프트를 보정하지 못한다.
  // 명시적 래치는 setHoldCurrentPosition()/startOffboardSetpointStream()/setHoldGps() 담당.
  if (!hold_gps_.valid && has_gps_) {
    hold_gps_ = GPSPoint{gps_.latitude, gps_.longitude, alt_m, true};
  }
}

void MissionContext::setHoldGps(const GPSPoint& point)
{
  if (!point.valid) return;
  std::lock_guard<std::mutex> lock(mutex_);
  hold_gps_ = point;
  hold_alt_m_ = point.alt;
}

void MissionContext::offboardTimerCb()
{
  mavros_msgs::msg::GlobalPositionTarget msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!offboard_stream_active_ || precision_lander_active_) return;
    msg.header.stamp = node_->now();
    msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    msg.type_mask =
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_VX |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_VY |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_VZ |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFX |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFY |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW |
      mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW_RATE;
    if (hold_gps_.valid) {
      msg.latitude = hold_gps_.lat;
      msg.longitude = hold_gps_.lon;
      msg.altitude = hold_alt_m_;
    } else if (has_gps_) {
      msg.latitude = gps_.latitude;
      msg.longitude = gps_.longitude;
      msg.altitude = hold_alt_m_;
    } else {
      return;
    }
  }
  global_setpoint_pub_->publish(msg);
  recordSetpointStamp();
}

void MissionContext::recordSetpointStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();
  setpoint_pub_times_.push_back(now);
  while (!setpoint_pub_times_.empty() && (now - setpoint_pub_times_.front()).seconds() > 3.0) {
    setpoint_pub_times_.pop_front();
  }
}

bool MissionContext::offboardStreamAlive(
  double min_rate_hz, double stable_duration_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!offboard_stream_active_ || setpoint_pub_times_.size() < 2) {
    return false;
  }

  const auto now = node_->now();
  if ((now - setpoint_pub_times_.back()).seconds() > 0.5) {
    return false;
  }

  const double required_span =
    std::max(0.5, std::min(3.0, stable_duration_sec));

  int count = 0;
  rclcpp::Time first_in_window(0, 0, node_->get_clock()->get_clock_type());
  rclcpp::Time last_in_window(0, 0, node_->get_clock()->get_clock_type());

  for (const auto& stamp : setpoint_pub_times_) {
    if ((now - stamp).seconds() <= required_span + 0.25) {
      if (count == 0) {
        first_in_window = stamp;
      }
      last_in_window = stamp;
      ++count;
    }
  }

  if (count < 2) {
    return false;
  }

  const double actual_span = (last_in_window - first_in_window).seconds();
  const double actual_rate =
    actual_span > 1e-3 ? static_cast<double>(count - 1) / actual_span : 0.0;

  const bool span_ok = actual_span >= required_span * 0.90;
  const bool rate_ok = actual_rate >= min_rate_hz;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 500,
    "OFFBOARD readiness: samples=%d span=%.2fs rate=%.1fHz "
    "required_span=%.2fs min_rate=%.1fHz ready=%d",
    count, actual_span, actual_rate, required_span, min_rate_hz,
    span_ok && rate_ok);

  return span_ok && rate_ok;
}

void MissionContext::publishVelocity(const geometry_msgs::msg::Twist& cmd)
{
  velocity_pub_->publish(cmd);
}

void MissionContext::publishZeroVelocity()
{
  geometry_msgs::msg::Twist cmd;
  publishVelocity(cmd);
}

void MissionContext::publishVisionTarget(const std::string& target)
{
  std_msgs::msg::String msg;
  msg.data = target;
  vision_target_pub_->publish(msg);
}

std::string MissionContext::resolvePlanPath(const std::string& plan_path) const
{
  if (plan_path.empty()) return plan_path;
  if (plan_path.front() == '/') return plan_path;
  if (!mission_dir_.empty()) return mission_dir_ + "/" + plan_path;
  return plan_path;
}

bool MissionContext::runExternalMissionLoader(const std::string& plan_path, const std::string& mission_name, double timeout_sec)
{
  const std::string resolved = resolvePlanPath(plan_path);
  std::ifstream f(resolved);
  if (!f.good()) {
    RCLCPP_WARN(node_->get_logger(), "Mission plan not found: %s", resolved.c_str());
    if (mission_upload_stub_success_) {
      markMissionUploaded(mission_name, true);
      RCLCPP_WARN(node_->get_logger(), "mission_upload_stub_success=true, treating missing plan as SUCCESS for staged test.");
      return true;
    }
    markMissionUploaded(mission_name, false);
    return false;
  }

  std::stringstream cmd;
  cmd << "timeout " << std::max(1.0, timeout_sec) << "s ros2 run krac_control mission_loader.py --ros-args -p plan_file:=" << resolved;
  RCLCPP_INFO(node_->get_logger(), "Running external mission loader: %s", cmd.str().c_str());
  int ret = std::system(cmd.str().c_str());
  bool ok = (ret == 0);
  if (!ok && mission_upload_stub_success_) {
    RCLCPP_WARN(node_->get_logger(), "Mission loader returned %d, but mission_upload_stub_success=true. Treating as SUCCESS for staged test.", ret);
    ok = true;
  }
  markMissionUploaded(mission_name, ok);
  return ok;
}

void MissionContext::markMissionUploaded(const std::string& mission_name, bool ok)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_uploaded_mission_name_ = mission_name;
  last_mission_upload_ok_ = ok;
}

bool MissionContext::isMissionUploaded(const std::string& mission_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_mission_upload_ok_ && last_uploaded_mission_name_ == mission_name;
}

void MissionContext::logThrottleWarn(const std::string&, const std::string& msg, double period_sec) const
{
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), static_cast<int>(period_sec * 1000.0), "%s", msg.c_str());
}

}  // namespace krac_control::bt
