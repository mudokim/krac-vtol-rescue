#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <krac_interfaces/msg/target_error.hpp>
#include <krac_interfaces/msg/flight_phase.hpp>

#include <deque>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

namespace krac_control::bt
{

struct GPSPoint
{
  double lat{0.0};
  double lon{0.0};
  double alt{0.0};
  bool valid{false};
};

class MissionContext
{
public:
  explicit MissionContext(const rclcpp::Node::SharedPtr& node);

  rclcpp::Node::SharedPtr node() const { return node_; }

  void init_ros_interfaces();
  void load_parameters();

  bool stateFresh(double max_age_sec) const;
  bool gpsFresh(double max_age_sec) const;
  bool visionFresh(double max_age_sec) const;
  bool connected() const;
  bool armed() const;
  std::string mode() const;
  int gpsStatus() const;
  int vtolState() const;
  int landedState() const;
  double relativeAltitude() const;
  double verticalSpeed() const;
  double horizontalSpeed() const;
  bool hasLocalPose() const;
  geometry_msgs::msg::PoseStamped localPose() const;
  bool hasRelAlt() const;
  bool objectDetected(double min_confidence) const;
  geometry_msgs::msg::Point visionError() const;

  GPSPoint currentGps() const;
  GPSPoint repPoint() const { return rep_point_; }
  GPSPoint homePoint() const { return home_point_; }
  double distanceTo(const GPSPoint& target) const;

  // -- Precision-lander / waypoint-triggered proxy control (ported from krac24 vtol_fsm.cpp) --
  bool targetErrorFresh(double max_age_sec) const;
  krac_interfaces::msg::TargetError targetError() const;
  int lastReachedWaypointSeq() const;
  int latestReachedWaypointSeq() const;
  uint64_t waypointReachedEventCount() const;
  double currentYaw() const;
  double computeBearingToWaypoint(int seq) const;

  void setPrecisionLanderEnabled(bool enable);
  bool precisionLanderEnabled() const { return precision_lander_active_; }

  rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr missionSetCurrentClient() { return mission_set_current_client_; }

  void publishMissionPhase(uint8_t phase);

  int rescueWpSeq() const { return rescue_wp_seq_; }
  int resumeWpSeq() const { return resume_wp_seq_; }
  int dropWpSeq() const { return drop_wp_seq_; }
  int landingWpSeq() const { return landing_wp_seq_; }
  double safeAscendAlt() const { return safe_ascend_alt_; }

  void startOffboardSetpointStream(double rate_hz, const std::string& mode);
  void stopOffboardSetpointStream();
  bool offboardStreamAlive(double min_rate_hz, double stable_duration_sec) const;
  void setHoldAltitude(double alt_m);
  void setHoldCurrentPosition();
  void setHoldGps(const GPSPoint& point);
  void publishVelocity(const geometry_msgs::msg::Twist& cmd);
  void publishZeroVelocity();
  void publishVisionTarget(const std::string& target);

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setModeClient() { return set_mode_client_; }
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr armingClient() { return arming_client_; }
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr commandClient() { return command_client_; }
  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr wpClearClient() { return wp_clear_client_; }

  bool runExternalMissionLoader(const std::string& plan_path, const std::string& mission_name, double timeout_sec);
  std::string resolvePlanPath(const std::string& plan_path) const;

  void markMissionUploaded(const std::string& mission_name, bool ok);
  bool isMissionUploaded(const std::string& mission_name) const;

  void setGripperClosed(bool closed) { gripper_closed_ = closed; }
  bool gripperClosed() const { return gripper_closed_; }
  bool gripperContact() const { std::lock_guard<std::mutex> lock(mutex_); return has_gripper_contact_ && gripper_contact_; }
  bool hasGripperContactFeedback() const { std::lock_guard<std::mutex> lock(mutex_); return has_gripper_contact_; }
  void markRescueCompleted();
  bool rescueCompleted() const;

  bool simBypass() const { return sim_bypass_; }
  bool missionUploadStubSuccess() const { return mission_upload_stub_success_; }
  bool gripperStubSuccess() const { return gripper_stub_success_; }
  double alignPGain() const { return align_p_gain_; }
  double maxAlignSpeed() const { return max_align_speed_; }
  double descendSpeed() const { return descend_speed_; }

  void logThrottleWarn(const std::string& key, const std::string& msg, double period_sec = 2.0) const;

private:
  static double haversineMeters(double lat1, double lon1, double lat2, double lon2);
  void offboardTimerCb();
  void recordSetpointStamp();
  void precisionLanderVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  mutable std::mutex mutex_;

  mavros_msgs::msg::State state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  sensor_msgs::msg::NavSatFix gps_;
  sensor_msgs::msg::BatteryState battery_;
  geometry_msgs::msg::PoseStamped local_pose_;
  geometry_msgs::msg::TwistStamped local_vel_;
  geometry_msgs::msg::Point vision_error_;
  std_msgs::msg::Float64 rel_alt_;
  mavros_msgs::msg::WaypointReached wp_reached_;
  krac_interfaces::msg::TargetError target_error_;
  std::vector<mavros_msgs::msg::Waypoint> wp_list_;

  bool has_state_{false};
  bool has_ext_state_{false};
  bool has_gps_{false};
  bool has_battery_{false};
  bool has_local_pose_{false};
  bool has_local_vel_{false};
  bool has_rel_alt_{false};
  bool has_vision_{false};
  bool has_wp_reached_{false};
  bool has_target_error_{false};

  // Highest wp_seq ever reported by /mavros/mission/reached. Waypoints placed
  // close together (e.g. same lat/lon, different altitude only) can all be
  // reached within a single BT tick period; tracking only the latest message
  // would let intermediate seqs slip past unseen. lastReachedWaypointSeq()
  // returns this so callers can use ">=" instead of exact "==" matching.
  int max_reached_wp_seq_{-1};
  int latest_reached_wp_seq_{-1};
  uint64_t wp_reached_event_count_{0};

  rclcpp::Time last_state_time_;
  rclcpp::Time last_gps_time_;
  rclcpp::Time last_vision_time_;
  rclcpp::Time last_battery_time_;
  rclcpp::Time last_target_error_time_;

  GPSPoint rep_point_;
  GPSPoint home_point_;

  bool precision_lander_active_{false};

  int rescue_wp_seq_{6};
  int resume_wp_seq_{7};
  int drop_wp_seq_{10};
  int landing_wp_seq_{13};
  double safe_ascend_alt_{30.0};

  std::string mavros_ns_{"/mavros"};
  std::string mission_dir_{""};
  bool sim_bypass_{true};
  bool mission_upload_stub_success_{true};
  bool gripper_stub_success_{true};
  double align_p_gain_{0.005};
  double max_align_speed_{0.5};
  double descend_speed_{0.25};

  bool offboard_stream_active_{false};
  std::string offboard_stream_mode_{"hold_current_pose"};
  double offboard_rate_hz_{20.0};
  GPSPoint hold_gps_;
  double hold_alt_m_{10.0};
  rclcpp::TimerBase::SharedPtr offboard_timer_;
  std::deque<rclcpp::Time> setpoint_pub_times_;

  std::string last_uploaded_mission_name_;
  bool last_mission_upload_ok_{false};
  bool gripper_closed_{false};
  bool gripper_contact_{false};
  bool has_gripper_contact_{false};
  bool rescue_completed_{false};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_error_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr wp_reached_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr wp_list_sub_;
  rclcpp::Subscription<krac_interfaces::msg::TargetError>::SharedPtr target_error_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr precision_lander_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_gripper_contact_sub_;

  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_target_pub_;
  rclcpp::Publisher<krac_interfaces::msg::FlightPhase>::SharedPtr mission_phase_pub_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr wp_clear_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr mission_set_current_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr precision_lander_enable_client_;
};

}  // namespace krac_control::bt
