#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

#define EARTH_RADIUS 6371000.0

enum MissionPhase {
    PHASE_WAITING_FOR_HANDOVER = 0,
    PHASE_MOVE_TO_RESCUE = 1,
    PHASE_SEARCH_RESCUE = 2,
    PHASE_VISION_ALIGN_RESCUE = 3,
    PHASE_ASCEND_WITH_VICTIM = 4,
    PHASE_MOVE_TO_DROP = 5,
    PHASE_SEARCH_DROP = 6,
    PHASE_VISION_ALIGN_DROP = 7,
    PHASE_DROP_AND_ASCEND = 8,
    PHASE_RETURN_TO_HOME = 9,
    PHASE_VISION_ALIGN_LAND = 10,
    PHASE_LANDING = 11
};

struct GPSPoint {
    double lat; double lon; double alt;
};

class RescueMissionNode : public rclcpp::Node
{
public:
    RescueMissionNode() : Node("rescue_mission_node")
    {
        auto qos = rclcpp::SensorDataQoS();
        last_vision_time_ = this->now();

        this->declare_parameter("handover_seq", 6);
        handover_seq_ = this->get_parameter("handover_seq").as_int();

        this->declare_parameter("rescue_loc", std::vector<double>{35.06945, 128.0864, 6.0});
        this->declare_parameter("drop_loc", std::vector<double>{35.06883, 128.0858, 6.0});
        this->declare_parameter("home_loc", std::vector<double>{35.06898, 128.0863, 5.0});

        load_param("rescue_loc", rescue_loc_);
        load_param("drop_loc", drop_loc_);
        load_param("home_loc", home_loc_);

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, std::bind(&RescueMissionNode::state_cb, this, std::placeholders::_1));

        global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "mavros/global_position/global", qos, std::bind(&RescueMissionNode::global_pos_cb, this, std::placeholders::_1));

        mission_reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
            "mavros/mission/reached", 10, std::bind(&RescueMissionNode::mission_reached_cb, this, std::placeholders::_1));

        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose", qos, std::bind(&RescueMissionNode::local_pose_cb, this, std::placeholders::_1));

        vision_error_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "camera/target_error", 10, std::bind(&RescueMissionNode::vision_cb, this, std::placeholders::_1));

        vel_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        global_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>(
            "mavros/setpoint_raw/global", 10);

        target_label_pub_ = this->create_publisher<std_msgs::msg::String>(
            "camera/set_target", 10);

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        manual_trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
            "cmd/mission_proceed", std::bind(&RescueMissionNode::manual_proceed_cb, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&RescueMissionNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Rescue Node Started. Handover Seq: %d", handover_seq_);
    }

private:
    MissionPhase current_phase_ = PHASE_WAITING_FOR_HANDOVER;
    int handover_seq_;

    GPSPoint rescue_loc_, drop_loc_, home_loc_;
    mavros_msgs::msg::State current_state_;
    sensor_msgs::msg::NavSatFix current_global_pos_;
    geometry_msgs::msg::PoseStamped current_local_pose_;
    geometry_msgs::msg::Point vision_error_;
    rclcpp::Time last_vision_time_;

    bool offboard_enabled_ = false;
    bool manual_ok_ = false;
    
    // [튜닝 파라미터]
    const double VISION_P_GAIN = 0.005;
    const double MAX_ALIGN_SPEED = 0.5;
    const double DESCEND_SPEED = -0.3;
    const double HOVER_STEP = 1.0; 
    const double SEARCH_SPEED = 0.2; 

    // [탐색 관련 변수]
    int search_step_ = 0;
    rclcpp::Time search_start_time_;

    // [하강 제어 변수]
    double target_hover_alt_ = 0.0;
    bool hover_stabilized_ = false;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr mission_reached_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_error_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_setpoint_pub_;
    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_label_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_trigger_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_param(std::string name, GPSPoint &pt) {
        std::vector<double> vec = this->get_parameter(name).as_double_array();
        if(vec.size() >= 3) { pt.lat = vec[0]; pt.lon = vec[1]; pt.alt = vec[2]; }
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void global_pos_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { current_global_pos_ = *msg; }
    void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_local_pose_ = *msg; }
    void vision_cb(const geometry_msgs::msg::Point::SharedPtr msg) { 
        vision_error_ = *msg; 
        if(msg->z > 0.5) last_vision_time_ = this->now();
    }

    double get_dist_to_gps(GPSPoint target) {
        if (current_global_pos_.status.status < 0) return 9999.0;
        double dLat = (target.lat - current_global_pos_.latitude) * M_PI / 180.0;
        double dLon = (target.lon - current_global_pos_.longitude) * M_PI / 180.0;
        double a = std::pow(std::sin(dLat / 2), 2) + 
                   std::pow(std::sin(dLon / 2), 2) * std::cos(current_global_pos_.latitude*M_PI/180.0) * std::cos(target.lat*M_PI/180.0);
        return EARTH_RADIUS * 2 * std::asin(std::sqrt(a));
    }

    void set_vision_label(std::string label) {
        std_msgs::msg::String msg; msg.data = label; target_label_pub_->publish(msg);
    }

    void manual_proceed_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        manual_ok_ = true;
        res->success = true;
        res->message = "Force Proceed Triggered";
        RCLCPP_INFO(this->get_logger(), "✅ Service Call Received! Force Proceeding.");
    }

    void mission_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg) {
        if (current_phase_ == PHASE_WAITING_FOR_HANDOVER && msg->wp_seq == handover_seq_) {
            RCLCPP_WARN(this->get_logger(), "🚀 Reached WP2! Switching to OFFBOARD.");
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
            offboard_enabled_ = true;
            current_phase_ = PHASE_MOVE_TO_RESCUE;
        }
    }

    double clamp_velocity(double vel) {
        return std::clamp(vel, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);
    }

    void execute_smooth_search_pattern(geometry_msgs::msg::Twist &vel_cmd) {
        if (search_step_ == 0) {
            search_start_time_ = this->now();
            search_step_ = 1;
        }
        double elapsed = (this->now() - search_start_time_).seconds();
        
        if (elapsed < 3.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = -SEARCH_SPEED; } 
        else if (elapsed < 4.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; } 
        else if (elapsed < 7.0) { vel_cmd.linear.x = -SEARCH_SPEED; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 8.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 11.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = SEARCH_SPEED; }
        else if (elapsed < 12.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 15.0) { vel_cmd.linear.x = SEARCH_SPEED; vel_cmd.linear.y = 0.0; }
        else { search_step_ = 0; }
        vel_cmd.linear.z = 0.0; vel_cmd.angular.z = 0.0; 
    }

    void execute_step_descent(geometry_msgs::msg::Twist &vel_cmd, double &target_alt) {
        double current_alt = current_local_pose_.pose.position.z;
        double dist_pixel = std::sqrt(vision_error_.x * vision_error_.x + vision_error_.y * vision_error_.y);

        vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
        vel_cmd.linear.y = clamp_velocity(-vision_error_.x * VISION_P_GAIN);
        vel_cmd.angular.z = 0.0;

        if (current_alt < target_alt - 0.1) target_alt = current_alt;

        if (std::abs(current_alt - target_alt) < 0.2) {
            if (dist_pixel < 30.0) { 
                if (target_alt > 1.5) {
                     target_alt -= HOVER_STEP;
                     if (target_alt < 1.5) target_alt = 1.5; 
                     RCLCPP_INFO(this->get_logger(), "Aligned! Stepping down to %.1f m", target_alt);
                }
            } else {
                vel_cmd.linear.z = 0.0;
            }
        } else if (current_alt > target_alt) {
            if (dist_pixel > 100.0) vel_cmd.linear.z = 0.0;
            else vel_cmd.linear.z = DESCEND_SPEED;
        } else {
            vel_cmd.linear.z = 0.0;
        }
    }

    void control_loop() {
        if (!offboard_enabled_) return;

        if (current_state_.mode != "OFFBOARD") {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
        }

        mavros_msgs::msg::GlobalPositionTarget global_target;
        global_target.header.stamp = this->now();
        global_target.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
        global_target.type_mask = 0b110111111000; 

        geometry_msgs::msg::Twist vel_cmd;
        bool use_vel_control = false;
        bool object_detected = (last_vision_time_.nanoseconds() > 0 && 
                               (this->now() - last_vision_time_).seconds() < 0.5); 

        if (manual_ok_) {
            manual_ok_ = false; 
            if (current_phase_ <= PHASE_VISION_ALIGN_RESCUE) {
                 RCLCPP_INFO(this->get_logger(), "Force Proceed -> ASCEND WITH VICTIM");
                 current_phase_ = PHASE_ASCEND_WITH_VICTIM;
            } else if (current_phase_ <= PHASE_VISION_ALIGN_DROP) {
                 RCLCPP_INFO(this->get_logger(), "Force Proceed -> DROP AND ASCEND");
                 current_phase_ = PHASE_DROP_AND_ASCEND;
            } else if (current_phase_ <= PHASE_VISION_ALIGN_LAND) {
                 RCLCPP_INFO(this->get_logger(), "Force Proceed -> LANDING");
                 current_phase_ = PHASE_LANDING;
            }
        }

        switch (current_phase_) {
            case PHASE_WAITING_FOR_HANDOVER:
                break;

            case PHASE_MOVE_TO_RESCUE:
                global_target.latitude = rescue_loc_.lat;
                global_target.longitude = rescue_loc_.lon;
                global_target.altitude = rescue_loc_.alt; 
                if (get_dist_to_gps(rescue_loc_) < 1.5) {
                    RCLCPP_INFO(this->get_logger(), "Arrived Rescue Zone. Searching 'basket'...");
                    set_vision_label("basket"); 
                    current_phase_ = PHASE_SEARCH_RESCUE;
                    search_step_ = 0;
                }
                break;

            case PHASE_SEARCH_RESCUE:
                use_vel_control = true;
                if (object_detected) {
                    RCLCPP_INFO(this->get_logger(), "Object Found! Aligning...");
                    target_hover_alt_ = current_local_pose_.pose.position.z; 
                    current_phase_ = PHASE_VISION_ALIGN_RESCUE;
                } else {
                    if (current_local_pose_.pose.position.z < 3.0) execute_smooth_search_pattern(vel_cmd);
                    else { vel_cmd.linear.z = -0.3; vel_cmd.angular.z = 0.0; }
                }
                break;

            case PHASE_VISION_ALIGN_RESCUE:
                use_vel_control = true;
                if (object_detected) {
                    if (current_local_pose_.pose.position.z < 1.6 && target_hover_alt_ <= 1.5) {
                        vel_cmd.linear.z = 0.0;
                        vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
                        vel_cmd.linear.y = clamp_velocity(-vision_error_.x * VISION_P_GAIN);
                        if (manual_ok_) {
                             RCLCPP_INFO(this->get_logger(), "Rescue Triggered! Ascending.");
                             manual_ok_ = false; current_phase_ = PHASE_ASCEND_WITH_VICTIM;
                        } else {
                             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hovering at 1.5m. Waiting for Trigger...");
                        }
                    } else {
                        execute_step_descent(vel_cmd, target_hover_alt_);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Lost Object! Searching...");
                    current_phase_ = PHASE_SEARCH_RESCUE;
                    search_step_ = 0; 
                }
                break;

            case PHASE_ASCEND_WITH_VICTIM:
                use_vel_control = true;
                vel_cmd.linear.z = 0.5; 
                if (current_local_pose_.pose.position.z > 5.0) {
                    RCLCPP_INFO(this->get_logger(), "Ascent Complete. Moving to Drop Zone.");
                    current_phase_ = PHASE_MOVE_TO_DROP;
                }
                break;

            case PHASE_MOVE_TO_DROP:
                global_target.latitude = drop_loc_.lat;
                global_target.longitude = drop_loc_.lon;
                global_target.altitude = drop_loc_.alt;
                if (get_dist_to_gps(drop_loc_) < 1.5) {
                    RCLCPP_INFO(this->get_logger(), "Arrived Drop Zone. Searching 'drop_zone'...");
                    set_vision_label("drop_zone"); 
                    current_phase_ = PHASE_SEARCH_DROP;
                    search_step_ = 0;
                }
                break;
            
            case PHASE_SEARCH_DROP:
                use_vel_control = true;
                if (object_detected) {
                    RCLCPP_INFO(this->get_logger(), "Drop Zone Found! Aligning...");
                    target_hover_alt_ = current_local_pose_.pose.position.z;
                    current_phase_ = PHASE_VISION_ALIGN_DROP;
                } else {
                    if (current_local_pose_.pose.position.z < 3.0) execute_smooth_search_pattern(vel_cmd);
                    else { vel_cmd.linear.z = -0.3; vel_cmd.angular.z = 0.0; }
                }
                break;

            case PHASE_VISION_ALIGN_DROP:
                use_vel_control = true;
                if (object_detected) {
                    if (current_local_pose_.pose.position.z < 1.6 && target_hover_alt_ <= 1.5) {
                        vel_cmd.linear.z = 0.0;
                        vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
                        vel_cmd.linear.y = clamp_velocity(-vision_error_.x * VISION_P_GAIN);
                        if (manual_ok_) {
                             RCLCPP_INFO(this->get_logger(), "Drop Triggered! Ascending.");
                             manual_ok_ = false; current_phase_ = PHASE_DROP_AND_ASCEND;
                        } else {
                             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hovering at 1.5m. Waiting for Trigger...");
                        }
                    } else {
                        execute_step_descent(vel_cmd, target_hover_alt_);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Lost Drop Zone! Searching...");
                    current_phase_ = PHASE_SEARCH_DROP;
                    search_step_ = 0;
                }
                break;

            case PHASE_DROP_AND_ASCEND:
                use_vel_control = true;
                vel_cmd.linear.z = 0.5;
                if (current_local_pose_.pose.position.z > 5.0) {
                    RCLCPP_INFO(this->get_logger(), "Drop Complete. Returning Home.");
                    current_phase_ = PHASE_RETURN_TO_HOME;
                }
                break;

            case PHASE_RETURN_TO_HOME:
                global_target.latitude = home_loc_.lat;
                global_target.longitude = home_loc_.lon;
                global_target.altitude = 5.0; 
                if (get_dist_to_gps(home_loc_) < 2.0) {
                    RCLCPP_INFO(this->get_logger(), "Home Reached. Searching Vertiport...");
                    set_vision_label("vertiport"); 
                    current_phase_ = PHASE_VISION_ALIGN_LAND;
                }
                break;
            
            case PHASE_VISION_ALIGN_LAND:
                use_vel_control = true;
                if (object_detected) {
                    double dist_pixel = std::sqrt(vision_error_.x * vision_error_.x + vision_error_.y * vision_error_.y);
                    
                    vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
                    vel_cmd.linear.y = clamp_velocity(-vision_error_.x * VISION_P_GAIN);
                    vel_cmd.angular.z = 0.0;

                    // [수정] 착륙 시에는 멈추지 않고 계속 하강 (Adaptive Descent)
                    if (dist_pixel < 100.0) {
                        vel_cmd.linear.z = -0.4; // 정렬 잘 되면 더 빨리 하강
                    } else {
                        vel_cmd.linear.z = -0.1; // 정렬 중이어도 멈추지 않고 천천히 하강
                    }

                    if (current_local_pose_.pose.position.z < 0.5) {
                        RCLCPP_INFO(this->get_logger(), "Near Ground. Switching to AUTO.LAND.");
                        current_phase_ = PHASE_LANDING;
                    }
                } else {
                     // 마커 놓쳐도 착륙 시에는 계속 하강
                     vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; vel_cmd.linear.z = -0.2;
                     vel_cmd.angular.z = 0.0;
                }
                break;

            case PHASE_LANDING:
                {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "AUTO.LAND";
                    set_mode_client_->async_send_request(req);
                    offboard_enabled_ = false;
                }
                break;
        }

        if (use_vel_control) {
            vel_setpoint_pub_->publish(vel_cmd);
        } else {
            global_setpoint_pub_->publish(global_target);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RescueMissionNode>());
    rclcpp::shutdown();
    return 0;
}
