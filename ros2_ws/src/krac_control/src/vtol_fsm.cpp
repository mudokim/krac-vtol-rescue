#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <krac_interfaces/msg/flight_phase.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

#define EARTH_RADIUS 6371000.0
#define M_PI 3.14159265358979323846
#define GRAVITY_ACCEL 9.80665

enum {
    MAV_VTOL_STATE_UNDEFINED = 0,
    MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
    MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
    MAV_VTOL_STATE_MC = 3, 
    MAV_VTOL_STATE_FW = 4  
};

struct GPSPoint {
    double lat; double lon; double alt;
};

class VtolGpsMissionNode : public rclcpp::Node
{
public:
    VtolGpsMissionNode() : Node("vtol_gps_mission_node")
    {
        auto qos_profile = rclcpp::SensorDataQoS();

        // [파라미터 설정]
        this->declare_parameter("wp1", std::vector<double>{35.06898, 128.0863, 40.0}); 
        this->declare_parameter("wp2", std::vector<double>{35.06861, 128.0868, 20.0});       
        this->declare_parameter("wp3", std::vector<double>{35.06737, 128.0873, 20.0}); 
        this->declare_parameter("wp4", std::vector<double>{35.06854, 128.0884, 20.0}); 
        this->declare_parameter("rescue_loc", std::vector<double>{35.06945, 128.0864, 5.0}); 
        this->declare_parameter("drop_loc", std::vector<double>{35.06883, 128.0858, 5.0});    

        load_param("wp1", wp1_); load_param("wp2", wp2_);
        load_param("wp3", wp3_); load_param("wp4", wp4_);
        load_param("rescue_loc", rescue_loc_); load_param("drop_loc", drop_loc_);

        // [Subscribers]
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", qos_profile, std::bind(&VtolGpsMissionNode::state_cb, this, std::placeholders::_1));
        extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("mavros/extended_state", qos_profile, std::bind(&VtolGpsMissionNode::extended_state_cb, this, std::placeholders::_1));
        global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("mavros/global_position/global", qos_profile, std::bind(&VtolGpsMissionNode::global_pos_cb, this, std::placeholders::_1));
        local_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local", qos_profile, std::bind(&VtolGpsMissionNode::local_vel_cb, this, std::placeholders::_1));
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", qos_profile, std::bind(&VtolGpsMissionNode::local_pose_cb, this, std::placeholders::_1));

        vision_error_sub_ = this->create_subscription<geometry_msgs::msg::Point>("camera/target_error", 10, std::bind(&VtolGpsMissionNode::vision_cb, this, std::placeholders::_1));
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10, std::bind(&VtolGpsMissionNode::cam_info_cb, this, std::placeholders::_1));

        // [Publishers]
        global_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
        vel_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        phase_pub_ = this->create_publisher<krac_interfaces::msg::FlightPhase>("krac/mission_phase", 10);

        // [Service Clients]
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
        
        manual_trigger_server_ = this->create_service<std_srvs::srv::Trigger>("cmd/mission_proceed", std::bind(&VtolGpsMissionNode::manual_proceed_cb, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&VtolGpsMissionNode::control_loop, this));
        
        target_label_pub_ = this->create_publisher<std_msgs::msg::String>("camera/set_target", 10);
        
        RCLCPP_INFO(this->get_logger(), "KRAC VTOL Mission Node Started (Accel Trigger Mode).");
    }

private:
    mavros_msgs::msg::State current_state_;
    mavros_msgs::msg::ExtendedState current_extended_state_;
    sensor_msgs::msg::NavSatFix current_global_pos_;
    geometry_msgs::msg::TwistStamped current_vel_;
    geometry_msgs::msg::PoseStamped current_local_pose_;

    int mission_step_ = 0;
    bool transition_cmd_sent_ = false;
    GPSPoint wp1_, wp2_, wp3_, wp4_, rescue_loc_, drop_loc_;

    // Vision Vars
    geometry_msgs::msg::Point vision_error_;
    double fx_ = 500.0, fy_ = 500.0;
    bool manual_ok_signal_ = false;
    rclcpp::Time last_vision_time_;

    // [추가] 가속도 계산용 변수
    rclcpp::Time last_vel_time_;
    geometry_msgs::msg::Twist last_vel_data_;
    double current_accel_mag_ = 0.0;

    // Tuning Params
    const double FW_ACCEPTANCE_RADIUS = 20.0;
    const double MC_ACCEPTANCE_RADIUS = 1.5; 
    const double BACK_TRANSITION_RADIUS = 80.0; 
    const double P_TURN_DIST = 70.0;
    // [규정] 천이 가속도 제한 0.3G (약 2.94 m/s^2)
    const double TRANSITION_ACCEL_THRESHOLD = 0.3 * GRAVITY_ACCEL; 

    // ROS Handles
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_error_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_setpoint_pub_;
    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_setpoint_pub_;
    rclcpp::Publisher<krac_interfaces::msg::FlightPhase>::SharedPtr phase_pub_;
    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_trigger_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_label_pub_;

    void set_vision_target(std::string label) {
        std_msgs::msg::String msg;
        msg.data = label;
        target_label_pub_->publish(msg);
    }

    void load_param(std::string name, GPSPoint &pt) {
        std::vector<double> vec = this->get_parameter(name).as_double_array();
        if(vec.size() >= 3) { pt.lat = vec[0]; pt.lon = vec[1]; pt.alt = vec[2]; }
    }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void extended_state_cb(const mavros_msgs::msg::ExtendedState::SharedPtr msg) { current_extended_state_ = *msg; }
    void global_pos_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { current_global_pos_ = *msg; }
    
    // [수정] 속도 콜백에서 가속도 계산 로직 추가
    void local_vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { 
        current_vel_ = *msg; 
        
        rclcpp::Time now = msg->header.stamp;
        if (last_vel_time_.nanoseconds() > 0) {
            double dt = (now - last_vel_time_).seconds();
            if (dt > 0.001) { // 0으로 나누기 방지
                double dvx = current_vel_.twist.linear.x - last_vel_data_.linear.x;
                double dvy = current_vel_.twist.linear.y - last_vel_data_.linear.y;
                double dvz = current_vel_.twist.linear.z - last_vel_data_.linear.z;
                
                // 가속도 벡터의 크기 계산 (|a|)
                current_accel_mag_ = std::sqrt(dvx*dvx + dvy*dvy + dvz*dvz) / dt;
            }
        }
        last_vel_time_ = now;
        last_vel_data_ = current_vel_.twist;
    }

    void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_local_pose_ = *msg; }
    
    void vision_cb(const geometry_msgs::msg::Point::SharedPtr msg) { 
        vision_error_ = *msg; 
        if (msg->z > 0.5) {
            last_vision_time_ = this->now();
        }
    }
    
    void cam_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) { fx_ = msg->k[0]; fy_ = msg->k[4]; }
    void manual_proceed_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        manual_ok_signal_ = true;
        res->success = true;
        res->message = "Proceed Signal Received";
        RCLCPP_INFO(this->get_logger(), "Manual Trigger Received!");
    }

    double get_gps_distance(double lat1, double lon1, double lat2, double lon2) {
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        double a = std::pow(std::sin(dLat / 2), 2) + std::pow(std::sin(dLon / 2), 2) * std::cos(lat1*M_PI/180.0) * std::cos(lat2*M_PI/180.0);
        return EARTH_RADIUS * 2 * std::asin(std::sqrt(a));
    }

    double get_gps_bearing(double lat1, double lon1, double lat2, double lon2) {
        double y = std::sin((lon2 - lon1) * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0);
        double x = std::cos(lat1 * M_PI / 180.0) * std::sin(lat2 * M_PI / 180.0) -
                     std::sin(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) * std::cos((lon2 - lon1) * M_PI / 180.0);
        return std::atan2(y, x);
    }

    double check_yaw_aligned(double target_yaw) {
        double qx = current_local_pose_.pose.orientation.x;
        double qy = current_local_pose_.pose.orientation.y;
        double qz = current_local_pose_.pose.orientation.z;
        double qw = current_local_pose_.pose.orientation.w;
        
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double current_yaw = std::atan2(siny_cosp, cosy_cosp);

        double error = std::abs(target_yaw - current_yaw);
        if (error > M_PI) error = 2 * M_PI - error;
        return error;
    }

    GPSPoint get_extended_gps_point(GPSPoint start, GPSPoint end, double extend_meters) {
        double dLat = end.lat - start.lat;
        double dLon = end.lon - start.lon;
        double dist = get_gps_distance(start.lat, start.lon, end.lat, end.lon);
        if (dist < 0.1) return end; 
        double ratio = (dist + extend_meters) / dist;
        return {start.lat + dLat * ratio, start.lon + dLon * ratio, end.alt};
    }

    void run_vision_control(double target_alt, bool is_landing) {
        geometry_msgs::msg::Twist vel_cmd;
        double current_z = current_local_pose_.pose.position.z;
        double current_h = std::max(current_z, 0.5);

        double err_x_m = (vision_error_.x * current_h) / fx_;
        double err_y_m = (vision_error_.y * current_h) / fy_;

        double kp = 1.0; 
        vel_cmd.linear.x = kp * (-err_y_m); 
        vel_cmd.linear.y = kp * (err_x_m);  
        
        if (current_z > target_alt + 0.1) vel_cmd.linear.z = is_landing ? -0.5 : -0.3;
        else if (current_z < target_alt - 0.1) vel_cmd.linear.z = 0.3;
        else vel_cmd.linear.z = 0.0;

        vel_setpoint_pub_->publish(vel_cmd);
    }
    
    void run_search_pattern(double target_alt) {
        static double search_timer = 0.0;
        search_timer += 0.05; 

        geometry_msgs::msg::Twist vel_cmd;
        vel_cmd.linear.x = 1.0 * std::sin(0.33 * search_timer);
        vel_cmd.linear.y = 0.0; 

        double current_z = current_local_pose_.pose.position.z;
        if (current_z > target_alt + 0.1) vel_cmd.linear.z = -0.3;
        else if (current_z < target_alt - 0.1) vel_cmd.linear.z = 0.3;
        else vel_cmd.linear.z = 0.0;

        vel_setpoint_pub_->publish(vel_cmd);
    }

    void control_loop()
    {
        if (!rclcpp::ok()) return;
        if (current_global_pos_.status.status < 0 && mission_step_ > 0) return;

        mavros_msgs::msg::GlobalPositionTarget target_msg;
        target_msg.header.stamp = this->now();
        target_msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT; 
        target_msg.type_mask = 0b110111111000; 

        GPSPoint current_pt = {current_global_pos_.latitude, current_global_pos_.longitude, current_global_pos_.altitude};
        bool use_vision_control = false; 
        bool ignore_yaw = true; 

        switch (mission_step_)
        {
        case 0: /* Wait for GPS */
            if (current_state_.connected && current_global_pos_.status.status >= 0) {
                static int c = 0; if(++c > 50) mission_step_ = 1;
            }
            break;

        case 1: /* Request Offboard */
            target_msg.latitude = current_global_pos_.latitude;
            target_msg.longitude = current_global_pos_.longitude;
            target_msg.altitude = 0.0; 
            request_offboard_mode(); 
            break;

        case 2: /* Takeoff to 30m */
            target_msg.latitude = wp1_.lat; target_msg.longitude = wp1_.lon; 
            target_msg.altitude = 30.0; 
            
            if(current_local_pose_.pose.position.z > 29.0) {
                RCLCPP_INFO(this->get_logger(), "Reached 30m Altitude. Starting Heading Alignment.");
                mission_step_ = 3;
            }
            break;
            
        case 3: /* WP1 -> WP2 Heading Align (at 30m) */
            target_msg.latitude = wp1_.lat; 
            target_msg.longitude = wp1_.lon; 
            target_msg.altitude = 30.0; 
            
            target_msg.yaw = get_gps_bearing(wp1_.lat, wp1_.lon, wp2_.lat, wp2_.lon);
            ignore_yaw = false; 
            
            {
                static int align_timer = 0;       
                static int stable_count = 0;      
                
                double current_yaw_error = check_yaw_aligned(target_msg.yaw);
                align_timer++;

                if (current_yaw_error < 0.1) stable_count++;
                else stable_count = 0; 

                if (align_timer % 20 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Aligning Yaw... Err: %.2f rad", current_yaw_error);
                }

                if (stable_count > 20 || align_timer > 200) {
                    // [수정] 바로 가속하지 않고 4초 호버링 대기 단계(35)로 이동
                    mission_step_ = 35; 
                    align_timer = 0; stable_count = 0;
                    RCLCPP_INFO(this->get_logger(), "Alignment Complete. Hovering 4s before Accel.");
                }
            }
            break;

        case 35: /* [추가됨] 4 Seconds Hovering Logic */
            target_msg.latitude = wp1_.lat; 
            target_msg.longitude = wp1_.lon; 
            target_msg.altitude = 30.0; 
            
            // 호버링 중에도 기수는 WP2 방향 유지
            target_msg.yaw = get_gps_bearing(wp1_.lat, wp1_.lon, wp2_.lat, wp2_.lon);
            ignore_yaw = false; 

            {
                static int hover_wait_timer = 0;
                hover_wait_timer++;

                if (hover_wait_timer % 20 == 0) {
                     RCLCPP_INFO(this->get_logger(), "Hovering before transition... %d/4 sec", hover_wait_timer / 20);
                }

                // 4초 대기 (50ms * 80 = 4000ms)
                if (hover_wait_timer >= 80) { 
                    mission_step_ = 4;
                    hover_wait_timer = 0; // 초기화
                    RCLCPP_INFO(this->get_logger(), "Hover complete. Starting Acceleration for Transition.");
                }
            }
            break;

        case 4: /* Accelerate & Transition on 0.3G */
            target_msg.latitude = wp2_.lat; 
            target_msg.longitude = wp2_.lon; 
            target_msg.altitude = 30.0; 
            
            ignore_yaw = true; 

            if (!transition_cmd_sent_) {
                if (current_accel_mag_ >= TRANSITION_ACCEL_THRESHOLD) {
                    RCLCPP_INFO(this->get_logger(), "Accel Reached %.2f m/s^2 (>= 0.3G). Triggering Transition!", current_accel_mag_);
                    perform_transition(true); 
                    transition_cmd_sent_ = true;
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Accelerating... Current: %.2f / Target: %.2f m/s^2", current_accel_mag_, TRANSITION_ACCEL_THRESHOLD);
                    
                    double dist_to_wp2 = get_gps_distance(current_pt.lat, current_pt.lon, wp2_.lat, wp2_.lon);
                    if (dist_to_wp2 < 50.0) {
                         RCLCPP_WARN(this->get_logger(), "Approaching WP2 without reaching 0.3G. Forcing Transition.");
                         perform_transition(true);
                         transition_cmd_sent_ = true;
                    }
                }
            }

            if (current_extended_state_.vtol_state == MAV_VTOL_STATE_FW) {
                RCLCPP_INFO(this->get_logger(), "Transition Complete (FW Mode). Continuing to WP2.");
                mission_step_ = 5;
            }
            break;

        case 5: /* Check WP2 Arrival */
            target_msg.latitude = wp2_.lat; target_msg.longitude = wp2_.lon; target_msg.altitude = wp2_.alt;
            if (get_gps_distance(current_pt.lat, current_pt.lon, wp2_.lat, wp2_.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 55;
            break;

        case 55: /* P-Turn Extension */
            {
                GPSPoint ext = get_extended_gps_point(wp1_, wp2_, P_TURN_DIST);
                target_msg.latitude = ext.lat; target_msg.longitude = ext.lon; target_msg.altitude = ext.alt;
                if (get_gps_distance(current_pt.lat, current_pt.lon, ext.lat, ext.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 6;
            }
            break;

        case 6: /* Fly to WP3 */
            target_msg.latitude = wp3_.lat; target_msg.longitude = wp3_.lon; target_msg.altitude = wp3_.alt;
            if (get_gps_distance(current_pt.lat, current_pt.lon, wp3_.lat, wp3_.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 66;
            break;

        case 66: /* WP3 Extension */
            {
                GPSPoint ext = get_extended_gps_point(wp2_, wp3_, P_TURN_DIST);
                target_msg.latitude = ext.lat; target_msg.longitude = ext.lon; target_msg.altitude = ext.alt;
                if (get_gps_distance(current_pt.lat, current_pt.lon, ext.lat, ext.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 7;
            }
            break;

        case 7: /* Fly to WP4 */
            target_msg.latitude = wp4_.lat; target_msg.longitude = wp4_.lon; target_msg.altitude = wp4_.alt;
            if (get_gps_distance(current_pt.lat, current_pt.lon, wp4_.lat, wp4_.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 77;
            break;

        case 77: /* WP4 Extension */
            {
                GPSPoint ext = get_extended_gps_point(wp3_, wp4_, P_TURN_DIST);
                target_msg.latitude = ext.lat; target_msg.longitude = ext.lon; target_msg.altitude = ext.alt;
                if (get_gps_distance(current_pt.lat, current_pt.lon, ext.lat, ext.lon) < FW_ACCEPTANCE_RADIUS) mission_step_ = 8;
            }
            break;

        case 8: /* Return & Back Transition */
            target_msg.latitude = wp2_.lat; target_msg.longitude = wp2_.lon; target_msg.altitude = wp2_.alt;
            if (get_gps_distance(current_pt.lat, current_pt.lon, wp2_.lat, wp2_.lon) < BACK_TRANSITION_RADIUS) {
                if(current_extended_state_.vtol_state == MAV_VTOL_STATE_FW) perform_transition(false); 
                mission_step_ = 9;
            }
            break;

        case 9: /* Approach Rescue (High) */
            target_msg.latitude = rescue_loc_.lat; target_msg.longitude = rescue_loc_.lon; target_msg.altitude = 20.0;
            if (current_extended_state_.vtol_state != MAV_VTOL_STATE_MC) perform_transition(false);
            else {
                if(get_gps_distance(current_pt.lat, current_pt.lon, rescue_loc_.lat, rescue_loc_.lon) < MC_ACCEPTANCE_RADIUS) mission_step_ = 10;
            }
            break;

        case 10: /* Approach Rescue (Low + Vision Check) */
            set_vision_target("box");
            target_msg.latitude = rescue_loc_.lat; target_msg.longitude = rescue_loc_.lon; target_msg.altitude = rescue_loc_.alt;
            target_msg.yaw = get_gps_bearing(current_pt.lat, current_pt.lon, rescue_loc_.lat, rescue_loc_.lon);
            
            ignore_yaw = false; 

            if (get_gps_distance(current_pt.lat, current_pt.lon, rescue_loc_.lat, rescue_loc_.lon) < MC_ACCEPTANCE_RADIUS) {
                RCLCPP_INFO(this->get_logger(), "Vision Control Start!");
                mission_step_ = 11;
            }
            break;

        case 11: /* Vision Hover (Rescue) */
            use_vision_control = true; 
            if ((this->now() - last_vision_time_).seconds() < 1.0) run_vision_control(1.0, false); 
            else run_search_pattern(1.0); 

            if (std::abs(current_local_pose_.pose.position.z - 1.0) < 0.2 && 
                (this->now() - last_vision_time_).seconds() < 0.5 && 
                manual_ok_signal_) {
                    manual_ok_signal_ = false; mission_step_ = 12;
                    RCLCPP_INFO(this->get_logger(), "Rescue OK! Moving to Drop Point.");
            }
            break;

        case 12: /* Move to Drop */
            set_vision_target("drop_zone");
            if (current_local_pose_.pose.position.z < 15.0) {
                 target_msg.latitude = rescue_loc_.lat; target_msg.longitude = rescue_loc_.lon; target_msg.altitude = 15.0;
            } else {
                 target_msg.latitude = drop_loc_.lat; target_msg.longitude = drop_loc_.lon; target_msg.altitude = 15.0;
                 if (get_gps_distance(current_pt.lat, current_pt.lon, drop_loc_.lat, drop_loc_.lon) < MC_ACCEPTANCE_RADIUS) {
                     mission_step_ = 13;
                 }
            }
            break;

        case 13: /* Vision Hover (Drop) */
            use_vision_control = true;
            if ((this->now() - last_vision_time_).seconds() < 1.0) run_vision_control(1.0, false);
            else run_search_pattern(1.0);

            if (std::abs(current_local_pose_.pose.position.z - 1.0) < 0.2 && 
                (this->now() - last_vision_time_).seconds() < 0.5 &&
                manual_ok_signal_) {
                    manual_ok_signal_ = false; mission_step_ = 14;
                    RCLCPP_INFO(this->get_logger(), "Drop OK! Returning Home.");
            }
            break;

        case 14: /* Return Home */
            set_vision_target("vertiport");
            target_msg.latitude = wp1_.lat; target_msg.longitude = wp1_.lon; target_msg.altitude = 20.0;
            if (get_gps_distance(current_pt.lat, current_pt.lon, wp1_.lat, wp1_.lon) < MC_ACCEPTANCE_RADIUS) {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "AUTO.LAND";
                set_mode_client_->async_send_request(req);
                mission_step_ = 99;
            }
            break;
        }

        if (ignore_yaw) target_msg.type_mask |= mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW;
        else target_msg.type_mask &= ~mavros_msgs::msg::GlobalPositionTarget::IGNORE_YAW;

        if (!use_vision_control) global_setpoint_pub_->publish(target_msg);
        
        krac_interfaces::msg::FlightPhase phase_msg;
        phase_msg.current_phase = mission_step_;
        phase_pub_->publish(phase_msg);
    }
    
    void request_offboard_mode() {
        if (current_state_.mode != "OFFBOARD") {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
        } else if (!current_state_.armed) {
            auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            req->value = true;
            arming_client_->async_send_request(req);
        } else {
             mission_step_ = 2;
        }
    }

    void perform_transition(bool to_fixed_wing) {
        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = 3000; 
        req->confirmation = 0;
        req->param1 = to_fixed_wing ? 4.0 : 3.0; 
        command_client_->async_send_request(req);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VtolGpsMissionNode>());
    rclcpp::shutdown();
    return 0;
}
