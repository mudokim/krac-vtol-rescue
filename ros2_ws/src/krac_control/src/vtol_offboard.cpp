#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp> 
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

enum MissionPhase {
    PHASE_TAKEOFF = 0,
    PHASE_MOVE_TO_RESCUE = 1,
    PHASE_HOVER_RESCUE = 2,
    PHASE_RESCUE_DESCENT = 3,
    PHASE_LANDING_AND_GRAB = 4,
    PHASE_ASCEND_WITH_VICTIM = 5,
    PHASE_RETURN_HOME = 6,
    PHASE_HOVER_HOME = 7,
    PHASE_HOME_DESCENT = 8,
    PHASE_FINAL_LANDING = 9
};

class OffboardMissionNode : public rclcpp::Node {
public:
    OffboardMissionNode() : Node("offboard_mission_node") {
        auto qos = rclcpp::SensorDataQoS();

        // 🎯 목표 좌표 설정
        rescue_x_ = -22.48;
        rescue_y_ = -31.59;
        hover_alt_ = 10.0;

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, std::bind(&OffboardMissionNode::state_cb, this, std::placeholders::_1));
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose", qos, std::bind(&OffboardMissionNode::local_pose_cb, this, std::placeholders::_1));
        lander_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/precision_lander/cmd_vel", 10, std::bind(&OffboardMissionNode::lander_vel_cb, this, std::placeholders::_1));

        // 위치 제어와 속도 제어 퍼블리셔 분리
        pos_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        vel_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        lander_client_ = this->create_client<std_srvs::srv::SetBool>("precision_lander/enable");

        manual_trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
            "cmd/mission_proceed", std::bind(&OffboardMissionNode::manual_proceed_cb, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardMissionNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "🚀 OFFBOARD 전 구간 비행 & 정밀 착륙 테스트 시작");
        RCLCPP_INFO(this->get_logger(), "목적지: X=%.2f, Y=%.2f", rescue_x_, rescue_y_);
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

private:
    MissionPhase current_phase_ = PHASE_TAKEOFF;
    double rescue_x_, rescue_y_, hover_alt_;
    bool manual_ok_ = false;
    int hover_counter_ = 0;
    
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_local_pose_;
    geometry_msgs::msg::Twist latest_lander_vel_; 
    rclcpp::Time last_lander_vel_time_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lander_vel_sub_; 
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_setpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_setpoint_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr lander_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_trigger_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_local_pose_ = *msg; }
    void lander_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_lander_vel_ = *msg;
        last_lander_vel_time_ = this->now();
    }

    void manual_proceed_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        manual_ok_ = true;
        res->success = true;
        RCLCPP_WARN(this->get_logger(), "▶️ [수동 개입] 임무 속행 승인!");
    }

    void set_precision_lander(bool enable) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = enable;
        lander_client_->async_send_request(req);
    }

    // 목표 좌표 도달 확인 함수
    bool is_reached(double target_x, double target_y, double target_z, double threshold = 0.5) {
        double dx = current_local_pose_.pose.position.x - target_x;
        double dy = current_local_pose_.pose.position.y - target_y;
        double dz = current_local_pose_.pose.position.z - target_z;
        return sqrt(dx*dx + dy*dy + dz*dz) < threshold;
    }

    void control_loop() {
        geometry_msgs::msg::PoseStamped pos_cmd;
        geometry_msgs::msg::Twist vel_cmd;
        bool use_velocity = false; // 기본은 위치 제어 사용

        // [중요] OFFBOARD 모드 유지
        if (current_state_.mode != "OFFBOARD" && current_phase_ != PHASE_FINAL_LANDING) {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
            
            // 모드 전환 중에는 제자리 호버링 명령 (안전)
            pos_cmd.pose.position.x = current_local_pose_.pose.position.x;
            pos_cmd.pose.position.y = current_local_pose_.pose.position.y;
            pos_cmd.pose.position.z = current_local_pose_.pose.position.z;
            pos_setpoint_pub_->publish(pos_cmd);
            return;
        }

        switch (current_phase_) {
            case PHASE_TAKEOFF:
                pos_cmd.pose.position.x = 0.0;
                pos_cmd.pose.position.y = 0.0;
                pos_cmd.pose.position.z = hover_alt_;
                
                if (is_reached(0.0, 0.0, hover_alt_)) {
                    RCLCPP_INFO(this->get_logger(), "✅ 이륙 완료. 구조 지점으로 이동합니다.");
                    current_phase_ = PHASE_MOVE_TO_RESCUE;
                }
                break;

            case PHASE_MOVE_TO_RESCUE:
                pos_cmd.pose.position.x = rescue_x_;
                pos_cmd.pose.position.y = rescue_y_;
                pos_cmd.pose.position.z = hover_alt_;

                if (is_reached(rescue_x_, rescue_y_, hover_alt_)) {
                    RCLCPP_INFO(this->get_logger(), "✅ 구조 지점 도착. 호버링 안정화 대기 중...");
                    current_phase_ = PHASE_HOVER_RESCUE;
                    hover_counter_ = 0;
                }
                break;

            case PHASE_HOVER_RESCUE:
                pos_cmd.pose.position.x = rescue_x_;
                pos_cmd.pose.position.y = rescue_y_;
                pos_cmd.pose.position.z = hover_alt_;
                hover_counter_++;
                
                if (hover_counter_ > 60) { // 3초 대기
                    RCLCPP_WARN(this->get_logger(), "🚨 구조 정밀 하강 시작!");
                    set_precision_lander(true);
                    current_phase_ = PHASE_RESCUE_DESCENT;
                }
                break;

            case PHASE_RESCUE_DESCENT:
                use_velocity = true; // 비전 속도 제어로 변경
                vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; vel_cmd.linear.z = -0.5; // 기본 하강

                if (last_lander_vel_time_.nanoseconds() > 0 && (this->now() - last_lander_vel_time_).seconds() < 0.5) {
                    vel_cmd = latest_lander_vel_; // 비전 노드 명령 덮어쓰기
                }
                
                if (current_local_pose_.pose.position.z < 0.4) {
                    set_precision_lander(false);
                    manual_ok_ = false; 
                    current_phase_ = PHASE_LANDING_AND_GRAB;
                }
                break;

            case PHASE_LANDING_AND_GRAB:
                use_velocity = true;
                vel_cmd.linear.z = -0.2; // 바닥 밀착
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "⏳ [파지 대기] 구조 완료 후 '/cmd/mission_proceed' 서비스를 호출하세요.");

                if (manual_ok_) {
                    manual_ok_ = false;
                    current_phase_ = PHASE_ASCEND_WITH_VICTIM;
                }
                break;

            case PHASE_ASCEND_WITH_VICTIM:
                pos_cmd.pose.position.x = rescue_x_;
                pos_cmd.pose.position.y = rescue_y_;
                pos_cmd.pose.position.z = hover_alt_;
                
                if (is_reached(rescue_x_, rescue_y_, hover_alt_)) {
                    RCLCPP_INFO(this->get_logger(), "✅ 재상승 완료. 버티포트(Home)로 복귀합니다.");
                    current_phase_ = PHASE_RETURN_HOME;
                }
                break;

            case PHASE_RETURN_HOME:
                pos_cmd.pose.position.x = 0.0;
                pos_cmd.pose.position.y = 0.0;
                pos_cmd.pose.position.z = hover_alt_;

                if (is_reached(0.0, 0.0, hover_alt_)) {
                    RCLCPP_INFO(this->get_logger(), "✅ 홈 도착. 호버링 안정화 대기 중...");
                    current_phase_ = PHASE_HOVER_HOME;
                    hover_counter_ = 0;
                }
                break;

            case PHASE_HOVER_HOME:
                pos_cmd.pose.position.x = 0.0;
                pos_cmd.pose.position.y = 0.0;
                pos_cmd.pose.position.z = hover_alt_;
                hover_counter_++;

                if (hover_counter_ > 60) {
                    RCLCPP_WARN(this->get_logger(), "🛬 버티포트 정밀 하강 시작!");
                    set_precision_lander(true);
                    current_phase_ = PHASE_HOME_DESCENT;
                }
                break;

            case PHASE_HOME_DESCENT:
                use_velocity = true;
                vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; vel_cmd.linear.z = -0.5;

                if (last_lander_vel_time_.nanoseconds() > 0 && (this->now() - last_lander_vel_time_).seconds() < 0.5) {
                    vel_cmd = latest_lander_vel_;
                }
                
                if (current_local_pose_.pose.position.z < 0.25) {
                    set_precision_lander(false);
                    current_phase_ = PHASE_FINAL_LANDING;
                }
                break;

            case PHASE_FINAL_LANDING:
                {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "AUTO.LAND";
                    set_mode_client_->async_send_request(req);
                    RCLCPP_WARN(this->get_logger(), "🏁 [테스트 종료] AUTO.LAND로 최종 착륙합니다.");
                }
                break;
        }

        // 제어 방식에 따라 MAVROS에 퍼블리시
        if (use_velocity) {
            vel_setpoint_pub_->publish(vel_cmd);
        } else {
            pos_setpoint_pub_->publish(pos_cmd);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardMissionNode>());
    rclcpp::shutdown();
    return 0;
}
