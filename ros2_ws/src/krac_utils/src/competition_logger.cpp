#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp> // [변경] GPS 메시지
#include <mavros_msgs/msg/state.hpp>
#include <krac_interfaces/msg/flight_phase.hpp>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <vector>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

// [변경] GPS 좌표 구조체
struct Waypoint {
    int id;
    double lat; double lon; double alt;
};

// [상수] 지구 반지름 (거리 계산용)
#define EARTH_RADIUS 6371000.0
#define M_PI 3.14159265358979323846

class CompetitionLogger : public rclcpp::Node
{
public:
    CompetitionLogger() : Node("competition_logger")
    {
        auto qos_profile = rclcpp::SensorDataQoS();

        this->declare_parameter("log_dir", "");

        // 1. 파라미터 로딩 (YAML 파일과 동일한 GPS 좌표 사용)
        this->declare_parameter("wp1", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("wp2", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("wp3", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("wp4", std::vector<double>{0.0, 0.0, 0.0});

        std::vector<double> p1 = this->get_parameter("wp1").as_double_array();
        std::vector<double> p2 = this->get_parameter("wp2").as_double_array();
        std::vector<double> p3 = this->get_parameter("wp3").as_double_array();
        std::vector<double> p4 = this->get_parameter("wp4").as_double_array();

        // 감시 리스트에 등록 (순서: Lat, Lon, Alt)
        waypoints_.push_back({1, p1[0], p1[1], p1[2]});
        waypoints_.push_back({2, p2[0], p2[1], p2[2]});
        waypoints_.push_back({3, p3[0], p3[1], p3[2]});
        waypoints_.push_back({4, p4[0], p4[1], p4[2]});

        RCLCPP_INFO(this->get_logger(), "Logger Watchlist Loaded (GPS): WP1(%.6f, %.6f)...", 
                    p1[0], p1[1]);

        // 2. Subscribers [변경됨]
        // 로컬 포지션 대신 GPS 데이터 구독
        global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "mavros/global_position/global", qos_profile, std::bind(&CompetitionLogger::global_pos_cb, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos_profile, std::bind(&CompetitionLogger::state_cb, this, std::placeholders::_1));

        phase_sub_ = this->create_subscription<krac_interfaces::msg::FlightPhase>(
            "krac/mission_phase", 10, std::bind(&CompetitionLogger::phase_cb, this, std::placeholders::_1));

        // 3. 파일 생성
        create_log_file();

        // 4. 타이머 (10Hz) - 규정: 10Hz 샘플링 
        timer_ = this->create_wall_timer(100ms, std::bind(&CompetitionLogger::log_data, this));
    }

private:
    sensor_msgs::msg::NavSatFix current_gps_; // [변경] 현재 위치 저장용
    mavros_msgs::msg::State current_state_;
    int current_phase_ = 0;
    
    std::ofstream log_file_;
    std::string file_path_;
    
    bool logging_started_ = false;
    const double TRIGGER_RADIUS = 5.0; // 미터 단위 감지 반경
    std::vector<Waypoint> waypoints_;
    int detected_wp_id_ = 0; 

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_; // [변경]
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<krac_interfaces::msg::FlightPhase>::SharedPtr phase_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void global_pos_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { current_gps_ = *msg; }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void phase_cb(const krac_interfaces::msg::FlightPhase::SharedPtr msg) { current_phase_ = msg->current_phase; }

    // [헬퍼] GPS 거리 계산 (Haversine Formula)
    double get_gps_distance(double lat1, double lon1, double lat2, double lon2) {
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        lat1 = lat1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;

        double a = std::pow(std::sin(dLat / 2), 2) +
                   std::pow(std::sin(dLon / 2), 2) * std::cos(lat1) * std::cos(lat2);
        double c = 2 * std::asin(std::sqrt(a));
        return EARTH_RADIUS * c;
    }

    void create_log_file()
    {
        const char* home_env = std::getenv("HOME");
        if (home_env == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "HOME environment variable is not set.");
            return;
        }

        std::string home_path = home_env;
        std::string log_dir = this->get_parameter("log_dir").as_string();
        if (log_dir.empty()) {
            // 기본값: 현재 실행 위치 기준 logs/competition.
            // ros2 launch를 repo/ros2_ws에서 실행하면 repo/ros2_ws/logs/competition에 생성됩니다.
            log_dir = std::filesystem::current_path().string() + "/logs/competition";
        }

        // 로그 저장 폴더 생성
        std::filesystem::create_directories(log_dir);

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << log_dir << "/flight_log_gps_"
        << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S")
        << ".csv";

        file_path_ = ss.str();
        log_file_.open(file_path_);

        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create log file at: %s", file_path_.c_str());
            return;
        }

        // [규정 준수] 헤더 변경 (형식2: LLA 좌표)
        // "자동, 수동 Flag", "경로점 Flag", "GPS 시각", "위도", "경도", "고도"
        log_file_ << "Auto_Manual,Event_Flag,GPS_Time,Latitude,Longitude,Altitude\n";

        RCLCPP_INFO(this->get_logger(), "Log file created at: %s", file_path_.c_str());
    }
    void check_waypoints_proximity()
    {
        // GPS 수신 전이면 패스
        if (current_gps_.status.status < 0) return;

        detected_wp_id_ = 0;
        double min_dist = 9999.0;
        int closest_wp = -1;

        for (const auto& wp : waypoints_) {
            // [변경] GPS 거리 계산 함수 사용
            double dist = get_gps_distance(current_gps_.latitude, current_gps_.longitude, wp.lat, wp.lon);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_wp = wp.id;
            }

            if (dist <= TRIGGER_RADIUS) {
                if (!logging_started_) {
                    logging_started_ = true;
                    RCLCPP_WARN(this->get_logger(), ">>> LOGGING STARTED! Reached WP%d (Dist: %.2fm) <<<", wp.id, dist);
                }
                detected_wp_id_ = wp.id;
            }
        }

        if (!logging_started_) {
            static int print_count = 0;
            if (print_count++ % 10 == 0) { 
                RCLCPP_INFO(this->get_logger(), "Waiting... Closest WP%d is %.1fm away", closest_wp, min_dist);
            }
        }
    }

    void log_data()
    {
        if (!log_file_.is_open()) return;
        
        check_waypoints_proximity();

        if (!logging_started_) return;

        int auto_flag = (current_state_.mode == "OFFBOARD") ? 1 : 0;
        int event_flag = (detected_wp_id_ > 0) ? detected_wp_id_ : current_phase_;
        
        // GPS Timestamp (메시지 시간 사용)
        double current_time = rclcpp::Time(current_gps_.header.stamp).seconds();

        // [규정 준수] 데이터 저장 형식 (소수점 자릿수 규정: 위경도 6자리, 고도 0.1m) 
        log_file_ << auto_flag << "," 
                  << event_flag << "," 
                  << std::fixed << std::setprecision(3) << current_time << ","
                  << std::setprecision(6) << current_gps_.latitude << ","
                  << std::setprecision(6) << current_gps_.longitude << ","
                  << std::setprecision(1) << current_gps_.altitude << "\n";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompetitionLogger>());
    rclcpp::shutdown();
    return 0;
}
