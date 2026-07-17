#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <krac_interfaces/msg/target_error.hpp> // 커스텀 메시지
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class PrecisionLander : public rclcpp::Node {
public:
    PrecisionLander() : Node("precision_lander_node") {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/precision_lander/cmd_vel", 10);
        vision_sub_ = this->create_subscription<krac_interfaces::msg::TargetError>("/vision/target_error", 10, std::bind(&PrecisionLander::vision_cb, this, std::placeholders::_1));

        auto qos = rclcpp::SensorDataQoS();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&PrecisionLander::pose_cb, this, std::placeholders::_1));

        enable_srv_ = this->create_service<std_srvs::srv::SetBool>("/precision_lander/enable", std::bind(&PrecisionLander::enable_cb, this, std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(50ms, std::bind(&PrecisionLander::control_loop, this));

        // 실측(2026-07-15): 픽셀오차를 0 으로 몰면 '카메라'가 타깃 위에 온다. 그런데
        // 카메라는 base_link 기준 body x=+0.2, 그리퍼는 x=0 이라 카메라가 타깃 위면
        // 그리퍼는 0.2m 뒤에 남는다. 실제로 착륙 후 그리퍼-트레이 거리가 0.168m 였고
        // (트레이 짧은변 반폭 68.5mm) 손가락이 허공을 조여서 파지가 매번 실패했다.
        // 목표를 이 오프셋만큼 편향시켜 '그리퍼'가 타깃 위에 오게 한다.
        // 축 매핑: body_x = -err_y_m , body_y = +err_x_m (아래 주석 참고).
        // -> 타깃을 카메라 기준 body x=-0.2 에 두려면 err_y_m 목표가 +0.2 가 된다.
        // 부호는 이 파일에서 과거에도 실측으로 뒤집힌 이력이 있으니 파라미터로 뺀다.
        //
        // ⚠️ 단위 주의: 이 값은 '참 미터'가 아니라 **아래 fx_/fy_ 로 환산된 lander 단위**다.
        // fx_/fy_ 는 실제 카메라와 안 맞는다(2026-07-17 확인):
        //   카메라 = PX4 스톡 mono_cam, 1280x960, hfov=1.74rad -> fx_orig=fy_orig=539.9
        //   vision_tracker 가 1024x1024 로 리사이즈(4:3 -> 1:1, 비등방: x0.8 / y1.0667)
        //   -> 참값은 fx=431.9, fy=575.9 인데 아래는 582.5 / 1036.7 (fx 1.35x, fy 1.80x 과대)
        // 오프셋이 0 이던 시절엔 이게 안 드러났다. err_m -> 0 을 몰면 px -> 0 이라
        // 수렴점이 f 와 무관했기 때문이다(f 는 게인만 바꿨다). 하지만 이 오프셋은
        // '타깃을 화면중심에서 물리적으로 얼마나 떨어뜨릴까'라서 f 가 수렴점에 직접
        // 곱해진다 -> 0.20 을 넣으면 실제로는 0.20*1.80 = 0.36m 가 걸려 그리퍼가
        // 바구니(짧은변 138mm, 반폭 69mm)를 ~0.16m 지나쳐 내려앉았다(실측 확인).
        // 여기서 원하는 참값은 0.20m 이므로 0.20 * (575.9/1036.7) = 0.111 을 넣는다.
        //
        // fx_/fy_ 를 참값으로 고치는 게 정공법이지만, ALIGNED_RADIUS_M / APPROACH_RADIUS_M /
        // 데드밴드 / lateral_scale 이 전부 이 틀린 f 기준으로 실측 튜닝된 값이라
        // 같이 재튜닝해야 한다. 그건 별건으로 두고 여기선 오프셋만 바로잡는다.
        this->declare_parameter("grasp_offset_err_y_m", 0.111);
        this->declare_parameter("grasp_offset_err_x_m", 0.0);
        grasp_offset_err_y_m_ = this->get_parameter("grasp_offset_err_y_m").as_double();
        grasp_offset_err_x_m_ = this->get_parameter("grasp_offset_err_x_m").as_double();
        RCLCPP_INFO(this->get_logger(),
            "파지 정렬 오프셋: err_x 목표=%.3f, err_y 목표=%.3f (lander 단위) "
            "-> 실제 %.3fm / %.3fm (fx_=%.1f fy_=%.1f 기준)",
            grasp_offset_err_x_m_, grasp_offset_err_y_m_,
            grasp_offset_err_x_m_ * (582.5 / 431.9), grasp_offset_err_y_m_ * (1036.7 / 575.9),
            fx_, fy_);

        RCLCPP_INFO(this->get_logger(), "🛬 정밀 착륙(Precision Lander) 가동! (0도/180도 최단거리 정렬 적용)");
    }

private:
    bool enabled_ = false;
    krac_interfaces::msg::TargetError vision_err_;
    rclcpp::Time last_detection_time_;
    double current_alt_ = 0.0;
    double current_yaw_ = 0.0;

    // 실측 확인: 정렬 오차가 0.02~0.1m 수준으로 거의 완벽해도, 구조 박스에
    // 근접(고도 ~0.5~1.6m)하면 YOLO/ArUco가 예외 없이 타겟을 놓친다(마커가
    // 화면을 너무 꽉 채우는 근접 촬영 각도/스케일 한계로 추정, 정렬/제어 문제
    // 아님). 이 근접 구간에서는 마지막으로 확인된 정렬을 신뢰하고 눈 감고
    // 그대로 하강을 이어간다 — 그래야 PrecisionLandOnTarget의 고도 기준
    // 성공 조건(비전과 무관)에 도달할 수 있다. 더 높은 고도에서 타겟을 잃으면
    // (아직 제대로 정렬된 적 없거나 실제로 이탈한 경우) 기존처럼 제자리 대기 후
    // BT가 REP로 복귀시켜 재시도하게 둔다.
    rclcpp::Time last_aligned_time_;
    bool has_been_aligned_ = false;
    const double CLOSE_RANGE_LOCK_ALT_M = 3.0;
    const double ALIGN_GRACE_SEC = 30.0;

    // 실측 확인: 초기 정렬 오차가 큰 시도(예: 3~20m)에서는 XY 정렬에 시간이
    // 걸리는 동안(dist_m>=1.5, 아래 z=0 명령) 고도가 계속 서서히 상승하는
    // 현상이 반복 확인됐다(예: 4.98m -> 5.27m, 11.10m -> 12.11m). 픽셀 오차는
    // 거의 안 변하는데 미터 환산 오차만 계속 커지는 로그 패턴으로 볼 때, 실제
    // 수평 이탈이 아니라 alt_factor(=현재 고도)가 커지면서 같은 픽셀 오차가
    // 더 큰 미터 오차로 환산되는 것 — 즉 원인은 "정렬 실패"가 아니라 "정렬 중
    // 고도가 안 잡히는 것"이었다. z=0.0을 순수 속도 0으로만 명령하면 OFFBOARD
    // 진입 직전 잔여 상승 관성/드리프트를 능동적으로 상쇄하지 못하는 것으로
    // 보여, Lander가 켜진 시점의 고도를 기준 고도로 잡아 정렬 단계 내내
    // 그 고도를 능동적으로 유지(비례 제어)하도록 바꿨다.
    double hold_alt_m_ = 0.0;
    const double KP_ALT_HOLD = 0.4;
    const double MAX_ALT_HOLD_VEL = 0.4;

    // 실제 카메라 캘리브레이션을 1024x1024 해상도 비율로 변환한 값
    const double fx_ = 582.5;
    const double fy_ = 1036.7;

    // PID 제어기 변수
    double prev_err_x_ = 0.0;
    double prev_err_y_ = 0.0;
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;

    // OBB 각도는 프레임마다 0도<->90도 근처로 튀는 노이즈(사각형 대칭성으로 인한
    // 각도 모호성)가 심해서, 이를 그대로 yaw 속도 명령에 넣으면 기체가 실제로
    // 계속 돌아버려서(요동) 카메라 시야에서 타겟이 이탈하는 문제가 있었다.
    // 지수평활(저역통과) 필터로 튀는 값을 걸러낸다.
    double filtered_yaw_err_ = 0.0;
    bool yaw_filter_initialized_ = false;
    const double YAW_FILTER_ALPHA = 0.12;

    // 요동침(Oscillation) 방지를 위해 PID 게인 및 최대 속도 대폭 하향 조정
    // Conservative P-only control for pickup descent.
    const double KP_XY = 0.18;
    const double KI_XY = 0.0;
    const double KD_XY = 0.0;
    const double MAX_XY_VEL = 0.30;

    // 헤딩(Yaw) 정렬 파라미터 (회전 떨림도 방지하기 위해 하향)
    const double KP_YAW = 0.3;
    const double MAX_YAW_VEL = 0.2;

    // 고도 강하 파라미터
    const double SEARCH_ALTITUDE = 7.0;
    const double BLIND_DESCENT_SPEED = -0.12;
    const double PRECISION_DESCEND_SPEED = -0.35;
    const double TARGET_FRESH_MAX_AGE_SEC = 3.0;

    // 실측(2026-07-15): 정렬 오차가 0.30~0.70m 대역에서 계속 머물러 하강이
    // 0.12m/s로 기었고, 0.70m를 넘길 때마다(60초 동안 21회) 고도 유지로 빠져
    // 아예 안 내려갔다. 결과적으로 3.75m에서 60초 동안 0.79m밖에 못 내려가
    // rescue 컨트롤러의 정밀착륙 타임아웃에 걸려 AUTO.LAND 로 넘어갔다.
    // 카메라가 관성 안정화가 아니라 기체 고정이라 XY 보정으로 기울 때마다
    // 화면 속 타깃이 흔들려서 이 대역을 못 벗어난다 - 게이트를 그 실측 대역
    // 바깥으로 넓혀 정상 하강 구간에 포함시킨다.
    const double ALIGNED_RADIUS_M = 0.50;
    const double APPROACH_RADIUS_M = 1.00;
    const double APPROACH_DESCEND_SCALE = 0.5;

    // 카메라 대신 그리퍼를 타깃 위에 세우기 위한 목표 편향(미터, 지면 평면).
    double grasp_offset_err_y_m_ = 0.20;
    double grasp_offset_err_x_m_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<krac_interfaces::msg::TargetError>::SharedPtr vision_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    void vision_cb(const krac_interfaces::msg::TargetError::SharedPtr msg) {
        vision_err_ = *msg;
        if (msg->is_detected) {
            last_detection_time_ = this->now();
        }
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_alt_ = msg->pose.position.z;
        const auto& q = msg->pose.orientation;
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    void enable_cb(const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
        enabled_ = req->data;
        res->success = true;

        if (enabled_) {
            sum_x_ = 0.0; sum_y_ = 0.0;
            prev_err_x_ = 0.0; prev_err_y_ = 0.0;
            yaw_filter_initialized_ = false;
            has_been_aligned_ = false;
            hold_alt_m_ = current_alt_;
            RCLCPP_INFO(this->get_logger(), "✅ Lander 상태: ON (안정화 제어 시작, 기준 고도=%.2fm)", hold_alt_m_);
        } else {
            RCLCPP_INFO(this->get_logger(), "⛔ Lander 상태: OFF");
        }
    }

    void control_loop() {
        if (!enabled_) return;

        geometry_msgs::msg::Twist vel_cmd;

        // =======================================================
        // 1. 타겟이 보이지 않을 때의 예외 처리
        // =======================================================
        const bool target_fresh =
            vision_err_.is_detected &&
            last_detection_time_.nanoseconds() > 0 &&
            (this->now() - last_detection_time_).seconds() <= TARGET_FRESH_MAX_AGE_SEC;

        if (!target_fresh) {
            // 구조 박스에 한 번이라도 0.5m 이내로 정렬되어 실제 하강을 시작했다면,
            // 이후 YOLO/ArUco가 근접 시야에서 사라져도 상승 복구로 넘기지 않는다.
            // 마지막 정렬을 신뢰하고 XY/yaw를 고정한 채 지면까지 수직 하강한다.
            if (has_been_aligned_) {
                vel_cmd.linear.x = 0.0;
                vel_cmd.linear.y = 0.0;
                vel_cmd.linear.z =
                    current_alt_ < 0.35 ? -0.08 :
                    (current_alt_ < 0.70 ? BLIND_DESCENT_SPEED : -0.18);
                vel_cmd.angular.z = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[Target LOST after alignment, alt=%.2fm] continuing committed blind descent.",
                    current_alt_);
                vel_pub_->publish(vel_cmd);
                return;
            }

            // 아직 한 번도 충분히 정렬되지 않았다면 위험한 blind descent를 하지 않고
            // 현재 위치를 유지한다. BT timeout 후 해당 구조 시도 전체를 재시도한다.
            vel_cmd.linear.x = 0.0;
            vel_cmd.linear.y = 0.0;
            vel_cmd.linear.z = 0.0;
            vel_cmd.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Target LOST before alignment] holding position until rescue-attempt timeout.");
            vel_pub_->publish(vel_cmd);
            return;
        }

        // =======================================================
        // 🎯 2. 타겟 감지 시 정밀 정렬 (요동침 방지 적용)
        // =======================================================
        double alt_factor = std::max(current_alt_, 0.5);
        // 카메라 기준 생 오차. 이걸 0 으로 몰면 '카메라'가 타깃 위에 서고 그리퍼는
        // 0.2m 뒤에 남는다 - 그래서 목표를 오프셋만큼 옮겨 '그리퍼'를 타깃 위에 세운다.
        const double err_x_raw = vision_err_.pixel_err_x * (alt_factor / fx_);
        const double err_y_raw = vision_err_.pixel_err_y * (alt_factor / fy_);
        double err_x_m = err_x_raw - grasp_offset_err_x_m_;
        double err_y_m = err_y_raw - grasp_offset_err_y_m_;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[DBG] px=(%.1f,%.1f) err_cam=(%.3f,%.3f) err_grip=(%.3f,%.3f) alt=%.2f yaw=%.1fdeg",
            vision_err_.pixel_err_x, vision_err_.pixel_err_y,
            err_x_raw, err_y_raw, err_x_m, err_y_m,
            current_alt_, current_yaw_ * 180.0 / M_PI);

        // PID 연산
        sum_x_ = std::clamp(sum_x_ + err_x_m * 0.05, -1.0, 1.0);
        sum_y_ = std::clamp(sum_y_ + err_y_m * 0.05, -1.0, 1.0);

        double v_x = (KP_XY * err_x_m) + (KI_XY * sum_x_) + (KD_XY * (err_x_m - prev_err_x_) / 0.05);
        double v_y = (KP_XY * err_y_m) + (KI_XY * sum_y_) + (KD_XY * (err_y_m - prev_err_y_) / 0.05);

        prev_err_x_ = err_x_m;
        prev_err_y_ = err_y_m;

        // 데드밴드(Deadband): 오차가 10cm 이내면 X, Y 이동 명령을 무시하여 제자리 떨림 방지
        if (std::abs(err_x_m) < 0.08) v_x = 0.0;
        if (std::abs(err_y_m) < 0.08) v_y = 0.0;

        // 앞/뒤(X축) 방향 오류 수정을 위해 v_y 앞에 다시 마이너스(-)를 붙임.
        const double body_vx = std::clamp(-v_y, -MAX_XY_VEL, MAX_XY_VEL); // 앞뒤(기체 전방) 제어
        const double body_vy = std::clamp(v_x, -MAX_XY_VEL, MAX_XY_VEL);  // 좌우(기체 좌측) 제어

        // /precision_lander/cmd_vel은 그대로 mavros setpoint_velocity/cmd_vel_unstamped로
        // 전달되며, mavros는 이를 world ENU(local) 프레임 속도로 해석한다. 위 body_vx/body_vy는
        // 카메라(=기체) 프레임 오차에서 나온 값이라 기체 헤딩이 0(동쪽)이 아니면 실제 필요한
        // 방향과 다른 쪽으로 이동 명령이 나가서 정밀 하강이 타겟에 정확히 수렴하지 못한다.
        // 기체 현재 yaw로 body(FLU) -> world(ENU) 회전을 적용해서 보정한다.
        const double cos_yaw = std::cos(current_yaw_);
        const double sin_yaw = std::sin(current_yaw_);
        double lateral_scale = 0.75;
        if (current_alt_ < 3.0) lateral_scale = 0.35;
        if (current_alt_ < 1.5) lateral_scale = 0.12;
        if (current_alt_ < 0.7) lateral_scale = 0.05;
        vel_cmd.linear.x = lateral_scale * (body_vx * cos_yaw - body_vy * sin_yaw);
        vel_cmd.linear.y = lateral_scale * (body_vx * sin_yaw + body_vy * cos_yaw);

        // =======================================================
        // 🔄 3. OBB 기반 헤딩(Yaw) 0도 또는 180도 최단거리 정렬
        // =======================================================
        double target_yaw_err = vision_err_.yaw_err_rad;

        // 💡 [핵심 추가] 각도 오차를 [-90도, +90도] 구간으로 강제 매핑(래핑)합니다.
        // 이를 통해 드론이 180도를 빙 돌지 않고, 가장 가까운 직사각형 축으로 바로 맞춥니다.
        while (target_yaw_err > M_PI / 2.0) {
            target_yaw_err -= M_PI;
        }
        while (target_yaw_err < -M_PI / 2.0) {
            target_yaw_err += M_PI;
        }

        // OBB 각도 자체가 프레임마다 0도/90도 근처로 튀는 노이즈가 심해서, raw
        // target_yaw_err를 그대로 명령에 쓰면 실제 기체가 계속 스핀하면서 카메라
        // 시야에서 타겟이 벗어나는 문제가 생긴다(정밀 하강 발산의 주요 원인 중
        // 하나로 실측 확인됨). ±90도 경계에서 튀는 값도 최단거리로 보고 지수평활.
        if (!yaw_filter_initialized_) {
            filtered_yaw_err_ = target_yaw_err;
            yaw_filter_initialized_ = true;
        } else {
            double diff = target_yaw_err - filtered_yaw_err_;
            while (diff > M_PI / 2.0) diff -= M_PI;
            while (diff < -M_PI / 2.0) diff += M_PI;
            filtered_yaw_err_ += YAW_FILTER_ALPHA * diff;
            while (filtered_yaw_err_ > M_PI / 2.0) filtered_yaw_err_ -= M_PI;
            while (filtered_yaw_err_ < -M_PI / 2.0) filtered_yaw_err_ += M_PI;
        }

        // 실측 결과 OBB 각도를 그대로 따라가면(필터링해도) 하강 내내 기체가
        // 서서히 계속 회전하고, 그 회전 때문에 위 body->world 변환 기준이
        // 흔들려서 수평 정렬 오차가 점점 벌어지는 현상이 반복 확인됐다
        // (yaw 드리프트가 작았던 시도는 0.5m 이내까지 잘 내려갔고, 드리프트가
        // 컸던 시도는 오차가 계속 커지다 타겟을 놓쳤음). 그리퍼로 집는 데는
        // 기체 헤딩이 박스 각도와 맞을 필요가 없으므로(그리퍼 자체 회전
        // 조인트는 이 로직과 무관), 정밀 하강 중엔 헤딩을 아예 고정한다.
        // filtered_yaw_err_/yaw_deg는 디버그 로그 표시용으로만 계속 계산한다.
        vel_cmd.angular.z = 0.0;

        // =======================================================
        // ⬇️ 4. 오차 반경에 따른 강하 속도 조절
        // =======================================================
        double dist_m = std::hypot(err_x_m, err_y_m);
        double yaw_deg = filtered_yaw_err_ * (180.0 / M_PI); // 필터링된 각도 출력

        if (dist_m < ALIGNED_RADIUS_M) {
            last_aligned_time_ = this->now();
            has_been_aligned_ = true;
            vel_cmd.linear.z =
                current_alt_ < 0.35 ? -0.08 :
                (current_alt_ < 0.70 ? BLIND_DESCENT_SPEED : PRECISION_DESCEND_SPEED);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🎯 [Aligned] 정밀 하강 중 (오차: %.2fm, 각도: %.1f도, 고도: %.1fm)", dist_m, yaw_deg, current_alt_);
        } else if (dist_m < APPROACH_RADIUS_M) {
            vel_cmd.linear.z = PRECISION_DESCEND_SPEED * APPROACH_DESCEND_SCALE;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "↘️ [Approaching] 센터 진입 중... (오차: %.2fm, 각도: %.1f도, 고도: %.1fm)",
                dist_m, yaw_deg, current_alt_);
        } else {
            vel_cmd.linear.z = std::clamp(KP_ALT_HOLD * (hold_alt_m_ - current_alt_),
                                           -MAX_ALT_HOLD_VEL, MAX_ALT_HOLD_VEL);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🔄 [Aligning] 센터 맞추기 집중... (현재 오차: %.2fm, 고도유지: %.2f->%.2fm)",
                dist_m, current_alt_, hold_alt_m_);
        }

        vel_pub_->publish(vel_cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrecisionLander>());
    rclcpp::shutdown();
    return 0;
}
