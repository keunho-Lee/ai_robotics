#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;

  // 상수 정의
  const float TARGET_SPEED = 0.15;     // 전진 속도 (m/s)
  const float SAFE_DISTANCE = 0.35;    // 전방 충돌 방지 거리 (m)
  const float ROBOT_WIDTH = 0.25;      // 로봇 지름 (m)
  const float CORRIDOR_WIDTH = 0.4;    // 길 폭 (m)
  
  // P제어 상수 (중앙 정렬을 위한 민감도)
  const float KP_ANGULAR = 1.5; 

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", lidar_qos_profile, callback);
        
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // 1. 센서 데이터 처리
    float front_dist = get_range_avg(scan, 0, 10);    // 전방 0도 기준 +-10도
    float left_dist = get_range_avg(scan, 90, 20);    // 좌측 90도 기준 +-20도
    float right_dist = get_range_avg(scan, 270, 20);  // 우측 270도 기준 +-20도

    // 2. 주행 명령 결정
    geometry_msgs::msg::Twist vel = decide_movement(front_dist, left_dist, right_dist);

    // 3. 로그 출력 및 명령 발행
    RCLCPP_INFO(this->get_logger(), 
      "Step: %d | F: %.2f L: %.2f R: %.2f | Lin: %.2f Ang: %.2f", 
      step_, front_dist, left_dist, right_dist, vel.linear.x, vel.angular.z);
      
    pose_pub_->publish(vel);
    step_++;
  }

private:
  // 특정 각도 범위의 평균 거리를 구하는 헬퍼 함수
  // center_angle: 중심 각도 (Degree), window: 양쪽 범위 (Degree)
  float get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
  {
    int size = scan->ranges.size();
    float sum = 0.0;
    int count = 0;

    for (int i = -window; i <= window; i++)
    {
      // 인덱스 wrap-around 처리 (0도 근처에서 음수 처리)
      int idx = (center_angle + i + size) % size;
      float r = scan->ranges[idx];

      // 유효하지 않은 데이터(inf, nan, 0) 필터링
      if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
        sum += r;
        count++;
      }
    }

    if (count == 0) return 2.0; // 데이터가 없으면 충분히 먼 것으로 간주
    return sum / count;
  }

  // 주행 알고리즘 핵심 로직
  geometry_msgs::msg::Twist decide_movement(float front, float left, float right)
  {
    geometry_msgs::msg::Twist vel;

    // 상황 1: 전방이 막혀있음 (코너 or 막다른 길)
    if (front < SAFE_DISTANCE)
    {
      // 제자리 회전 모드
      vel.linear.x = 0.0; 
      
      // 더 넓은 쪽으로 회전
      if (left > right) {
        vel.angular.z = 0.5; // 좌회전 (반시계)
      } else {
        vel.angular.z = -0.5; // 우회전 (시계)
      }
    }
    // 상황 2: 전방이 뚫려있음 (직진 + 중앙 정렬)
    else
    {
      vel.linear.x = TARGET_SPEED; // 0.15 m/s

      // 벽 사이 중앙 유지 알고리즘 (P 제어)
      // error가 양수면 왼쪽이 더 넓음 -> 왼쪽으로 붙어야 함 -> 좌회전(Positive)
      // 길 폭이 매우 좁으므로(여유공간 좌우 7.5cm) 민감하게 반응해야 함
      float error = left - right;
      
      // 너무 큰 에러는 튀는 값이므로 제한
      error = std::max(-1.0f, std::min(1.0f, error));

      vel.angular.z = error * KP_ANGULAR;
    }

    return vel;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
