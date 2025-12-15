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


  const float TARGET_SPEED = 0.15;     
  const float SAFE_DISTANCE = 0.35;   
  const float ROBOT_WIDTH = 0.25;   
  const float CORRIDOR_WIDTH = 0.4; 
  
 
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
   
    float front_dist = get_range_avg(scan, 0, 10);   
    float left_dist = get_range_avg(scan, 90, 20);    
    float right_dist = get_range_avg(scan, 270, 20); 


    geometry_msgs::msg::Twist vel = decide_movement(front_dist, left_dist, right_dist);

 
    RCLCPP_INFO(this->get_logger(), 
      "Step: %d | F: %.2f L: %.2f R: %.2f | Lin: %.2f Ang: %.2f", 
      step_, front_dist, left_dist, right_dist, vel.linear.x, vel.angular.z);
      
    pose_pub_->publish(vel);
    step_++;
  }

private:


  float get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
  {
    int size = scan->ranges.size();
    float sum = 0.0;
    int count = 0;

    for (int i = -window; i <= window; i++)
    {
    
      int idx = (center_angle + i + size) % size;
      float r = scan->ranges[idx];


      if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
        sum += r;
        count++;
      }
    }

    if (count == 0) return 2.0; 
    return sum / count;
  }

 
  geometry_msgs::msg::Twist decide_movement(float front, float left, float right)
  {
    geometry_msgs::msg::Twist vel;

  
    if (front < SAFE_DISTANCE)
    {
    
      vel.linear.x = 0.0; 
      
     
      if (left > right) {
        vel.angular.z = 0.5; 
      } else {
        vel.angular.z = -0.5; 
      }
    }
  
    else
    {
      vel.linear.x = TARGET_SPEED; // 0.15 m/s

     
      float error = left - right;
      
      
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
