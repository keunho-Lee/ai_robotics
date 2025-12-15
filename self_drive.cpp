#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;
  
  enum State {
    FORWARD,
    OBSTACLE_HANDLE
  };
  State current_state_ = FORWARD;

  const float TARGET_SPEED = 0.15;
  const float STOP_DISTANCE = 0.30;
  const float TURN_SPEED = 0.5;
  const float KP_ANGULAR = 1.2;        

public:
  SelfDrive() : rclcpp::Node("self_drive")
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    lidar_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", lidar_qos_profile, 
        std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1));
    

    pose_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  }

private:
  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
 
    float front_dist = get_scan_min(scan, 0, 10);    
    float left_dist = get_scan_avg(scan, 45, 15);    
    float right_dist = get_scan_avg(scan, 315, 15);  


    geometry_msgs::msg::TwistStamped vel_msg;
    
   
    vel_msg.header.stamp = this->now();
    vel_msg.header.frame_id = "base_link"; 


    switch (current_state_)
    {
      case FORWARD:
        if (front_dist < STOP_DISTANCE) {
          current_state_ = OBSTACLE_HANDLE;
         
          vel_msg.twist.linear.x = 0.0;
          vel_msg.twist.angular.z = 0.0;
          RCLCPP_WARN(this->get_logger(), "Obstacle Detected! Stopping.");
        } 
        else {
          vel_msg.twist.linear.x = TARGET_SPEED;
          
          float error = left_dist - right_dist;
          error = std::max(-0.5f, std::min(0.5f, error));
          
          vel_msg.twist.angular.z = error * KP_ANGULAR;
        }
        break;

      case OBSTACLE_HANDLE:
        if (front_dist >= STOP_DISTANCE + 0.05) {
          current_state_ = FORWARD;
          vel_msg.twist.linear.x = 0.0;
          vel_msg.twist.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Path Clear! Moving Forward.");
        } 
        else {
          vel_msg.twist.linear.x = 0.0;

          if (left_dist > right_dist) {
            vel_msg.twist.angular.z = TURN_SPEED; 
          } else {
            vel_msg.twist.angular.z = -TURN_SPEED; 
          }
        }
        break;
    }

    pose_pub_->publish(vel_msg);
  }

  float get_scan_min(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
  {
    int size = scan->ranges.size();
    float min_dist = 100.0; 

    for (int i = -window; i <= window; i++)
    {
      int idx = (center_angle + i + size) % size;
      float r = scan->ranges[idx];

      if (std::isfinite(r) && r > 0.01) {
        if (r < min_dist) {
          min_dist = r;
        }
      }
    }
    
    if (min_dist >= 100.0) return 2.0; 
    return min_dist;
  }

  float get_scan_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
  {
    int size = scan->ranges.size();
    float sum = 0.0;
    int count = 0;

    for (int i = -window; i <= window; i++)
    {
      int idx = (center_angle + i + size) % size;
      float r = scan->ranges[idx];

      if (std::isfinite(r) && r > 0.01) {
        sum += r;
        count++;
      }
    }

    if (count == 0) return 1.0; 
    return sum / count;
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

