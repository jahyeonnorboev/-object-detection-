₩#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class HumanFollower : public rclcpp::Node
{
public:
  HumanFollower() : Node("human_follower")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&HumanFollower::scanCallback, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "ROS2 Human Follower started");
    RCLCPP_INFO(this->get_logger(), "LiDAR /scan 기준으로 정면 가까운 사람 후보를 따라갑니다.");
  }

private:
  const double FOLLOW_DISTANCE = 0.6;
  const double STOP_DISTANCE = 0.35;
  const double DETECT_RANGE = 1.8;
  const double FOV_HALF_DEG = 70.0;

  const double MAX_LINEAR_SPEED = 0.15;
  const double MAX_ANGULAR_SPEED = 0.8;

  const double KP_LINEAR = 0.4;
  const double KP_ANGULAR = 0.8;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist twist;
    cmd_pub_->publish(twist);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) {
      stopRobot();
      return;
    }

    double fov_half_rad = FOV_HALF_DEG * M_PI / 180.0;

    double best_dist = 999.0;
    double best_angle = 0.0;
    bool found = false;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      double r = msg->ranges[i];

      if (!std::isfinite(r)) {
        continue;
      }

      if (r < msg->range_min || r > DETECT_RANGE) {
        continue;
      }

      double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      angle = normalizeAngle(angle);

      if (std::abs(angle) > fov_half_rad) {
        continue;
      }

      if (r < best_dist) {
        best_dist = r;
        best_angle = angle;
        found = true;
      }
    }

    if (!found) {
      stopRobot();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "감지 없음 — 정지"
      );
      return;
    }

    geometry_msgs::msg::Twist twist;

    if (best_dist < STOP_DISTANCE) {
      twist.linear.x = -0.04;
      twist.angular.z = 0.0;
    } else {
      double error_dist = best_dist - FOLLOW_DISTANCE;

      double linear = KP_LINEAR * error_dist;
      double angular = KP_ANGULAR * best_angle;

      linear = std::clamp(linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
      angular = std::clamp(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

      twist.linear.x = linear;
      twist.angular.z = angular;
    }

    cmd_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "FOLLOWING dist=%.2fm angle=%.1fdeg v=%.2f w=%.2f",
      best_dist,
      best_angle * 180.0 / M_PI,
      twist.linear.x,
      twist.angular.z
    );
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
