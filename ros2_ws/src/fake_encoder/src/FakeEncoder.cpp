#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

struct VolksbotParameters
{
    double wheel_radius = 0.0985;
    double axis_length = 0.41;
    int64_t gear_ratio = 74;
    int64_t max_vel_l = 8250;
    int64_t max_vel_r = 8400;
    int64_t max_acc_l = 10000;
    int64_t max_acc_r = 10000;
    int64_t num_wheels = 4;
};


class FakeEncoder : public rclcpp::Node
{
  public:
    FakeEncoder()
    : Node("minimal_publisher"), x_(0), y_(0), theta_(0), rotation_l_(0), rotation_r_(0)
    {
      // Joint names of our URDF model
      joint_names_ = {
        "left_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_front_wheel_joint",
        "right_rear_wheel_joint"};

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        auto timer_callback =
          [this]() -> void {
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->now();
            message.name = joint_names_;
            rotation_l_ += (22.0 / 7.0) / 180.0;
            rotation_r_ += (22.0 / 7.0) / 180.0;
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.position[0]);
            if (rotation_l_ > 2 * (22 / 7)) {
              rotation_l_ -= 2 * (22 / 7);
            }
            if (rotation_r_ > 2 * (22 / 7)) {
              rotation_r_ -= 2 * (22 / 7);
            }
            message.position = {rotation_l_, rotation_l_, rotation_r_, rotation_r_};
            this->publisher_->publish(message);
        };

      timer_ = this->create_wall_timer(100ms, timer_callback);
    }

  private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    size_t count_;
    /// Current x position
    double x_;

    /// Current y position
    double y_;

    /// Current orientation
    double theta_;

    /// Rotation (in radians!) of the left wheel
    double rotation_l_;

    /// Rotation (in radians!) of the right wheel
    double rotation_r_;

    /// Robot parameters
    VolksbotParameters params_;

    /// Names of the joint names used in the 
    /// Volksbot URDF
    std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeEncoder>());
  rclcpp::shutdown();
  return 0;
}