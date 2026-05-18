#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class TrajectoryRecorder : public rclcpp::Node
{
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out) :
        Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out)
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TrajectoryRecorder::odom_callback, this, _1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&TrajectoryRecorder::imu_callback, this, _1));
    }


private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_out_ << msg->pose.pose.position.x << " "
                  << msg->pose.pose.position.y << "\n";
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_out_ << msg->linear_acceleration.x << " "
                 << msg->linear_acceleration.y << " "
                 << msg->angular_velocity.z << "\n";
    }

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}