#include <memory>
#include <fstream>
#include <cmath>

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
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            10,
            std::bind(&TrajectoryRecorder::imu_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&TrajectoryRecorder::odom_callback, this, _1));
    }


private:

    void imu_callback(const sensor_msgs::msg::Imu& msg)
    {
        double t = 0.01;

        double vx = msg.linear_acceleration.x * t;
        double vy = msg.linear_acceleration.y * t;
        double vz = msg.linear_acceleration.z * t;

        double delta_theta = msg.angular_velocity.z * t;

        double v = sqrt(vx * vx + vy * vy + vz * vz);

        double x = last_x + v * sin(last_theta + delta_theta / 2);
        double y = last_y + v * cos(last_theta + delta_theta / 2);

        last_x = x;
        last_y = y;
        last_theta = last_theta + delta_theta;

        if(imu_out_.good()) {
            imu_out_ << x << " " << y << "\n";
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry& msg)
    {
        if(odom_out_.good()) {
            odom_out_ << msg.pose.pose.position.x << " "
                      << msg.pose.pose.position.y << "\n";
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    double last_theta = 0;
    double last_x = 0;
    double last_y = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}
