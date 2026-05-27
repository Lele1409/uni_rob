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

        global_x_ = 0;
        global_y_ = 0;
        global_z_ = 0;
        last_time_ = 0;
        first_ = true;
        first_odom_ = true;
    }


private:

    void imu_callback(const sensor_msgs::msg::Imu& msg) 
    {
        double t = 0.01;
        double vx = msg.linear_acceleration.x;
        double vy = msg.linear_acceleration.z;
        double vz = msg.linear_acceleration.x;

        double delta_theta = msg.angular_velocity.z * t;

        double v = sqrt(vx * vx + vy * vy + vz * vz);

        double x = last_x + v * t * sin(last_theta + delta_theta / 2 * t);
        last_x = x;
        double y  = last_y + v * t * cos(last_theta + delta_theta / 2 * t);
        last_y = y;
        double z = last_z + v * t * sin(last_theta + delta_theta / 2 * t);
        last_z = z;

        last_theta = last_theta + delta_theta;

        std::cout << v << "\n";

        if(imu_out_.good()) {
            imu_out_ << x << " " << y << "\n";
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry& msg) 
    {
            double x = msg.pose.pose.position.x;
            double y = msg.pose.pose.position.y;

            odom_out_ << x << " " << y << "\n";
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    double global_x_;
    double global_y_;
    double global_z_;

    double odom_offset_x_ = 0.0;
    double odom_offset_y_ = 0.0;
    double odom_offset_z_ = 0.0;

    double last_vel_x_ = 0.0;
    double last_vel_y_ = 0.0;
    double last_vel_z_ = 0.0;

    bool first_;
    bool first_odom_;

    double last_time_ = 0;

    double last_theta = 0;
    double last_x = 0;
    double last_y = 0;
    double last_z = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}
