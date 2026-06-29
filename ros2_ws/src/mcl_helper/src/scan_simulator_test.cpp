#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/rclcpp.hpp>

#include <mcl_helper/scan_simulator.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/// @brief  Ros2 node to simulate 2D laser scans in grid using the "2D Goal Pose" 
///         feature in RVIz2
class ScanSimulatorNode : public rclcpp::Node
{
public:
    ScanSimulatorNode() : rclcpp::Node("scan_simulator_node")
    {
        // Create policiy for received maps
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

        // Subscribe to /map with QoS policy
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 
            map_qos, 
            std::bind(&ScanSimulatorNode::gridMapCallback, this, _1));

        // Subscribe to /goal_pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 
            10, std::bind(&ScanSimulatorNode::poseCallback, this, _1));

        // Create scan publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("simulated_scan", 1);

        // Create tf broadcaster
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:

    void gridMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        // Print message and store received map in scan simulator
        std::cout << "Got map" << std::endl;

        map_ = map;
        scan_simulator_.setMap(*map);
    }

    void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped)
    {
        std::cout << "Got pose" << std::endl;

        // TODO (1): Setup laser scan message meta data
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();
        scan.header.frame_id = "scan_simulator";
        scan.angle_min = -1.5708;
        scan.angle_max = +1.5708;
        scan.range_min = 0;
        scan.range_max = 10;
        scan.angle_increment = 0.0174553;

        // TODO (2): Simulate scan and publish
        scan_simulator_.simulateScan(pose_stamped->pose, scan);
        scan_pub_->publish(scan);

        // TODO (3): Setup transformation and publish via broadcaster
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "scan_simulator";
        transform.transform.translation.x = pose_stamped->pose.position.x;
        transform.transform.translation.y = pose_stamped->pose.position.y;
        transform.transform.translation.z = pose_stamped->pose.position.z;
        transform.transform.rotation = pose_stamped->pose.orientation;
        broadcaster_->sendTransform(transform);
    }

    /// @brief  Subscriber to the /map topic. Used to get the grid map from 
    ///         map server. It is put into the scan simulator
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    /// @brief  Subscriber to the /goal_pose topic published by RViz2. Used
    ///         to optain the pose of the simulated scan
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    /// @brief  Publisher for the simulated scan
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    /// @brief  Grid map of the environment
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;

    /// @brief  Our personal scan simulator
    mcl_helper::ScanSimulator scan_simulator_;

    /// @brief  Transform publisher. Used to send the transformation
    ///         between the map and the simulated scan, i.e., the 
    ///         clicked pose in RViz2
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

};

int main(int argc, char** argv)
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}