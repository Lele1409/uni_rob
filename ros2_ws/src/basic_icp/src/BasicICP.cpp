#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <laser_geometry/laser_geometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <tf2/convert.h>

class BasicICPNode : public rclcpp::Node
{
public:
    BasicICPNode() : rclcpp::Node("basic_icp_node")
    {
        model_sub_.subscribe(this, "/model");
        scan_sub_.subscribe(this, "/scan");

        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), model_sub_, scan_sub_);
        syncApproximate_->registerCallback(&BasicICPNode::laserCallback, this);
    }

private:

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> model_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> syncApproximate_;

    laser_geometry::LaserProjection projector_;

    /// Pairs to store correspondences as Point32 structs with .x, .y, .z fields
    using CorrPair = std::pair<geometry_msgs::msg::Point32, geometry_msgs::msg::Point32>;

    /// Vector of corresponding pointd
    using CorrVec = std::vector<CorrPair>;

    /// @brief  Computes the correspondences between two point clouds using the 
    ///         closest point approximation
    ///
    /// @param scan_cloud               Data scan point cloud 
    /// @param model_cloud              Model scan point cloud
    /// @param point_correspondences    Vector with the resulting correspondences
    /// @param eps                      Summed-up squared distance of all found
    ///                                 correspondences
    void calcICPCorrespondences(sensor_msgs::msg::PointCloud2::SharedPtr scan_cloud,
                                sensor_msgs::msg::PointCloud2::SharedPtr model_cloud,
                                CorrVec &point_correspondences, double &eps)
    {
        /// TODO: Implement correspondence search
    }

    /**
     * @brief   Computes the transformation derived from given point 
     *          correspondences using the Lu-Milios formula from the 
     *          slides
     *   
     * @param point_correspondences     Vector of corresponding points 
     * @return tf2::Transform           Resulting transformation
     */
    tf2::Transform
    calcTransFormation(const CorrVec &point_correspondences)
    {
        tf2::Transform transform;
        return transform;
    }

    /**
     * @brief Transforms a point cloud according to the given transformation
     * 
     * @param in_cloud                  Untransformed point cloud
     * @param out_cloud                 Transformed point cloud
     * @param transform                 Transformation used to transform \ref in_cloud
     */
    void
    transformPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr in_cloud,
                        sensor_msgs::msg::PointCloud2::SharedPtr out_cloud,
                        tf2::Transform& transform)
    {
       
    }

    /// @brief Synchronized callback for both laser scans
    void laserCallback(sensor_msgs::msg::LaserScan::SharedPtr scan, sensor_msgs::msg::LaserScan::SharedPtr model)
    {
        // since we later on work with translation in x and y it makes sense to
        // convert both scans to PointClouds first
        sensor_msgs::msg::PointCloud2::SharedPtr tmp_cloud(new sensor_msgs::msg::PointCloud2);
        sensor_msgs::msg::PointCloud2::SharedPtr scan_cloud(new sensor_msgs::msg::PointCloud2);
        sensor_msgs::msg::PointCloud2::SharedPtr model_cloud(new sensor_msgs::msg::PointCloud2);

        // convert model and scan to point clouds
        projector_.projectLaser(*model, *model_cloud);
        projector_.projectLaser(*scan, *scan_cloud);

        /// TODO: Implement ICP with above helper functions
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicICPNode>());
    rclcpp::shutdown();
    return 0;
}
