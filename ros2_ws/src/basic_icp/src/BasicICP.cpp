#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <laser_geometry/laser_geometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <limits>
#include <vector>

class BasicICPNode : public rclcpp::Node
{
public:
    BasicICPNode() : rclcpp::Node("basic_icp_node"),
        total_tx_(0.0), total_ty_(0.0), total_theta_(0.0),
        iteration_(0), data_received_(false)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/icp_correspondences", 10);

        model_sub_.subscribe(this, "/model");
        scan_sub_.subscribe(this, "/scan");

        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
            approximate_policy(10), model_sub_, scan_sub_);
        syncApproximate_->registerCallback(&BasicICPNode::laserCallback, this);

        // ICP runs at 0.25 Hz (4 s per iteration) — as required
        icp_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4000),
            std::bind(&BasicICPNode::icpStep, this));
    }

private:
    // Correspondence pair: .first = model point P_i, .second = data/scan point P'_i
    // Follows the error function on slide 181: E = Σ ||P_i - (R_θ · P'_i + T)||²
    using CorrPair = std::pair<geometry_msgs::msg::Point32, geometry_msgs::msg::Point32>;
    using CorrVec  = std::vector<CorrPair>;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> model_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> approximate_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> syncApproximate_;

    laser_geometry::LaserProjection projector_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr icp_timer_;

    sensor_msgs::msg::PointCloud2::SharedPtr current_scan_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr model_cloud_;

    // Cumulative transformation represented as (tx, ty, θ) — slide 195
    double total_tx_, total_ty_, total_theta_;
    int    iteration_;
    bool   data_received_;

    /// @brief Closest Point Heuristic — slide 193.
    ///
    /// For each data point P'_i, find the nearest model point P_i.
    /// Points outside the ε-neighbourhood have no correspondence and are skipped.
    /// eps returns the RMSE of all matched pair distances.
    void calcICPCorrespondences(sensor_msgs::msg::PointCloud2::SharedPtr scan_cloud,
                                sensor_msgs::msg::PointCloud2::SharedPtr model_cloud,
                                CorrVec &point_correspondences, double &eps)
    {
        point_correspondences.clear();
        eps = 0.0;

        // Collect valid model points P_i into a flat list for O(n·m) search
        std::vector<std::pair<float, float>> model_pts;
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*model_cloud, "x");
             it != it.end(); ++it)
        {
            // it[0]=x, it[1]=y — laser_geometry packs xyz as consecutive float32s
            if (!std::isnan(it[0]) && !std::isnan(it[1]))
                model_pts.emplace_back(it[0], it[1]);
        }
        if (model_pts.empty()) return;

        for (sensor_msgs::PointCloud2ConstIterator<float> it(*scan_cloud, "x");
             it != it.end(); ++it)
        {
            if (std::isnan(it[0]) || std::isnan(it[1])) continue;

            const float sx = it[0], sy = it[1];
            float min_d2 = std::numeric_limits<float>::max();
            float bx = 0.f, by = 0.f;

            for (auto& [mx, my] : model_pts)
            {
                const float dx = sx - mx, dy = sy - my;
                const float d2 = dx*dx + dy*dy;
                if (d2 < min_d2) { min_d2 = d2; bx = mx; by = my; }
            }

            // Store as (model P_i, data P'_i) — convention matches slide 182
            geometry_msgs::msg::Point32 mp, sp;
            mp.x = bx; mp.y = by; mp.z = 0.f;
            sp.x = sx; sp.y = sy; sp.z = 0.f;
            point_correspondences.emplace_back(mp, sp);
            eps += static_cast<double>(min_d2);
        }

        if (!point_correspondences.empty())
            eps = std::sqrt(eps / static_cast<double>(point_correspondences.size()));
    }

    /// @brief Lu/Milios closed-form minimiser — slide 182.
    ///
    /// Slide notation uses (x, z) for the 2D ground plane; we substitute z → y for ROS.
    /// P_i = model (corr.first), P'_i = data/scan (corr.second).
    ///
    /// Returns the step transform ΔT as a tf2::Transform.
    tf2::Transform calcTransFormation(const CorrVec &point_correspondences)
    {
        const int N = static_cast<int>(point_correspondences.size());

        // Centroids c (model) and c' (data) — slide 182, eq. (5.15)
        double cx = 0, cy = 0, cxp = 0, cyp = 0;
        for (auto& [p, pp] : point_correspondences)
        {
            cx  += p.x;  cy  += p.y;
            cxp += pp.x; cyp += pp.y;
        }
        cx /= N; cy /= N; cxp /= N; cyp /= N;

        // Cross-covariance terms — slide 182
        // Using the slides' naming with z→y substitution:
        //   Sxx ≙ S_xx'  Sxy ≙ S_xz'  Syx ≙ S_zx'  Syy ≙ S_zz'
        double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
        for (auto& [p, pp] : point_correspondences)
        {
            const double pbx = p.x  - cx,  pby = p.y  - cy;   // p̄_i  (model centred)
            const double ppx = pp.x - cxp, ppy = pp.y - cyp;  // p̄'_i (data centred)
            Sxx += pbx * ppx;
            Sxy += pbx * ppy;
            Syx += pby * ppx;
            Syy += pby * ppy;
        }

        // Optimal rotation — slide 182 (top formula) / eq. (5.22)
        // θ = arctan2(S_yx' - S_xy',  S_xx' + S_yy')
        const double dtheta = std::atan2(Syx - Sxy, Sxx + Syy);

        // Optimal translation t = c - R(θ) · c' — slide 182 / slide 183
        const double ct = std::cos(dtheta), st = std::sin(dtheta);
        const double dtx = cx  - (cxp * ct - cyp * st);
        const double dty = cy  - (cxp * st + cyp * ct);

        tf2::Transform result;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, dtheta);
        result.setRotation(q);
        result.setOrigin(tf2::Vector3(dtx, dty, 0.0));
        return result;
    }

    /// @brief Apply a 2D rigid transform to every valid point in in_cloud → out_cloud.
    void transformPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr in_cloud,
                             sensor_msgs::msg::PointCloud2::SharedPtr out_cloud,
                             tf2::Transform &transform)
    {
        *out_cloud = *in_cloud;

        const double dtx = transform.getOrigin().x();
        const double dty = transform.getOrigin().y();
        double roll, pitch, dtheta;
        tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, dtheta);
        const double ct = std::cos(dtheta), st = std::sin(dtheta);

        sensor_msgs::PointCloud2ConstIterator<float> in_x(*in_cloud,  "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*in_cloud,  "y");
        sensor_msgs::PointCloud2Iterator<float>      out_x(*out_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float>      out_y(*out_cloud, "y");

        for (; in_x != in_x.end(); ++in_x, ++in_y, ++out_x, ++out_y)
        {
            if (std::isnan(*in_x) || std::isnan(*in_y)) continue;
            *out_x = static_cast<float>(ct * (*in_x) - st * (*in_y) + dtx);
            *out_y = static_cast<float>(st * (*in_x) + ct * (*in_y) + dty);
        }
    }

    /// Publish correspondences as a LINE_LIST marker in the "model" frame.
    void publishCorrespondences(const CorrVec &correspondences)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "model";
        marker.header.stamp    = this->now();
        marker.ns   = "icp_correspondences";
        marker.id   = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = 1.0f; marker.color.g = 0.5f;
        marker.color.b = 0.0f; marker.color.a = 1.0f;
        marker.pose.orientation.w = 1.0;

        for (auto& [p, pp] : correspondences)
        {
            // LINE_LIST: each pair of consecutive points draws one line
            geometry_msgs::msg::Point sp, mp;
            sp.x = pp.x; sp.y = pp.y; sp.z = 0.0;   // scan/data point  P'_i
            mp.x = p.x;  mp.y = p.y;  mp.z = 0.0;   // model point      P_i
            marker.points.push_back(sp);
            marker.points.push_back(mp);
        }
        marker_pub_->publish(marker);
    }

    /// Publish the current cumulative transform T = (total_tx_, total_ty_, total_theta_)
    /// as the tf transform model → laser.
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = this->now();
        ts.header.frame_id = "model";
        ts.child_frame_id  = "laser";
        ts.transform.translation.x = total_tx_;
        ts.transform.translation.y = total_ty_;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, total_theta_);
        ts.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(ts);
    }

    /// One ICP iteration at 0.25 Hz — full algorithm from slide 195.
    void icpStep()
    {
        if (!data_received_) return;

        // Slide 195, line 8: find correspondences with current scan estimate
        CorrVec correspondences;
        double  eps;
        calcICPCorrespondences(current_scan_cloud_, model_cloud_, correspondences, eps);
        if (correspondences.empty()) {
            RCLCPP_WARN(this->get_logger(), "No correspondences found");
            return;
        }

        publishCorrespondences(correspondences);

        // Slide 195, line 9: compute ΔT via Lu/Milios (slide 182)
        tf2::Transform step = calcTransFormation(correspondences);

        // Slide 195, line 10: T = T + R(θ_current) · ΔT  (SE(2) right-composition)
        const double dtx = step.getOrigin().x();
        const double dty = step.getOrigin().y();
        double roll, pitch, dtheta;
        tf2::Matrix3x3(step.getRotation()).getRPY(roll, pitch, dtheta);

        const double ct = std::cos(total_theta_), st = std::sin(total_theta_);
        total_tx_    += ct * dtx - st * dty;
        total_ty_    += st * dtx + ct * dty;
        total_theta_ += dtheta;

        // Slide 195, line 11: transform D with ΔT for next iteration
        sensor_msgs::msg::PointCloud2::SharedPtr tmp(new sensor_msgs::msg::PointCloud2);
        transformPointCloud(current_scan_cloud_, tmp, step);
        current_scan_cloud_ = tmp;

        RCLCPP_INFO(this->get_logger(),
            "Iteration %d | tx=%.4f  ty=%.4f  theta=%.4f deg | RMSE=%.4f",
            iteration_,
            total_tx_, total_ty_, total_theta_ * 180.0 / M_PI,
            eps);

        // Publish updated tf at the end of every iteration
        publishTransform();
        ++iteration_;
    }

    /// Synchronized callback: receives one model scan and one data scan.
    /// model_sub_ is first in the synchronizer, so model_msg arrives as the first argument.
    void laserCallback(sensor_msgs::msg::LaserScan::SharedPtr model_msg,
                       sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (data_received_) return;  // use only the first synchronised pair

        sensor_msgs::msg::PointCloud2::SharedPtr m_cloud(new sensor_msgs::msg::PointCloud2);
        sensor_msgs::msg::PointCloud2::SharedPtr s_cloud(new sensor_msgs::msg::PointCloud2);
        projector_.projectLaser(*model_msg, *m_cloud);
        projector_.projectLaser(*scan_msg,  *s_cloud);

        model_cloud_        = m_cloud;
        current_scan_cloud_ = s_cloud;
        total_tx_ = 0.0; total_ty_ = 0.0; total_theta_ = 0.0;
        iteration_    = 0;
        data_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Scans received — starting ICP at 0.25 Hz");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicICPNode>());
    rclcpp::shutdown();
    return 0;
}
