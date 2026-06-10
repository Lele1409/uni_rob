#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <vector>

class OnlineLineFinder : public rclcpp::Node
{
public:
    OnlineLineFinder() : rclcpp::Node("online_line_finder")
    {
        // Folie 203: ε (local check) and ε(k) (global check), both in (0,1)
        // For the ICP bag: epsilon = epsilon_k = 0.98 (see w4/B/4.2/NOTES.md)
        this->declare_parameter("epsilon",         0.98);
        this->declare_parameter("epsilon_k",       0.98);
        this->declare_parameter("min_points",      3);
        this->declare_parameter("max_point_dist",  2.0);
        this->declare_parameter("scan_topic",      std::string("/scanout/scan"));

        auto scan_topic = this->get_parameter("scan_topic").as_string();
        // Match the bag's best_effort QoS so messages are not silently dropped
        auto qos = rclcpp::QoS(10).best_effort();

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lines", 10);
        scan_sub_   = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, qos,
            std::bind(&OnlineLineFinder::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "online_line_finder started | topic=%s  epsilon=%.3f  epsilon_k=%.3f  max_dist=%.2f",
            scan_topic.c_str(),
            this->get_parameter("epsilon").as_double(),
            this->get_parameter("epsilon_k").as_double(),
            this->get_parameter("max_point_dist").as_double());
    }

private:
    struct Pt { double x, y; };

    static double pdist(const Pt& a, const Pt& b)
    {
        double dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Convert LaserScan polar data to Cartesian points.
    // Invalid ranges are returned as {NaN, NaN} to preserve scan order — the
    // line finder uses them as mandatory break points so gaps in the scan
    // (openings, out-of-range beams) never produce lines through empty space.
    std::vector<Pt> toCartesian(const sensor_msgs::msg::LaserScan& msg) const
    {
        std::vector<Pt> pts;
        double angle = msg.angle_min;
        for (float r : msg.ranges) {
            if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max)
                pts.push_back({r * std::cos(angle), r * std::sin(angle)});
            else
                pts.push_back({std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()});
            angle += msg.angle_increment;
        }
        return pts;
    }

    // Online line finder — Folien 202-203.
    //
    // Conditions for extending line a_j...a_k with new point a_{k+1}:
    //   (1) Step distance  ||a_k, a_{k+1}|| <= max_point_dist
    //   (2) Local check    ||a_{k-1}, a_{k+1}|| / (||a_{k-1},a_k|| + ||a_k,a_{k+1}||) >= 1 - eps
    //   (3) Global check   ||a_j, a_{k+1}|| / sum_path >= 1 - eps_k
    //
    // A line is published only when it contains >= min_points points.
    // Each published line is represented by its first and last point.
    using Line = std::pair<Pt, Pt>;

    std::vector<Line> findLines(const std::vector<Pt>& pts)
    {
        const double eps   = this->get_parameter("epsilon").as_double();
        const double eps_k = this->get_parameter("epsilon_k").as_double();
        const int    min_n = this->get_parameter("min_points").as_int();
        const double dmax  = this->get_parameter("max_point_dist").as_double();

        std::vector<Line> lines;
        if ((int)pts.size() < min_n) return lines;

        std::vector<Pt> cur;
        double path_len = 0.0;

        auto finalize = [&]() {
            if ((int)cur.size() >= min_n)
                lines.emplace_back(cur.front(), cur.back());
        };

        // Skip leading NaN points
        size_t start = 0;
        while (start < pts.size() && std::isnan(pts[start].x)) ++start;
        if (start == pts.size()) return lines;
        cur.push_back(pts[start]);

        for (size_t k = start + 1; k < pts.size(); ++k) {
            const Pt& a_kp1 = pts[k];

            // Invalid beam → mandatory line break
            if (std::isnan(a_kp1.x)) {
                finalize();
                cur.clear();
                path_len = 0.0;
                // skip ahead until next valid point
                while (k + 1 < pts.size() && std::isnan(pts[k + 1].x)) ++k;
                if (k + 1 < pts.size()) cur.push_back(pts[k + 1]), ++k;
                continue;
            }

            const Pt& a_k   = cur.back();
            double    d_step = pdist(a_k, a_kp1);

            // (1) Gap too large → close current line and start fresh
            if (d_step > dmax) {
                finalize();
                cur.clear();
                cur.push_back(a_kp1);
                path_len = 0.0;
                continue;
            }

            bool ok = true;

            // (2) Local straightness check (needs at least 2 points in current line)
            if (cur.size() >= 2) {
                const Pt& a_km1  = cur[cur.size() - 2];
                double d_km1_kp1 = pdist(a_km1, a_kp1);
                double d_km1_k   = pdist(a_km1, a_k);

                // "keine Linien durchs Nichts": local jump must also be bounded
                if (d_km1_kp1 > dmax) {
                    ok = false;
                } else {
                    double denom = d_km1_k + d_step;
                    if (denom > 1e-9 && d_km1_kp1 / denom < eps)
                        ok = false;
                }
            }

            // (3) Global straightness check (Luftlinien-Argument)
            if (ok) {
                double new_path  = path_len + d_step;
                double d_global  = pdist(cur.front(), a_kp1);
                if (new_path > 1e-9 && d_global / new_path < eps_k) {
                    ok = false;
                } else {
                    path_len = new_path;
                }
            }

            if (!ok) {
                finalize();
                cur.clear();
                cur.push_back(a_kp1);
                path_len = 0.0;
            } else {
                cur.push_back(a_kp1);
            }
        }
        finalize();
        return lines;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto pts   = toCartesian(*msg);
        auto lines = findLines(pts);

        visualization_msgs::msg::Marker m;
        m.header               = msg->header;
        m.ns                   = "online_lines";
        m.id                   = 0;
        m.type                 = visualization_msgs::msg::Marker::LINE_LIST;
        m.action               = visualization_msgs::msg::Marker::ADD;
        m.scale.x              = 0.05;
        m.color.r              = 0.0f;
        m.color.g              = 1.0f;
        m.color.b              = 0.0f;
        m.color.a              = 1.0f;
        m.pose.orientation.w   = 1.0;

        for (auto& [start, end] : lines) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = start.x; p1.y = start.y;
            p2.x = end.x;   p2.y = end.y;
            m.points.push_back(p1);
            m.points.push_back(p2);
        }

        RCLCPP_INFO(this->get_logger(),
            "Scan processed: %zu input points → %zu lines", pts.size(), lines.size());
        marker_pub_->publish(m);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineLineFinder>());
    rclcpp::shutdown();
    return 0;
}
