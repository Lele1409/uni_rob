#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

static std::vector<float> median_filter(const std::vector<float>& ranges, int delta = 5)
{
    int half = delta / 2;
    int n = static_cast<int>(ranges.size());
    std::vector<float> out(n);
    for (int i = 0; i < n; ++i) {
        int lo = std::max(0, i - half);
        int hi = std::min(n - 1, i + half);
        std::vector<float> window(ranges.begin() + lo, ranges.begin() + hi + 1);
        std::sort(window.begin(), window.end());
        out[i] = window[window.size() / 2];
    }
    return out;
}

static void write_scan(std::ofstream& file, const sensor_msgs::msg::LaserScan& msg)
{
    auto filtered = median_filter(msg.ranges);
    for (size_t i = 0; i < filtered.size(); ++i) {
        float angle = msg.angle_min + i * msg.angle_increment;
        file << angle << " " << msg.ranges[i] << " " << filtered[i] << "\n";
    }
    file << "\n";
    file.flush();
}

class ScanFilterNode : public rclcpp::Node
{
public:
    ScanFilterNode(std::ofstream& scan_out, std::ofstream& model_out)
    : Node("scan_filter"), scan_out_(scan_out), model_out_(model_out)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanFilterNode::scan_callback, this, _1));
        model_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/model", 10, std::bind(&ScanFilterNode::model_callback, this, _1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan& msg)
    {
        if (scan_out_.good())
            write_scan(scan_out_, msg);
    }

    void model_callback(const sensor_msgs::msg::LaserScan& msg)
    {
        if (model_out_.good())
            write_scan(model_out_, msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr model_sub_;

    std::ofstream& scan_out_;
    std::ofstream& model_out_;
};

int main(int argc, char** argv)
{
    std::ofstream scan_out("scan_filtered.txt");
    std::ofstream model_out("model_filtered.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilterNode>(scan_out, model_out));
    rclcpp::shutdown();
}
