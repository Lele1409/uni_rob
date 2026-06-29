#ifndef SCAN_SIMULATOR
#define SCAN_SIMULATOR
#ifndef SCAN_SIMULATOR_H
#define SCAN_SIMULATOR_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mcl_helper
{

// Possible values of a grid in the map
constexpr int MAP_FREE = 0;
constexpr int MAP_UNKNOWN = -1;
constexpr int MAP_OCCUPIED = 100;

/**
 * @class ScanSimulator
 * 
 * @brief Provides functions for simulating a scan at a specific position in the map and evaluation 
 *        a set of particles using an observed scan
 */
class ScanSimulator
{
    /// Map where a scan should be simulated
    nav_msgs::msg::OccupancyGrid m_map;
    /// Transformation from the global ccordinate system to the map offset
    geometry_msgs::msg::TransformStamped m_worldToMap;

    /**
     * @brief Perform a bresenham algorithm for calculating the distance between the origin 
     *        and the first obstacle between the ray from origin to target point
     * 
     * @param origin Origin of the simulated ray
     * @param target End of the simulated ray
     * 
     * @return Euclidean distance between the origin and the first detected obstacle
     */
    double simulateRay(const std::pair<int, int>& origin, const std::pair<int, int>& target) const;

    /**
     * @brief Low case of bresenham
     */
    double simulateRayLow(int x1, int y1, int x2, int y2, int increment) const;

    /**
     * @brief High case of bresenham 
     */
    double simulateRayHigh(int x1, int y1, int x2, int y2, int increment) const;

    /**
     * @brief Calculates the euclidean distance between two points each represented by two coordinates 
     */
    double getWorldDistance(int x1, int y1, int x2, int y2) const;


    /**
     * @brief Calculates the equivalent grid position in the map for a given point
     * 
     * @param point Point in world resolution
     * 
     * @return Grid position of the given point
     */
    std::pair<int, int> getGridPosition(const geometry_msgs::msg::Point& point) const;

    /**
     * @brief Transform a given point from world to map grid coordinate system
     */
    geometry_msgs::msg::Point transformToMap(const geometry_msgs::msg::Point& point) const;

    /**
     * @brief Convert the given coordinate to map resolution
     */
    int convertResolution(double coordinate) const;

  public:
    /**
     * @brief No attribute initialization 
     */
    ScanSimulator();

    /**
     * @brief Initializes a map for simulating scans in it
     */
    ScanSimulator(const nav_msgs::msg::OccupancyGrid& map);

    /**
     * @brief Provides access to the used map
     */
    const nav_msgs::msg::OccupancyGrid& getMap() const;

    /**
     * @brief Sets the used map
     */
    void setMap(const nav_msgs::msg::OccupancyGrid& map);

    /**
     * @brief Simulates a scan in the used map at the given pose and stores it in the given scan
     * 
     * @param pose Pose on which a scabn should be simulated
     * @param scan Must contain about the scan, which should be simulated. 
     *        The range field will be overwritten by the calculated ranges 
     */
    void simulateScan(const geometry_msgs::msg::Pose& pose, sensor_msgs::msg::LaserScan& scan) const;
};

} // namespace mcl

//#include "scan_simulator.tcc"

#endif


#endif // SCAN_SIMULATOR
