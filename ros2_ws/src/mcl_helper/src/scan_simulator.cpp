#include "mcl_helper/scan_simulator.h"
#include "mcl_helper/util.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>



namespace mcl_helper
{
ScanSimulator::ScanSimulator()
{

}

ScanSimulator::ScanSimulator(const nav_msgs::msg::OccupancyGrid& map) 
{
  setMap(map);
}

const nav_msgs::msg::OccupancyGrid& ScanSimulator::getMap() const
{
  return m_map;
}

void ScanSimulator::setMap(const nav_msgs::msg::OccupancyGrid& map)
{
  m_map = map;
  m_worldToMap = getWorldToMapTF(map.info);
}

void ScanSimulator::simulateScan(const geometry_msgs::msg::Pose& pose, sensor_msgs::msg::LaserScan& scan) const
{
  auto current_angle = scan.angle_min;
  auto grid_origin = getGridPosition(pose.position);
  geometry_msgs::msg::Point target_point;

  //double error = 0.0;

  size_t Nranges = static_cast<size_t>((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
  scan.ranges.resize(Nranges);

  for(auto step = 0u; step < scan.ranges.size(); ++step)
  {
    target_point.x = scan.range_max * cos(current_angle);
    target_point.y = scan.range_max * sin(current_angle);

    auto simulated_range = simulateRay(grid_origin, getGridPosition(transformPoint(target_point, pose))); 

    scan.ranges[step] = simulated_range;

    current_angle += scan.angle_increment;
  }
}

double ScanSimulator::simulateRayLow(int x1, int y1, int x2, int y2, int increment) const
{
  int dx = abs(x2 - x1);
  int dy = y2 - y1;

  int y_increment = 1;

  if(dy < 0)
  {
    y_increment = -1;
    dy = -dy;
  }

  int error = 2 * dy - dx;
  int y_current = y1;

  for(int x_current = x1 * increment; x_current <= x2 * increment; x_current += increment * increment)
  {
    if(m_map.data[y_current * m_map.info.width + x_current * increment] > MAP_FREE)
    {
      return getWorldDistance(x1, y1, x_current * increment, y_current);
    }

    if(error > 0)
    {
      y_current += y_increment;
      error -= 2 * dx;
    }

    error += 2 * dy;
  }

  return 0.0;
}

double ScanSimulator::simulateRayHigh(int x1, int y1, int x2, int y2, int increment) const
{
  int dx = x2 - x1;
  int dy = abs(y2 - y1);

  int x_increment = 1;

  if(dx < 0)
  {
    x_increment = -1;
    dx = -dx;
  }

  int error = 2 * dx - dy;
  int x_current = x1;

  for(int y_current = y1 * increment; y_current <= y2 * increment; y_current += increment * increment)
  {
    if(m_map.data[y_current * increment * m_map.info.width + x_current] > MAP_FREE)
    {
      return getWorldDistance(x1, y1, x_current, y_current * increment);
    }

    if(error > 0)
    {
      x_current += x_increment;
      error -= 2 * dy;
    }

    error += 2 * dx;
  }

  return 0.0;
}

double ScanSimulator::simulateRay(const std::pair<int, int>& point1, const std::pair<int, int>& point2) const
{
  if(abs(point2.second - point1.second) < abs(point2.first - point1.first))
  {
    if(point1.first > point2.first)
    {
      return simulateRayLow(point1.first, point1.second, point2.first, point2.second, -1);
    }
    else
    {
      return simulateRayLow(point1.first, point1.second, point2.first, point2.second, 1);
    }
  }
  else
  {
    if(point1.second > point2.second)
    {
      return simulateRayHigh(point1.first, point1.second, point2.first, point2.second, -1);
    }
    else
    {
      return simulateRayHigh(point1.first, point1.second, point2.first, point2.second, 1);
    }
  }
}

double ScanSimulator::getWorldDistance(int x1, int y1, int x2, int y2) const
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)) * m_map.info.resolution;
}

inline std::pair<int, int> ScanSimulator::getGridPosition(const geometry_msgs::msg::Point& point) const
{
  std::pair<int, int> grid_position; 
  auto transformed_point = transformToMap(point);

  grid_position.first = convertResolution(transformed_point.x);
  grid_position.second = convertResolution(transformed_point.y);

  return grid_position;
}

geometry_msgs::msg::Point ScanSimulator::transformToMap(const geometry_msgs::msg::Point& point) const
{
  geometry_msgs::msg::Point result_point;
  tf2::doTransform(point, result_point, m_worldToMap);

  return result_point;
}

int ScanSimulator::convertResolution(double coordinate) const
{
  return coordinate / m_map.info.resolution;
}

} // namespace mcl