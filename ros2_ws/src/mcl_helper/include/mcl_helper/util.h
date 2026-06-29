#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace mcl_helper
{

/**
 * @brief Extract the yaw (two dimensional rotation) from a given quaternion
 * 
 * @param quaternion Quaternion where the angle should be extracted
 * 
 * @return The yaw angle from the given quaternion 
 */
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);

/**
 * @brief Transform a given point based into a coordinate system represented by a pose
 * 
 * @param point Point which should be transformed
 * @param transform Representation of the target coordinate system
 * 
 * @return transformed point
 */
geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Pose& transform);

/**
 * @brief Builds a tf transform from a map representation to a global frame  
 * 
 * @param map_meta Information about the map representation
 * 
 * @return Transformation from the map representation to the global coordinate system 
 */
geometry_msgs::msg::TransformStamped getMapToWorldTF(const nav_msgs::msg::MapMetaData& map_meta);

/**
 * @brief Builds a tf transform from a global frame to a map representation   
 * 
 * @param map_meta Information about the map representation
 * 
 * @return Transformation from the global coordinate system to the map representation 
 */
geometry_msgs::msg::TransformStamped getWorldToMapTF(const nav_msgs::msg::MapMetaData& map_meta);

} // namespace mcl

#endif