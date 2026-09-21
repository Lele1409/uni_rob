#!/usr/bin/env python3
"""Relay Raupy's low ToF range sensors into sensor_msgs/Range for Nav2.

The lidar sits 13 cm high, so anything lower (skirting boards, furniture feet, low
corners) is invisible to SLAM and the costmaps. The four ToF sensors at ~5.5 cm cover the
front and rear corners. The robot publishes them as single-beam LaserScans on
/range/<id>; this node republishes each one twice (see raupy_exploration.range_logic):

    /range/<id>  (LaserScan)  ->  /range_costmap/<id>  (Range, "nothing seen" = max_range)
                              ->  /range_monitor/<id>  (Range, "nothing seen" = +inf)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from raupy_exploration.range_logic import to_range_values


class RangeRelay(Node):
    def __init__(self):
        super().__init__('range_relay')
        sensors = self.declare_parameter('sensors', ['fl', 'fr', 'rl', 'rr']).value
        # 0 = use the sensor's own range_max. Lower it if the sensors report the floor.
        self.cutoff = self.declare_parameter('max_range_cutoff', 0.0).value
        # Reported cone = driver FOV (15 deg) * fov_scale. Narrower cones mark a smaller area,
        # so one close reading doesn't block the whole front of the robot.
        self.fov_scale = min(max(self.declare_parameter('fov_scale', 0.5).value, 0.05), 1.0)
        for sensor in sensors:
            costmap_pub = self.create_publisher(
                Range, f'/range_costmap/{sensor}', qos_profile_sensor_data)
            monitor_pub = self.create_publisher(
                Range, f'/range_monitor/{sensor}', qos_profile_sensor_data)
            self.create_subscription(
                LaserScan, f'/range/{sensor}',
                lambda msg, c=costmap_pub, m=monitor_pub: self._on_scan(msg, c, m),
                qos_profile_sensor_data)
        self.get_logger().info(
            f'Relaying range sensors {sensors} (cutoff {self.cutoff} m, fov x{self.fov_scale})')

    def _on_scan(self, scan, costmap_pub, monitor_pub):
        effective_max, costmap_range, monitor_range = to_range_values(
            scan.ranges, scan.range_min, scan.range_max, self.cutoff)
        out = Range()
        out.header = scan.header  # sensor frame, x axis = beam centre
        out.radiation_type = Range.INFRARED
        out.field_of_view = (scan.angle_max - scan.angle_min) * self.fov_scale
        out.min_range = scan.range_min
        out.max_range = effective_max
        out.range = costmap_range
        costmap_pub.publish(out)
        out.range = monitor_range
        monitor_pub.publish(out)


def main():
    rclpy.init()
    node = RangeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
