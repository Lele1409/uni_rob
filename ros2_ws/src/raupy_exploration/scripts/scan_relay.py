#!/usr/bin/env python3
"""Pass-through for LaserScan: scan_in -> scan_out, used when the box filter is disabled.

The whole laptop stack (slam_toolbox, costmaps, collision monitor) reads one topic,
/scan_clean. With use_scan_filter:=false this relay keeps that topic alive, so switching
the filter off never silently starves Nav2 of sensor data. Sensor-data (best effort) QoS
on both sides, matching the robot's lidar driver.

With frame_id set, it also rewrites header.frame_id. slam.launch.py uses that to correct
the lidar mounting: Raupy's URDF has base_link -> laser at yaw -90 deg, but the lidar is
really mounted at 180 deg (measured 2026-09-21 against a wall in front of the robot).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.pub = self.create_publisher(LaserScan, 'scan_out', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_in', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
