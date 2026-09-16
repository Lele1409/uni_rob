#!/usr/bin/env python3
"""Minimal stand-in for the real Raupy robot, for testing the laptop stack offline.

It reproduces exactly the interfaces the real ROSbot 2 PRO exposes (see wf/plan.md, section 2),
so exploration.launch.py can run unchanged without hardware:

  * subscribes /cmd_vel as geometry_msgs/TwistStamped (plain Twist is ignored on the robot too)
    and stops after 0.5 s without commands, like the robot's cmd_vel_timeout
  * publishes /odometry/filtered and TF odom -> base_link (the robot's EKF output)
  * publishes static TF base_link -> laser with the real mount: xyz (0.02, 0, 0.131), yaw -90 deg
  * publishes /scan in frame `laser`: 360 deg, best-effort sensor QoS, 10 Hz, ray-cast in a
    built-in 2D floor plan (rooms, doors, a corridor)

This is a kinematic toy, not a physics simulator: no wheel slip, no collisions (the robot is
simply stopped at walls). It exists to check wiring, frames, QoS and the supervisor logic.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

RES = 0.05  # floor plan resolution [m/cell]


def build_world():
    """Return an occupancy array (True = wall) of a small office: 2 rooms + corridor."""
    w, h = int(14.0 / RES), int(9.0 / RES)
    g = np.zeros((h, w), dtype=bool)

    def wall(x0, y0, x1, y1):  # axis-aligned wall in metres, 10 cm thick
        xs, xe = sorted((int(x0 / RES), int(x1 / RES)))
        ys, ye = sorted((int(y0 / RES), int(y1 / RES)))
        g[ys:max(ye, ys + 2), xs:max(xe, xs + 2)] = True

    def door(x0, y0, x1, y1):  # cut an opening
        xs, xe = sorted((int(x0 / RES), int(x1 / RES)))
        ys, ye = sorted((int(y0 / RES), int(y1 / RES)))
        g[ys:max(ye, ys + 2), xs:max(xe, xs + 2)] = False

    wall(0, 0, 14, 0); wall(0, 9, 14, 9); wall(0, 0, 0, 9); wall(14, 0, 14, 9)  # outer
    wall(0, 3, 14, 3)                  # corridor (y 0..3) / rooms (y 3..9)
    wall(6, 3, 6, 9); wall(10, 3, 10, 9)
    door(1.5, 3, 2.5, 3); door(7.5, 3, 8.5, 3); door(11.5, 3, 12.5, 3)
    wall(3, 5.5, 4, 6.5)               # an obstacle (cupboard) in room 1
    return g


class FakeRaupy(Node):
    def __init__(self):
        super().__init__('fake_raupy')
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 1.5)
        self.declare_parameter('beams', 720)       # real A3 has 1800; fewer keeps CPU low
        self.declare_parameter('range_max', 12.0)
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.th = 0.0
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.world = build_world()
        self.beams = int(self.get_parameter('beams').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self._publish_laser_mount()
        self.dt = 0.05
        self.create_timer(self.dt, self.step)      # 20 Hz odometry, like the robot
        self.create_timer(0.1, self.publish_scan)  # 10 Hz scan
        self.get_logger().info('fake_raupy up: TwistStamped /cmd_vel -> /odometry/filtered, /scan')

    def _publish_laser_mount(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.translation.x, t.transform.translation.z = 0.02, 0.131
        t.transform.rotation.z = math.sin(-math.pi / 4)  # yaw -90 deg
        t.transform.rotation.w = math.cos(-math.pi / 4)
        self.static_tf.sendTransform(t)

    def on_cmd(self, msg):
        self.v = max(-1.0, min(1.0, msg.twist.linear.x))
        self.w = max(-3.14, min(3.14, msg.twist.angular.z))
        self.last_cmd = self.get_clock().now()

    def occupied(self, x, y):
        i, j = int(y / RES), int(x / RES)
        h, w = self.world.shape
        return not (0 <= i < h and 0 <= j < w) or self.world[i, j]

    def step(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds > 0.5e9:  # robot's cmd_vel_timeout
            self.v = self.w = 0.0
        th = self.th + self.w * self.dt
        nx = self.x + self.v * math.cos(th) * self.dt
        ny = self.y + self.v * math.sin(th) * self.dt
        # Crude collision: refuse to move the centre within 12 cm of a wall.
        if not any(self.occupied(nx + 0.12 * math.cos(a), ny + 0.12 * math.sin(a))
                   for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)):
            self.x, self.y = nx, ny
        self.th = math.atan2(math.sin(th), math.cos(th))

        q_z, q_w = math.sin(self.th / 2), math.cos(self.th / 2)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = q_z, q_w
        self.tf.sendTransform(t)

        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q_z, q_w
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)

    def publish_scan(self):
        # Laser pose in the world: base pose + mount offset, laser yaw = base yaw - 90 deg.
        lx = self.x + 0.02 * math.cos(self.th)
        ly = self.y + 0.02 * math.sin(self.th)
        lyaw = self.th - math.pi / 2
        angles = np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
        steps = np.arange(0.1, self.range_max, RES / 2)
        wa = lyaw + angles
        xs = lx + np.outer(np.cos(wa), steps)
        ys = ly + np.outer(np.sin(wa), steps)
        ii, jj = (ys / RES).astype(int), (xs / RES).astype(int)
        h, w = self.world.shape
        inside = (ii >= 0) & (ii < h) & (jj >= 0) & (jj < w)
        hit = ~inside
        hit[inside] = self.world[ii[inside], jj[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, steps[np.maximum(first, 0)], np.inf)
        ranges += np.random.normal(0.0, 0.01, ranges.shape)  # 1 cm noise

        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'laser'
        s.angle_min, s.angle_max = -math.pi, math.pi - 2 * math.pi / self.beams
        s.angle_increment = 2 * math.pi / self.beams
        s.scan_time, s.range_min, s.range_max = 0.1, 0.05, 25.0
        s.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(s)


def main():
    rclpy.init()
    node = FakeRaupy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
