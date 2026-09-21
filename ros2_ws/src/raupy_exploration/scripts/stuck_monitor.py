#!/usr/bin/env python3
"""Detect Raupy getting stuck, mark what it got stuck on, and get it free again.

Two triggers (logic in raupy_exploration.stuck_logic):

  STUCK        /cmd_vel commands linear motion for window_s, but /scan_clean has hardly
               changed (pushing against a chair leg, wheels slipping or blocked). Wheel
               odometry is not used, it is fooled by slipping. A virtual obstacle is placed
               just outside the footprint on the side it was driving towards, published in
               the map frame on /stuck_obstacles and kept by the costmaps' stuck_layer for the
               rest of the run, so Nav2 plans around it afterwards.
  NO PROGRESS  a navigate_to_pose goal is active but the SLAM pose hasn't moved or turned for
               no_progress_window_s. That's Nav2 considering the start pose itself in
               collision: controller, spin and back-up all refuse, and the robot just stands.

Either trigger starts the RESCUE, a non-blocking state machine run from the 10 Hz timer:
pause the explorer, cancel Nav2 goals, wait a moment, drive escape_distance away (backwards
unless it got stuck while reversing), stop, resume the explorer.

The escape is published straight on /cmd_vel. Jazzy's BackUp behaviour cannot switch off
its collision check, and that check is exactly what fails when the robot is already "in
collision"; the collision monitor would likewise block moving away from an obstacle inside
the footprint. Instead the escape is guarded here: the ToF sensors on that side and the lidar
points in the strip the robot would sweep must be clear, before and during the move, and it
is limited by distance (odometry) and time.

The explorer is only resumed if the supervisor hasn't ended the run (latched
/exploration_stopped). `ros2 service call /stuck_monitor/clear std_srvs/srv/Empty` removes
all virtual obstacles (e.g. after a false detection) and resets both costmaps, because the
stuck_layer never clears marks by itself.
"""

import math

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from nav2_msgs.srv import ClearEntireCostmap
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from raupy_exploration.stuck_logic import (
    corridor_blocked,
    NoProgressDetector,
    range_clear,
    StuckDetector,
    to_map_frame,
    virtual_obstacle_points,
)

try:
    from frontier_exploration_ros2.srv import ControlExploration
except ImportError:  # explorer not built: rescue still cancels Nav2 and escapes
    ControlExploration = None

TICK_HZ = 10.0
# A command older than this counts as "no command" (the robot's own cmd_vel_timeout is 0.5 s).
CMD_TIMEOUT_S = 0.5
# Height of the virtual points; stuck_layer accepts 0..0.5 m.
POINT_Z = 0.05
# Same as the supervisor's latched publisher.
LATCHED_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Rescue steps and how long each may take at most.
PAUSE, CANCEL, SETTLE, ESCAPE, STOP, RESUME = 'pause', 'cancel', 'settle', 'escape', 'stop', 'resume'
SERVICE_TIMEOUT_S = 5.0
STOP_S = 0.5


class StuckMonitor(Node):
    def __init__(self):
        super().__init__('stuck_monitor')
        p = self.declare_parameter
        self.detector = StuckDetector(
            window_s=p('window_s', 5.0).value,
            min_cmd_speed=p('min_cmd_speed', 0.05).value,
            min_cmd_fraction=p('min_cmd_fraction', 0.9).value,
            max_changed_fraction=p('max_changed_fraction', 0.10).value,
            abs_tol=p('change_abs_tol', 0.05).value,
            rel_tol=p('change_rel_tol', 0.02).value,
            cooldown_s=p('cooldown_s', 10.0).value)
        self.x_offset = p('obstacle_x_offset', 0.21).value
        self.half_width = p('obstacle_half_width', 0.15).value
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_link').value

        self.rescue_enabled = p('rescue_enabled', True).value
        self.no_progress = NoProgressDetector(
            window_s=p('no_progress_window_s', 20.0).value,
            min_dist=p('no_progress_min_dist', 0.10).value,
            min_yaw=p('no_progress_min_yaw', 0.35).value)
        self.escape_distance = p('escape_distance', 0.15).value
        self.escape_speed = p('escape_speed', 0.06).value
        self.escape_timeout_s = p('escape_timeout_s', 6.0).value
        self.escape_clearance = p('escape_clearance', 0.05).value
        self.settle_s = p('settle_s', 1.5).value
        self.rescue_cooldown_s = p('rescue_cooldown_s', 15.0).value
        # Chassis edges from base_link (measured) and where the ToF sensors sit (|x|).
        self.front_edge = p('front_edge', 0.10).value
        self.rear_edge = p('rear_edge', 0.12).value
        self.chassis_half_width = p('chassis_half_width', 0.14).value
        self.tof_x = p('tof_x', 0.093).value

        self.last_cmd_v = 0.0
        self.last_cmd_time = None
        self.obstacles = []  # (x, y) in map frame, kept for the whole run
        self.goal_active = False
        self.run_over = False
        self.tof = {}  # 'fl' -> list of ranges
        self.last_scan = None
        self.rescue = None  # dict while a rescue is running
        self.rescue_quiet_until = -math.inf

        navigate_action = p('navigate_action', '/navigate_to_pose').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(LaserScan, '/scan_clean', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, navigate_action + '/_action/status',
            self._on_nav_status, 10)
        self.create_subscription(
            EmptyMsg, '/exploration_stopped', self._on_run_over, LATCHED_QOS)
        for side in ('fl', 'fr', 'rl', 'rr'):
            self.create_subscription(
                LaserScan, f'/range/{side}',
                lambda msg, s=side: self.tof.__setitem__(s, list(msg.ranges)),
                qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/stuck_obstacles', 10)
        self.change_pub = self.create_publisher(Float32, '~/scan_change', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_service(Empty, '~/clear', self._on_clear)
        self.costmap_clearers = [
            self.create_client(ClearEntireCostmap, name) for name in (
                '/local_costmap/clear_entirely_local_costmap',
                '/global_costmap/clear_entirely_global_costmap')]
        self.control_client = (
            self.create_client(ControlExploration, p('control_service', '/control_exploration').value)
            if ControlExploration is not None else None)
        self.cancel_client = self.create_client(
            CancelGoal, navigate_action + '/_action/cancel_goal')
        self.create_timer(1.0 / TICK_HZ, self._on_tick)
        self.create_timer(0.5, self._publish_obstacles)
        self.get_logger().info(
            f'Stuck monitor: stuck = |v| >= {self.detector.min_cmd_speed} m/s for '
            f'{self.detector.window_s} s with <= {self.detector.max_changed_fraction:.0%} '
            f'of scan beams changed; no progress = goal active but < '
            f'{self.no_progress.min_dist} m / {self.no_progress.min_yaw} rad in '
            f'{self.no_progress.window_s} s; rescue {"on" if self.rescue_enabled else "off"}.')

    # --- inputs ---------------------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.last_cmd_v = msg.twist.linear.x
        self.last_cmd_time = self._now()

    def _on_scan(self, msg):
        self.last_scan = msg
        self.detector.add_scan(self._now(), list(msg.ranges))

    def _on_nav_status(self, msg):
        active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        self.goal_active = any(s.status in active for s in msg.status_list)

    def _on_run_over(self, _msg):
        if not self.run_over:
            self.get_logger().info('Supervisor ended the run: no more rescues.')
        self.run_over = True

    def _pose(self, frame):
        """(x, y, yaw) of base_link in `frame`, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException:
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    def _scan_points_base(self):
        """Latest /scan_clean as (x, y) points in base_link (empty if unavailable)."""
        scan = self.last_scan
        if scan is None:
            return []
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, Time())
        except TransformException:
            return []
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = tf.transform.translation.x + r[ok] * np.cos(a[ok] + yaw)
        y = tf.transform.translation.y + r[ok] * np.sin(a[ok] + yaw)
        return list(zip(x.tolist(), y.tolist()))

    # --- detection ------------------------------------------------------------------------

    def _on_tick(self):
        t = self._now()
        if self.rescue is not None:
            self._step_rescue(t)
            return

        fresh = self.last_cmd_time is not None and t - self.last_cmd_time < CMD_TIMEOUT_S
        self.detector.add_cmd(t, self.last_cmd_v if fresh else 0.0)
        direction = self.detector.check(t)
        if self.detector.last_change is not None:
            self.change_pub.publish(Float32(data=self.detector.last_change))
        if direction:
            self._on_stuck(t, direction)
            self._start_rescue(t, 'stuck', escape_dir=-direction, allow_other_side=False)
            return

        pose = self._pose(self.map_frame)
        if pose is not None:
            self.no_progress.add(t, *pose, self.goal_active)
            if self.no_progress.check(t):
                self.get_logger().warn(
                    f'NO PROGRESS: a Nav2 goal was active for {self.no_progress.window_s:.0f} s '
                    'but the robot neither moved nor turned (likely "in collision" for Nav2).')
                self._start_rescue(t, 'no progress', escape_dir=-1, allow_other_side=True)

    def _on_stuck(self, t, direction):
        change = self.detector.last_change
        self.detector.reset(t)  # cooldown, whether or not marking works
        pose = self._pose(self.map_frame)
        if pose is None:
            self.get_logger().error(f'STUCK detected but no {self.map_frame} pose.')
            return
        points = to_map_frame(
            virtual_obstacle_points(direction, self.x_offset, self.half_width), *pose)
        self.obstacles.extend(points)
        side = 'front' if direction > 0 else 'rear'
        self.get_logger().warn(
            f'STUCK: driving {"forward" if direction > 0 else "backward"} for '
            f'{self.detector.window_s:.0f} s but only {change:.0%} of the scan changed. '
            f'Marking a virtual obstacle at the {side} (robot at {pose[0]:.2f}, {pose[1]:.2f}); '
            f'{len(self.obstacles)} virtual points in total.')
        self._publish_obstacles()

    # --- rescue ---------------------------------------------------------------------------

    def _start_rescue(self, t, reason, escape_dir, allow_other_side):
        self.no_progress.reset(t)
        if not self.rescue_enabled or self.run_over or t < self.rescue_quiet_until:
            return
        self.get_logger().warn(f'RESCUE ({reason}): pausing exploration and backing off.')
        self.rescue = {
            'reason': reason, 'dir': escape_dir, 'other_side': allow_other_side,
            'state': None, 'until': 0.0, 'future': None, 'start': None}
        self._enter(t, PAUSE)

    def _enter(self, t, state):
        r = self.rescue
        r['state'], r['future'] = state, None
        if state == PAUSE:
            r['until'] = t + SERVICE_TIMEOUT_S
            if self.control_client is not None and self.control_client.service_is_ready():
                req = ControlExploration.Request()
                req.action = ControlExploration.Request.ACTION_STOP
                req.quit_after_stop = False
                r['future'] = self.control_client.call_async(req)
            else:
                r['until'] = t  # nothing to wait for
        elif state == CANCEL:
            r['until'] = t + SERVICE_TIMEOUT_S
            if self.cancel_client.service_is_ready():
                # Zero goal id + zero stamp = cancel all goals.
                r['future'] = self.cancel_client.call_async(CancelGoal.Request())
            else:
                r['until'] = t
        elif state == SETTLE:
            r['until'] = t + self.settle_s
        elif state == ESCAPE:
            r['until'] = t + self.escape_timeout_s
            r['start'] = self._pose(self.odom_frame)
        elif state == STOP:
            r['until'] = t + STOP_S

    def _step_rescue(self, t):
        r = self.rescue
        state = r['state']
        if self.run_over and state not in (STOP, RESUME):
            self.get_logger().info('Supervisor ended the run during the rescue; stopping it.')
            self._enter(t, STOP)
            return
        waiting = r['future'] is not None and not r['future'].done()
        if state in (PAUSE, CANCEL) and waiting and t < r['until']:
            return
        if state == PAUSE:
            self._enter(t, CANCEL)
        elif state == CANCEL:
            self._enter(t, SETTLE)
        elif state == SETTLE:
            if t < r['until']:
                return
            chosen = self._pick_escape_dir(r['dir'], r['other_side'])
            if chosen is None:
                self.get_logger().warn('RESCUE: no free side to back off to; only re-planning.')
                self._enter(t, STOP)
            else:
                r['dir'] = chosen
                self.get_logger().info(
                    f'RESCUE: driving {self.escape_distance:.2f} m '
                    f'{"backward" if chosen < 0 else "forward"}.')
                self._enter(t, ESCAPE)
        elif state == ESCAPE:
            self._step_escape(t)
        elif state == STOP:
            self._publish_speed(0.0)
            if t >= r['until']:
                self._enter(t, RESUME)
        elif state == RESUME:
            self._finish_rescue(t)

    def _escape_clear(self, direction, remaining):
        edge = self.front_edge if direction > 0 else self.rear_edge
        sensors = ('fl', 'fr') if direction > 0 else ('rl', 'rr')
        # ToF sensors sit (edge - tof_x) inside the chassis edge.
        needed = remaining + self.escape_clearance + (edge - self.tof_x)
        tof_ok = all(range_clear(self.tof.get(s, []), needed) for s in sensors)
        lidar_blocked = corridor_blocked(
            self._scan_points_base(), direction, edge,
            remaining + self.escape_clearance, self.chassis_half_width)
        return tof_ok and not lidar_blocked

    def _pick_escape_dir(self, preferred, allow_other_side):
        if self._escape_clear(preferred, self.escape_distance):
            return preferred
        if allow_other_side and self._escape_clear(-preferred, self.escape_distance):
            return -preferred
        return None

    def _step_escape(self, t):
        r = self.rescue
        now_pose = self._pose(self.odom_frame)
        moved = (math.hypot(now_pose[0] - r['start'][0], now_pose[1] - r['start'][1])
                 if now_pose is not None and r['start'] is not None else 0.0)
        remaining = self.escape_distance - moved
        if remaining <= 0.0:
            self.get_logger().info(f'RESCUE: backed off {moved:.2f} m.')
        elif t >= r['until']:
            self.get_logger().warn(f'RESCUE: escape timed out after {moved:.2f} m.')
        elif not self._escape_clear(r['dir'], remaining):
            self.get_logger().warn(f'RESCUE: something in the way, stopped after {moved:.2f} m.')
        else:
            self._publish_speed(r['dir'] * self.escape_speed)
            return
        self._enter(t, STOP)

    def _finish_rescue(self, t):
        if not self.run_over and self.control_client is not None \
                and self.control_client.service_is_ready():
            req = ControlExploration.Request()
            req.action = ControlExploration.Request.ACTION_START
            self.control_client.call_async(req)  # fire and forget
            self.get_logger().info('RESCUE done: exploration resumed.')
        else:
            self.get_logger().info('RESCUE done (exploration not resumed).')
        self.rescue = None
        self.rescue_quiet_until = t + self.rescue_cooldown_s
        self.detector.reset(t)  # our own escape commands must not count as "driving"
        self.no_progress.reset(t)

    def _publish_speed(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = v
        self.cmd_pub.publish(msg)

    # --- virtual obstacles ----------------------------------------------------------------

    def _publish_obstacles(self):
        # Published even when empty: an empty cloud keeps the costmap source "current".
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        cloud = point_cloud2.create_cloud_xyz32(
            header, [(x, y, POINT_Z) for x, y in self.obstacles])
        self.cloud_pub.publish(cloud)

    def _on_clear(self, request, response):
        self.get_logger().info(f'Clearing {len(self.obstacles)} virtual obstacle points.')
        self.obstacles.clear()
        self._publish_obstacles()
        # Fire and forget: the lidar and range layers refill within a second.
        for client in self.costmap_clearers:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(f'{client.srv_name} not available, costmap not reset.')
        return response


def main():
    rclpy.init()
    node = StuckMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # Ctrl+C can invalidate the context while a callback is being set up ("context is
        # not valid"); that's shutdown, not a failure. Anything while still running is real.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
