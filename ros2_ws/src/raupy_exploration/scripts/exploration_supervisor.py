#!/usr/bin/env python3
"""Exploration supervisor for Raupy: decides when exploration ends, stops the robot, saves the map.

Stop conditions (see ``raupy_exploration.supervisor_logic.StopDecider``):
  * the frontier explorer reports completion (no frontiers left),
  * the known map area stagnates,
  * the time budget is used up.

Stop sequence, run exactly once:
  1. STOP the frontier explorer via its control service,
  2. cancel all Nav2 ``navigate_to_pose`` goals,
  3. publish zero TwistStamped on /cmd_vel for about one second,
  4. save the map with nav2 ``map_saver`` and serialize the slam_toolbox pose graph,
  5. write ``<base>_summary.yaml``,
  6. optionally shut the node down.

Threading: the node is spun by a MultiThreadedExecutor in the main thread. The stop sequence
runs in its own worker thread and waits on service futures with timeouts. That keeps the
sequence readable top to bottom, while the executor stays free to deliver the service
responses; a missing or hanging service only costs its timeout, it never blocks the stop.
"""

import os
import threading
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Empty
import yaml

from raupy_exploration.supervisor_logic import (
    build_summary,
    known_area_m2,
    map_base_path,
    StopDecider,
)

try:
    # The explorer is a separate package; if it is not built/sourced the supervisor must still
    # be able to stop the robot and save the map, so the import failure is handled at runtime.
    from frontier_exploration_ros2.srv import ControlExploration
except ImportError:
    ControlExploration = None


# Latched, reliable QoS: slam_toolbox publishes /map this way and the explorer publishes its
# completion event this way (frontier_explorer_node.cpp). Matching it means a supervisor that
# starts late still receives the last map and a completion that already happened.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ZERO_TWIST_RATE_HZ = 10.0
ZERO_TWIST_DURATION_S = 1.0


class ExplorationSupervisor(Node):

    def __init__(self):
        super().__init__('exploration_supervisor')

        # --- Parameters -------------------------------------------------------------------
        self.max_duration_s = self.declare_parameter('max_duration_s', 600.0).value
        self.stagnation_window_s = self.declare_parameter('stagnation_window_s', 90.0).value
        self.min_new_area_m2 = self.declare_parameter('min_new_area_m2', 0.5).value
        self.startup_grace_s = self.declare_parameter('startup_grace_s', 20.0).value
        check_period_s = self.declare_parameter('check_period_s', 1.0).value
        map_topic = self.declare_parameter('map_topic', '/map').value
        completion_topic = self.declare_parameter(
            'completion_topic', '/exploration_complete').value
        self.control_service = self.declare_parameter(
            'control_service', '/control_exploration').value
        self.navigate_action = self.declare_parameter(
            'navigate_action', '/navigate_to_pose').value
        self.map_saver_service = self.declare_parameter(
            'map_saver_service', '/map_saver/save_map').value
        self.serialize_service = self.declare_parameter(
            'serialize_service', '/slam_toolbox/serialize_map').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.map_name = self.declare_parameter('map_name', 'raupy_map').value
        self.map_output_dir = self.declare_parameter('map_output_dir', '').value
        self.shutdown_on_finish = self.declare_parameter('shutdown_on_finish', True).value
        # Upper bound for each single service wait/call in the stop sequence.
        self.service_timeout_s = self.declare_parameter('service_timeout_s', 10.0).value
        # map_saver needs the map topic name in its request; it subscribes itself.
        self.map_topic = map_topic

        self.decider = StopDecider(
            self.max_duration_s, self.stagnation_window_s,
            self.min_new_area_m2, self.startup_grace_s)

        # --- State (touched only from the monitor callback group) -------------------------
        self.start_time = None          # node clock time of the first map
        self.completion_received = False
        self.known_area = 0.0
        self.map_info = None            # nav_msgs/MapMetaData of the latest map
        self.stop_started = False
        # Set by the worker thread; main() uses it to leave the spin loop.
        self.finished = threading.Event()

        # Map, completion and the periodic check share one mutually exclusive group, so the
        # decision state above is never read and written concurrently.
        monitor_group = MutuallyExclusiveCallbackGroup()
        # Service responses must be processed while the worker thread waits on them, even if
        # a monitor callback is running, hence a separate reentrant group.
        client_group = ReentrantCallbackGroup()

        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, LATCHED_QOS,
            callback_group=monitor_group)
        self.create_subscription(
            Empty, completion_topic, self._on_completion, LATCHED_QOS,
            callback_group=monitor_group)
        self.create_timer(check_period_s, self._on_check, callback_group=monitor_group)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        self.control_client = None
        if ControlExploration is not None:
            self.control_client = self.create_client(
                ControlExploration, self.control_service, callback_group=client_group)
        else:
            self.get_logger().error(
                'frontier_exploration_ros2 is not importable (workspace not sourced?). '
                'The explorer cannot be paused on stop; the supervisor will still cancel '
                'Nav2 goals, stop the robot and save the map.')
        # rclpy's ActionClient can only cancel goals it sent itself. The explorer sends the
        # goals, so we talk to the action's cancel service directly instead.
        self.cancel_client = self.create_client(
            CancelGoal, self.navigate_action.rstrip('/') + '/_action/cancel_goal',
            callback_group=client_group)
        self.map_saver_client = self.create_client(
            SaveMap, self.map_saver_service, callback_group=client_group)
        self.serialize_client = self.create_client(
            SerializePoseGraph, self.serialize_service, callback_group=client_group)

        budget = f'{self.max_duration_s:.0f} s' if self.max_duration_s > 0 else 'unlimited'
        self.get_logger().info(
            f'Supervisor ready: time budget {budget}, stagnation < {self.min_new_area_m2} m^2 '
            f'in {self.stagnation_window_s:.0f} s (armed after {self.startup_grace_s:.0f} s). '
            f'Waiting for the first map on {map_topic} to start the clock.')

    # --- Monitoring --------------------------------------------------------------------------

    def _on_map(self, msg):
        if self.start_time is None:
            # The budget starts with the first map rather than node start, so a slow SLAM or
            # lidar start-up does not eat into the exploration time.
            self.start_time = self.get_clock().now()
            self.get_logger().info('First map received, exploration clock started.')
        # Only area and metadata are kept; holding the full grid would waste memory.
        self.known_area = known_area_m2(msg.data, msg.info.resolution)
        self.map_info = msg.info

    def _on_completion(self, _msg):
        if not self.completion_received:
            self.get_logger().info('Explorer reports: no frontiers left.')
        self.completion_received = True

    def _elapsed_s(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _on_check(self):
        if self.stop_started or self.start_time is None:
            return
        elapsed = self._elapsed_s()
        reason = self.decider.update(elapsed, self.known_area, self.completion_received)
        if reason is None:
            return

        self.stop_started = True
        self.get_logger().warn(
            f'STOP: reason={reason}, after {elapsed:.1f} s, '
            f'known area {self.known_area:.2f} m^2. Running stop sequence.')
        # Hand over to a worker thread; this callback must return so the executor can keep
        # processing the service responses the sequence waits for.
        threading.Thread(
            target=self._stop_sequence, args=(reason, elapsed), daemon=True).start()

    # --- Stop sequence (worker thread) -------------------------------------------------------

    def _stop_sequence(self, reason, elapsed):
        results = {}
        try:
            results['explorer_stop'] = self._stop_explorer()
            results['nav2_cancel'] = self._cancel_nav2_goals()
            self._publish_zero_velocity()

            base_path = map_base_path(self.map_output_dir, self.map_name)
            os.makedirs(os.path.dirname(base_path), exist_ok=True)
            results['map_saver'] = self._save_map(base_path)
            results['slam_serialize'] = self._serialize_pose_graph(base_path)

            self._write_summary(reason, elapsed, results, base_path)
        except Exception as exc:  # noqa: BLE001 - log anything, then still finish cleanly
            self.get_logger().error(f'Stop sequence failed: {exc!r}')
        finally:
            ok = [k for k, v in results.items() if v]
            failed = [k for k, v in results.items() if not v]
            self.get_logger().info(
                f'Stop sequence done (reason={reason}). Succeeded: {ok or "none"}; '
                f'failed: {failed or "none"}.')
            if self.shutdown_on_finish:
                self.get_logger().info('shutdown_on_finish is set, shutting down.')
                self.finished.set()
            else:
                self.get_logger().info('Staying idle (shutdown_on_finish is false).')

    def _call(self, client, request, what):
        """Call a service with bounded waits; return the response or None on any failure."""
        if not client.wait_for_service(timeout_sec=self.service_timeout_s):
            self.get_logger().error(
                f'{what}: service {client.service_name} not available after '
                f'{self.service_timeout_s:.0f} s, skipping.')
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self.service_timeout_s):
            # Abandon the request so a late response does not leak a pending future.
            client.remove_pending_request(future)
            self.get_logger().error(
                f'{what}: no response within {self.service_timeout_s:.0f} s, skipping.')
            return None
        if future.exception() is not None:
            self.get_logger().error(f'{what}: call raised {future.exception()!r}')
            return None
        return future.result()

    def _stop_explorer(self):
        if self.control_client is None:
            return False
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        # Keep the explorer node alive: the launch file owns its lifetime, and a quitting
        # node could be treated as a crash there.
        request.quit_after_stop = False
        response = self._call(self.control_client, request, 'Explorer STOP')
        if response is None:
            return False
        log = self.get_logger().info if response.accepted else self.get_logger().warn
        log(f'Explorer STOP: accepted={response.accepted}, message="{response.message}"')
        return bool(response.accepted)

    def _cancel_nav2_goals(self):
        # A default CancelGoal request has a zero goal id and a zero stamp, which the action
        # server interprets as "cancel all goals". This also catches goals the explorer's own
        # STOP does not cover (e.g. the return-to-start goal or goals sent from RViz).
        response = self._call(self.cancel_client, CancelGoal.Request(), 'Nav2 cancel')
        if response is None:
            return False
        # REJECTED with an empty list just means nothing was active, which is fine for us.
        self.get_logger().info(
            f'Nav2 cancel: return_code={response.return_code}, '
            f'{len(response.goals_canceling)} goal(s) canceling.')
        return response.return_code in (
            CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_REJECTED)

    def _publish_zero_velocity(self):
        # Nav2's controller or velocity smoother may still emit a last non-zero command right
        # after the cancel; a short stream of zeros overrides it. The robot also stops on its
        # own 0.5 s cmd_vel timeout, this just makes the stop immediate.
        count = int(ZERO_TWIST_RATE_HZ * ZERO_TWIST_DURATION_S)
        for _ in range(count):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Raupy's diff drive controller expects base_link (it has no base_footprint).
            msg.header.frame_id = 'base_link'
            self.cmd_vel_pub.publish(msg)
            # Wall-clock sleep on purpose: the stop must not depend on a running /clock.
            time.sleep(1.0 / ZERO_TWIST_RATE_HZ)
        self.get_logger().info(f'Published {count} zero velocity commands.')

    def _save_map(self, base_path):
        request = SaveMap.Request()
        request.map_topic = self.map_topic
        request.map_url = base_path  # map_saver appends .yaml / .pgm itself
        request.image_format = 'pgm'
        # trinary + 0.25/0.65 is the usual Nav2 map format that map_server can load back.
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        response = self._call(self.map_saver_client, request, 'map_saver')
        ok = response is not None and bool(response.result)
        if ok:
            self.get_logger().info(f'Map saved: {base_path}.yaml / .pgm')
        elif response is not None:
            self.get_logger().error('map_saver reported failure.')
        return ok

    def _serialize_pose_graph(self, base_path):
        # The serialized pose graph lets slam_toolbox continue or localize in this map later,
        # which the pgm/yaml pair alone cannot.
        request = SerializePoseGraph.Request()
        request.filename = base_path  # absolute, so slam_toolbox's cwd does not matter
        response = self._call(self.serialize_client, request, 'slam_toolbox serialize')
        ok = response is not None and response.result == SerializePoseGraph.Response.RESULT_SUCCESS
        if ok:
            self.get_logger().info(f'Pose graph serialized: {base_path}.posegraph / .data')
        elif response is not None:
            self.get_logger().error(f'slam_toolbox serialize failed (result={response.result}).')
        return ok

    def _write_summary(self, reason, elapsed, results, base_path):
        info = self.map_info
        summary = build_summary(
            reason=reason,
            duration_s=elapsed,
            known_area=self.known_area,
            resolution=info.resolution if info else 0.0,
            width=info.width if info else 0,
            height=info.height if info else 0,
            save_results=results,
            base_path=base_path,
        )
        path = base_path + '_summary.yaml'
        with open(path, 'w') as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(
            f'Summary written to {path}: reason={summary["reason"]}, '
            f'duration={summary["duration_s"]} s, known area={summary["known_area_m2"]} m^2')


def main():
    rclpy.init()
    node = ExplorationSupervisor()
    # Two threads suffice: one for monitor callbacks, one for service responses.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        # Spin in short slices so the loop notices when the stop sequence has finished.
        while rclpy.ok() and not node.finished.is_set():
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
