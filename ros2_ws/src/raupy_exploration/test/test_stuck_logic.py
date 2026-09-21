"""Unit tests for the ROS-free stuck detection logic (no ROS graph needed)."""

import math
import os
import sys

import numpy as np

try:
    from raupy_exploration import stuck_logic as logic
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from raupy_exploration import stuck_logic as logic

RNG = np.random.default_rng(0)
ROOM = 1.0 + 3.0 * np.abs(np.sin(np.linspace(0, 6, 1800)))  # 1-4 m, "a room"


def noisy(scan, sigma=0.01):
    return scan + RNG.normal(0.0, sigma, scan.shape)


def feed(det, v, scan_at, t_end=6.0, dt=0.1):
    """Tick at 10 Hz from t=0 to t_end with command v and scan_at(t)."""
    result = 0
    t = 0.0
    while t <= t_end + 1e-9:
        det.add_cmd(t, v)
        det.add_scan(t, scan_at(t))
        result = det.check(t)
        if result:
            return result, t
        t += dt
    return result, t


# --- scan_change_fraction ---------------------------------------------------------------

def test_identical_scans_do_not_change():
    assert logic.scan_change_fraction(ROOM, ROOM) == 0.0


def test_sensor_noise_is_below_threshold():
    assert logic.scan_change_fraction(noisy(ROOM), noisy(ROOM)) < 0.02


def test_moving_changes_most_beams():
    assert logic.scan_change_fraction(ROOM, ROOM - 0.25) > 0.9


def test_invalid_beams_are_ignored():
    a = ROOM.copy()
    b = ROOM.copy()
    a[:100] = np.inf
    b[100:200] = np.nan
    b[200:300] = 0.0
    assert logic.scan_change_fraction(a, b) == 0.0


def test_uncomparable_scans_return_none():
    assert logic.scan_change_fraction(ROOM, ROOM[:-1]) is None
    assert logic.scan_change_fraction(np.full(1800, np.inf), ROOM) is None


# --- StuckDetector ----------------------------------------------------------------------

def test_commanded_but_scan_static_is_stuck_forward():
    det = logic.StuckDetector(window_s=5.0)
    result, t = feed(det, 0.2, lambda t: noisy(ROOM))
    assert result == 1
    assert 5.0 <= t <= 5.6


def test_stuck_while_reversing_reports_rear():
    det = logic.StuckDetector(window_s=5.0)
    assert feed(det, -0.1, lambda t: noisy(ROOM))[0] == -1


def test_driving_and_scan_changing_is_not_stuck():
    det = logic.StuckDetector(window_s=5.0)
    # 0.2 m/s towards everything: ranges shrink by 0.2 m per second
    assert feed(det, 0.2, lambda t: noisy(ROOM - 0.2 * t), t_end=10.0)[0] == 0


def test_standing_still_without_command_is_not_stuck():
    det = logic.StuckDetector(window_s=5.0)
    assert feed(det, 0.0, lambda t: noisy(ROOM), t_end=10.0)[0] == 0


def test_slow_commands_below_threshold_are_ignored():
    # e.g. the collision monitor slowing the robot to a crawl in front of an obstacle
    det = logic.StuckDetector(window_s=5.0, min_cmd_speed=0.05)
    assert feed(det, 0.03, lambda t: noisy(ROOM), t_end=10.0)[0] == 0


def test_mostly_stopped_window_is_not_stuck():
    det = logic.StuckDetector(window_s=5.0, min_cmd_fraction=0.9)
    # commands on only half the time
    t = 0.0
    while t <= 10.0:
        det.add_cmd(t, 0.2 if int(t) % 2 == 0 else 0.0)
        det.add_scan(t, noisy(ROOM))
        assert det.check(t) == 0
        t += 0.1


def test_needs_a_full_window_of_history():
    det = logic.StuckDetector(window_s=5.0)
    assert feed(det, 0.2, lambda t: noisy(ROOM), t_end=4.5)[0] == 0


def test_stale_lidar_is_not_stuck():
    det = logic.StuckDetector(window_s=5.0)
    t = 0.0
    while t <= 8.0:
        det.add_cmd(t, 0.2)
        if t < 5.2:  # lidar stops publishing
            det.add_scan(t, noisy(ROOM))
        if t > 6.5:
            assert det.check(t) == 0
        else:
            det.check(t)
        t += 0.1


def test_cooldown_after_reset():
    det = logic.StuckDetector(window_s=5.0, cooldown_s=10.0)
    result, t = feed(det, 0.2, lambda t: noisy(ROOM))
    assert result == 1
    det.reset(t)
    # right after the reset nothing is reported, even though still stuck
    t2 = t
    while t2 < t + 9.9:
        t2 += 0.1
        det.add_cmd(t2, 0.2)
        det.add_scan(t2, noisy(ROOM))
        assert det.check(t2) == 0


# --- virtual obstacle geometry -----------------------------------------------------------

def test_virtual_obstacle_front_and_rear():
    front = logic.virtual_obstacle_points(1, x_offset=0.21, half_width=0.15, spacing=0.05)
    rear = logic.virtual_obstacle_points(-1, x_offset=0.21, half_width=0.15, spacing=0.05)
    assert len(front) == 7
    assert all(x == 0.21 for x, _ in front)
    assert all(x == -0.21 for x, _ in rear)
    assert front[0][1] == -0.15 and math.isclose(front[-1][1], 0.15)


def test_to_map_frame_rotates_and_translates():
    (x, y), = logic.to_map_frame([(1.0, 0.0)], 2.0, 3.0, math.pi / 2)
    assert math.isclose(x, 2.0, abs_tol=1e-9) and math.isclose(y, 4.0)


# --- NoProgressDetector --------------------------------------------------------------------

def feed_poses(det, pose_at, active=True, t_end=25.0, dt=0.5):
    t = 0.0
    while t <= t_end + 1e-9:
        det.add(t, *pose_at(t), active)
        if det.check(t):
            return t
        t += dt
    return None


def test_goal_active_and_standing_still_is_no_progress():
    det = logic.NoProgressDetector(window_s=20.0)
    t = feed_poses(det, lambda t: (1.0, 2.0, 0.3))
    assert t is not None and 19.5 <= t <= 21.0  # 0.5 s tolerance at the window start


def test_driving_is_progress():
    det = logic.NoProgressDetector(window_s=20.0)
    assert feed_poses(det, lambda t: (1.0 + 0.05 * t, 2.0, 0.0), t_end=40.0) is None


def test_turning_in_place_is_progress():
    det = logic.NoProgressDetector(window_s=20.0, min_yaw=0.35)
    assert feed_poses(det, lambda t: (1.0, 2.0, 0.1 * t), t_end=40.0) is None


def test_no_goal_means_no_alarm():
    det = logic.NoProgressDetector(window_s=20.0)
    assert feed_poses(det, lambda t: (1.0, 2.0, 0.0), active=False, t_end=40.0) is None


def test_small_jitter_is_still_no_progress():
    det = logic.NoProgressDetector(window_s=20.0, min_dist=0.10)
    assert feed_poses(det, lambda t: (1.0 + 0.02 * math.sin(t), 2.0, 0.05 * math.sin(t))) is not None


def test_yaw_wraparound_is_not_a_turn():
    det = logic.NoProgressDetector(window_s=20.0)
    yaw = lambda t: math.pi - 0.01 if int(t) % 2 else -math.pi + 0.01
    assert feed_poses(det, lambda t: (1.0, 2.0, yaw(t))) is not None


def test_reset_keeps_quiet():
    det = logic.NoProgressDetector(window_s=20.0)
    det.reset(0.0, quiet_s=100.0)
    assert feed_poses(det, lambda t: (1.0, 2.0, 0.0), t_end=40.0) is None


# --- escape guards -------------------------------------------------------------------------

def test_range_clear():
    assert logic.range_clear([math.nan], 0.2)
    assert logic.range_clear([], 0.2)
    assert logic.range_clear([0.5], 0.2)
    assert not logic.range_clear([0.1], 0.2)


def test_corridor_blocked_behind_and_ahead():
    behind = [(-0.25, 0.0)]
    assert logic.corridor_blocked(behind, -1, edge=0.12, length=0.2, half_width=0.14)
    assert not logic.corridor_blocked(behind, +1, edge=0.10, length=0.2, half_width=0.14)
    assert not logic.corridor_blocked([(-0.25, 0.3)], -1, edge=0.12, length=0.2, half_width=0.14)
    assert not logic.corridor_blocked([(-0.5, 0.0)], -1, edge=0.12, length=0.2, half_width=0.14)
