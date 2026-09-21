"""Pure logic for stuck_monitor.py (no ROS imports, unit-tested).

Stuck = the robot has been commanded to drive (linear |v| >= min_cmd_speed for most of the
last window_s seconds), but the lidar scan has hardly changed over that window.

Why not odometry: Raupy's EKF takes forward speed only from the wheels, so wheels slipping
against an obstacle below the lidar plane look like motion to Nav2 (progress checker) and
to the EKF. The scan comes from the lidar on the robot's body; if the body doesn't move, the
scan doesn't change, whether the wheels are slipping or blocked.

Pure rotation commands (v = 0) are not counted, and any real motion, turning included,
changes the scan, so neither triggers a false "stuck".
"""

from collections import deque
import math

import numpy as np


def scan_change_fraction(ref, cur, abs_tol=0.05, rel_tol=0.02, min_valid=50):
    """Fraction of beams whose range changed by more than abs_tol + rel_tol * range.

    Returns None if the scans can't be compared (different beam count or too few beams
    valid in both), which callers must treat as "can't tell", never as "stuck".
    """
    a = np.asarray(ref, dtype=float)
    b = np.asarray(cur, dtype=float)
    if a.shape != b.shape:
        return None
    valid = np.isfinite(a) & np.isfinite(b) & (a > 0.0) & (b > 0.0)
    n = int(valid.sum())
    if n < min_valid:
        return None
    a, b = a[valid], b[valid]
    changed = np.abs(a - b) > abs_tol + rel_tol * np.minimum(a, b)
    return float(changed.sum()) / n


class StuckDetector:
    """Feed it commanded speeds (on a fixed tick) and scans; ask check(t) on each tick."""

    def __init__(self, window_s=5.0, min_cmd_speed=0.05, min_cmd_fraction=0.9,
                 max_changed_fraction=0.10, abs_tol=0.05, rel_tol=0.02, cooldown_s=10.0):
        self.window_s = window_s
        self.min_cmd_speed = min_cmd_speed
        self.min_cmd_fraction = min_cmd_fraction
        self.max_changed_fraction = max_changed_fraction
        self.abs_tol = abs_tol
        self.rel_tol = rel_tol
        self.cooldown_s = cooldown_s
        self.cmds = deque()   # (t, v)
        self.scans = deque()  # (t, ranges)
        self.quiet_until = -math.inf
        self.last_change = None  # latest scan_change_fraction, for monitoring/tuning

    def add_cmd(self, t, v):
        self.cmds.append((t, v))
        self._trim(t)

    def add_scan(self, t, ranges):
        self.scans.append((t, ranges))
        self._trim(t)

    def _trim(self, t):
        # Keep a bit more than one window, so a reference scan older than window_s exists.
        horizon = t - 1.5 * self.window_s
        while self.cmds and self.cmds[0][0] < horizon:
            self.cmds.popleft()
        while len(self.scans) > 1 and self.scans[1][0] < horizon:
            self.scans.popleft()

    def reset(self, t):
        """Forget history and stay quiet for cooldown_s (call after reacting to a detection)."""
        self.cmds.clear()
        self.scans.clear()
        self.quiet_until = t + self.cooldown_s

    def check(self, t):
        """Return +1 (stuck driving forward), -1 (stuck reversing) or 0 (not stuck)."""
        self.last_change = None
        if t < self.quiet_until or not self.cmds or not self.scans:
            return 0
        start = t - self.window_s
        # The command history must cover the whole window, otherwise we can't judge it.
        if self.cmds[0][0] > start + 0.5:
            return 0
        window = [v for (tc, v) in self.cmds if tc >= start]
        driving = [v for v in window if abs(v) >= self.min_cmd_speed]
        if not window or len(driving) < self.min_cmd_fraction * len(window):
            return 0

        ref = None
        for ts, ranges in self.scans:  # oldest first: the newest scan taken before `start`
            if ts <= start:
                ref = ranges
            else:
                break
        t_cur, cur = self.scans[-1]
        if ref is None or t - t_cur > 1.0:  # no reference yet, or the lidar went quiet
            return 0
        change = scan_change_fraction(ref, cur, self.abs_tol, self.rel_tol)
        self.last_change = change
        if change is None or change > self.max_changed_fraction:
            return 0
        return 1 if sum(driving) > 0 else -1


def virtual_obstacle_points(direction, x_offset=0.21, half_width=0.15, spacing=0.05):
    """Points (x, y) in base_link just outside the footprint, across the front (+1) or rear (-1).

    Just outside, not inside: a lethal cell under the robot would make the planner refuse
    to plan from the current pose.
    """
    n = int(round(2 * half_width / spacing)) + 1
    x = direction * x_offset
    return [(x, -half_width + i * spacing) for i in range(n)]


def to_map_frame(points, tx, ty, yaw):
    """Transform base_link (x, y) points with the 2D pose (tx, ty, yaw) of base_link in map."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(tx + c * x - s * y, ty + s * x + c * y) for x, y in points]


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class NoProgressDetector:
    """Nav2 is trying to drive (a navigate_to_pose goal is active) but the robot doesn't move.

    Catches the deadlock where Nav2 considers the start pose itself in collision: the
    controller, spin and back-up all refuse, goals are aborted and re-sent, and the robot
    stands still until the supervisor ends the run. Stuck detection (StuckDetector) can't
    see this, because nothing is being commanded.
    Feed map-frame poses (SLAM-corrected, so wheel slip doesn't count as progress).
    """

    def __init__(self, window_s=20.0, min_dist=0.10, min_yaw=0.35, min_active_fraction=0.5):
        self.window_s = window_s
        self.min_dist = min_dist
        self.min_yaw = min_yaw
        self.min_active_fraction = min_active_fraction
        self.samples = deque()  # (t, x, y, yaw, goal_active)
        self.quiet_until = -math.inf

    def add(self, t, x, y, yaw, goal_active):
        self.samples.append((t, x, y, yaw, bool(goal_active)))
        while self.samples and self.samples[0][0] < t - 1.5 * self.window_s:
            self.samples.popleft()

    def reset(self, t, quiet_s=0.0):
        self.samples.clear()
        self.quiet_until = t + quiet_s

    def check(self, t):
        if t < self.quiet_until or not self.samples:
            return False
        start = t - self.window_s
        if self.samples[0][0] > start + 0.5:  # window not covered yet
            return False
        window = [s for s in self.samples if s[0] >= start]
        if not window:
            return False
        if sum(1 for s in window if s[4]) < self.min_active_fraction * len(window):
            return False
        _, x0, y0, yaw0, _ = window[0]
        moved = max(math.hypot(x - x0, y - y0) for _, x, y, _, _ in window)
        turned = max(abs(_angle_diff(yaw, yaw0)) for _, _, _, yaw, _ in window)
        return moved < self.min_dist and turned < self.min_yaw


def range_clear(ranges, needed):
    """True if a ToF reading leaves at least `needed` metres (NaN/inf/empty = nothing seen)."""
    finite = [r for r in ranges if math.isfinite(r) and r > 0.0]
    return not finite or min(finite) >= needed


def corridor_blocked(points_xy, direction, edge, length, half_width):
    """Any lidar point (base_link x, y) in the strip the robot sweeps when driving `length`
    metres forward (+1) or backward (-1) from its front/rear `edge` (distance from base_link)?
    """
    lo, hi = edge, edge + length
    for x, y in points_xy:
        if abs(y) <= half_width and lo <= direction * x <= hi:
            return True
    return False
