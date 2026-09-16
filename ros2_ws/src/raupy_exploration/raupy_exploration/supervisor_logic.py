"""ROS-free decision logic of the exploration supervisor.

Everything in here works on plain numbers so that it can be unit tested without a ROS graph,
a simulator or the robot. The node in ``scripts/exploration_supervisor.py`` only feeds ROS data
into these functions and turns the results into service calls.
"""

from collections import deque
from datetime import datetime
import os

import numpy as np

# Stop reasons, also written verbatim into the summary file.
REASON_COMPLETE = 'complete'
REASON_TIMEOUT = 'timeout'
REASON_STAGNATION = 'stagnation'

# OccupancyGrid value for "never observed".
UNKNOWN_CELL = -1


def known_area_m2(data, resolution):
    """Return the observed (free or occupied) area of an occupancy grid in square metres.

    Free and occupied cells both count, because both mean the robot has *seen* that spot;
    this is the "information gained" measure used for the stagnation check.

    :param data: flat cell values as in ``nav_msgs/OccupancyGrid.data`` (list, array.array or
        numpy array of int8, -1 = unknown).
    :param resolution: cell edge length in metres.
    """
    # numpy instead of a Python loop: slam_toolbox maps easily reach millions of cells and
    # this runs on every map update.
    cells = np.asarray(data, dtype=np.int8)
    known_cells = int(np.count_nonzero(cells != UNKNOWN_CELL))
    return known_cells * float(resolution) ** 2


class StopDecider:
    """Decide when autonomous exploration should end.

    Feed it one sample per supervisor tick via :meth:`update`. Times are seconds since the
    exploration started (the first received map), not wall time, so the decider works the
    same with real and simulated clocks.

    Stop reasons, highest precedence first if several apply in the same tick:

    1. ``complete``   - the explorer reported that no frontiers are left. It is the most
       informative reason, so it wins over the others.
    2. ``timeout``    - the time budget ``max_duration_s`` is used up. A hard budget beats the
       heuristic stagnation check.
    3. ``stagnation`` - the known area grew by less than ``min_new_area_m2`` during the last
       ``stagnation_window_s``. The check is only armed after ``startup_grace_s`` (SLAM and
       Nav2 need a moment before the robot moves) and only once a full window of history
       exists (otherwise a short history would look like "no growth").

    A non-positive ``max_duration_s`` or ``stagnation_window_s`` disables that condition.
    """

    def __init__(self, max_duration_s, stagnation_window_s, min_new_area_m2, startup_grace_s):
        self.max_duration_s = float(max_duration_s)
        self.stagnation_window_s = float(stagnation_window_s)
        self.min_new_area_m2 = float(min_new_area_m2)
        self.startup_grace_s = float(startup_grace_s)
        # (t, known_area) samples, oldest first. Pruned in _remember() so it only spans a
        # little more than one stagnation window and cannot grow over a long run.
        self._history = deque()

    def update(self, t, known_area, completion_received=False):
        """Add a sample and return the stop reason, or ``None`` to keep exploring.

        :param t: seconds since exploration start.
        :param known_area: current known map area in m^2.
        :param completion_received: True once the explorer's completion event arrived.
        """
        t = float(t)
        self._remember(t, float(known_area))

        if completion_received:
            return REASON_COMPLETE
        if self.max_duration_s > 0.0 and t >= self.max_duration_s:
            return REASON_TIMEOUT
        if self._is_stagnating(t):
            return REASON_STAGNATION
        return None

    def growth_in_window(self, t):
        """Return area growth over the last window, or ``None`` if no full window exists yet."""
        if self.stagnation_window_s <= 0.0 or not self._history:
            return None
        oldest_t, oldest_area = self._history[0]
        # The oldest kept sample is the reference point. It only covers the whole window if it
        # lies at least one window length in the past.
        if oldest_t > t - self.stagnation_window_s:
            return None
        return self._history[-1][1] - oldest_area

    def _is_stagnating(self, t):
        if t < self.startup_grace_s:
            return False
        growth = self.growth_in_window(t)
        return growth is not None and growth < self.min_new_area_m2

    def _remember(self, t, area):
        # A jump back in time means the (simulated) clock was reset; old samples would
        # produce nonsense windows, so start the history over.
        if self._history and t < self._history[-1][0]:
            self._history.clear()
        self._history.append((t, area))

        if self.stagnation_window_s <= 0.0:
            # Stagnation disabled: only the newest sample is ever needed.
            while len(self._history) > 1:
                self._history.popleft()
            return

        # Drop samples as long as the *next* one is still old enough to serve as the window
        # reference. This keeps exactly one sample at or before (t - window), so the growth
        # is always measured over at least the full window.
        cutoff = t - self.stagnation_window_s
        while len(self._history) >= 2 and self._history[1][0] <= cutoff:
            self._history.popleft()


def map_base_path(output_dir, map_name, when=None):
    """Return ``<output_dir>/<map_name>_<YYYYmmdd_HHMMSS>`` without file extension.

    The timestamp keeps runs from overwriting each other. An empty ``output_dir`` means
    ``~/raupy_maps``, so the default works regardless of the directory ros2 was started in.
    """
    when = when or datetime.now()
    directory = os.path.expanduser(output_dir) if output_dir else os.path.expanduser(
        '~/raupy_maps')
    return os.path.join(directory, f'{map_name}_{when.strftime("%Y%m%d_%H%M%S")}')


def build_summary(reason, duration_s, known_area, resolution, width, height, save_results,
                  base_path):
    """Build the run summary as plain Python types (safe for ``yaml.safe_dump``).

    :param save_results: mapping step name -> bool, e.g. ``{'map_saver': True}``.
    """
    return {
        'reason': str(reason),
        'duration_s': round(float(duration_s), 1),
        'known_area_m2': round(float(known_area), 2),
        'map': {
            'resolution_m': round(float(resolution), 4),  # msg field is float32
            'width_cells': int(width),
            'height_cells': int(height),
            'width_m': round(int(width) * float(resolution), 2),
            'height_m': round(int(height) * float(resolution), 2),
        },
        'steps_succeeded': {str(k): bool(v) for k, v in save_results.items()},
        'files_base_path': str(base_path),
    }
