"""Pure conversion logic for range_relay.py (no ROS imports, unit-tested).

Raupy's four low ToF sensors (/range/fl, fr, rl, rr, ~5.5 cm above the floor) are published
as single-beam LaserScans; NaN means "nothing within range". Nav2 consumers want
sensor_msgs/Range, and they disagree on how "nothing seen" must be encoded:

- costmap RangeSensorLayer: clears the cone only for range == max_range
  (with clear_on_max_reading); NaN/inf are dropped, so the cone would never be cleared.
- collision_monitor range source: turns a max_range reading into obstacle points at
  max_range; it needs an out-of-span value (+inf) to ignore the reading.
"""

import math


def nearest_hit(ranges, range_min, range_max):
    """Closest valid reading in [range_min, range_max], or None if nothing was detected."""
    valid = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    return min(valid) if valid else None


def to_range_values(ranges, range_min, range_max, cutoff=None):
    """Return (effective_max, costmap_range, monitor_range) for one scan.

    ``cutoff`` (optional) shortens the usable range, e.g. if the sensors pick up the floor
    at a distance; hits beyond it count as "nothing seen".
    """
    effective_max = min(range_max, cutoff) if cutoff and cutoff > 0.0 else range_max
    hit = nearest_hit(ranges, range_min, effective_max)
    if hit is None:
        return effective_max, effective_max, math.inf
    return effective_max, hit, hit
