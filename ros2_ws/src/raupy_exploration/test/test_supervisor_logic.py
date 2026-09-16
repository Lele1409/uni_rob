"""Unit tests for the ROS-free supervisor logic (no ROS graph needed)."""

from datetime import datetime
import os
import sys

import pytest

try:
    from raupy_exploration import supervisor_logic as logic
except ImportError:
    # Not installed / workspace not sourced: import straight from the package source dir.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from raupy_exploration import supervisor_logic as logic


def make_decider(max_duration_s=600.0, window=90.0, min_new=0.5, grace=20.0):
    return logic.StopDecider(max_duration_s, window, min_new, grace)


def feed(decider, samples, completion=False):
    """Feed (t, area) samples and return the reason from the last update."""
    reason = None
    for t, area in samples:
        reason = decider.update(t, area, completion)
    return reason


# --- known_area_m2 ----------------------------------------------------------------------------

def test_known_area_counts_free_and_occupied_only():
    data = [-1, 0, 100, 50, -1, -1]  # three known cells
    assert logic.known_area_m2(data, 0.05) == pytest.approx(3 * 0.05 ** 2)


def test_known_area_all_unknown_and_empty():
    assert logic.known_area_m2([-1] * 10, 0.05) == 0.0
    assert logic.known_area_m2([], 0.05) == 0.0


def test_known_area_accepts_array_array():
    import array
    data = array.array('b', [0, 0, -1, 100])  # the type rclpy uses for OccupancyGrid.data
    assert logic.known_area_m2(data, 0.1) == pytest.approx(0.03)


# --- StopDecider ------------------------------------------------------------------------------

def test_no_stop_during_grace_period_even_with_full_window():
    # Window shorter than grace, so a full, flat window exists before grace ends.
    d = make_decider(window=5.0, grace=20.0)
    assert feed(d, [(t, 10.0) for t in range(0, 20)]) is None
    # First tick after the grace period: now stagnation is armed.
    assert d.update(20.0, 10.0) == logic.REASON_STAGNATION


def test_no_stagnation_without_full_window():
    # Grace already over, but only 89 s of history for a 90 s window.
    d = make_decider(window=90.0, grace=0.0)
    assert feed(d, [(t, 10.0) for t in range(0, 90)]) is None
    assert d.growth_in_window(89.0) is None
    assert d.update(90.0, 10.0) == logic.REASON_STAGNATION


def test_full_window_counts_from_first_sample_not_zero():
    # History starts late (t=30); the window is only full at t=120.
    d = make_decider(window=90.0, grace=20.0)
    assert feed(d, [(t, 5.0) for t in range(30, 120)]) is None
    assert d.update(120.0, 5.0) == logic.REASON_STAGNATION


def test_stagnation_when_growth_below_threshold():
    d = make_decider(window=90.0, min_new=0.5, grace=20.0)
    # 0.4 m^2 over 90 s: below the 0.5 m^2 threshold.
    samples = [(t, 10.0 + 0.4 * t / 90.0) for t in range(0, 91)]
    assert feed(d, samples) == logic.REASON_STAGNATION


def test_growth_prevents_stagnation():
    d = make_decider(window=90.0, min_new=0.5, grace=20.0)
    # 1 m^2 every 90 s, sustained for a long time: never stagnating.
    samples = [(t, 10.0 + t / 90.0) for t in range(0, 500)]
    for t, area in samples:
        assert d.update(t, area) is None


def test_stagnation_after_growth_stops():
    d = make_decider(max_duration_s=0.0, window=90.0, min_new=0.5, grace=20.0)
    assert feed(d, [(t, float(t)) for t in range(0, 200)]) is None  # growing
    # From t=200 the area stays at 199 m^2.
    reason = None
    t = 200
    while reason is None:
        reason = d.update(t, 199.0)
        t += 1
    assert reason == logic.REASON_STAGNATION
    # The window reference must lie in the frozen phase (t-90 >= 199), i.e. t = 289.
    assert t - 1 == 199 + 90


def test_timeout():
    d = make_decider(max_duration_s=60.0, window=90.0, grace=20.0)
    assert d.update(59.9, 1.0) is None
    assert d.update(60.0, 100.0) == logic.REASON_TIMEOUT


def test_timeout_beats_stagnation():
    d = make_decider(max_duration_s=100.0, window=90.0, grace=20.0)
    feed(d, [(t, 10.0) for t in range(0, 90)])
    # At t=100 both apply (flat full window and budget used up).
    assert d.update(100.0, 10.0) == logic.REASON_TIMEOUT


def test_completion_beats_timeout_and_stagnation():
    d = make_decider(max_duration_s=100.0, window=90.0, grace=20.0)
    feed(d, [(t, 10.0) for t in range(0, 100)])
    assert d.update(100.0, 10.0, completion_received=True) == logic.REASON_COMPLETE


def test_completion_stops_immediately():
    d = make_decider()
    assert d.update(0.0, 0.0, completion_received=True) == logic.REASON_COMPLETE


def test_disabled_timeout_and_stagnation():
    d = make_decider(max_duration_s=0.0, window=0.0, grace=0.0)
    assert feed(d, [(t, 1.0) for t in range(0, 5000, 10)]) is None


def test_history_stays_bounded():
    d = make_decider(max_duration_s=0.0, window=90.0, grace=20.0)
    for t in range(0, 100000):
        d.update(t * 0.1, t * 0.1)  # 10 Hz for ~2.8 h, always growing
    # About one window of samples at 10 Hz, not the whole run.
    assert len(d._history) <= 90 * 10 + 2


def test_clock_jump_back_resets_history():
    d = make_decider(max_duration_s=0.0, window=90.0, grace=0.0)
    feed(d, [(t, 10.0) for t in range(100, 180)])
    # Simulated clock restarted: the old samples must not complete a window.
    assert d.update(5.0, 10.0) is None
    assert d.growth_in_window(5.0) is None


# --- helpers ----------------------------------------------------------------------------------

def test_map_base_path_with_dir_and_default():
    when = datetime(2026, 9, 16, 14, 3, 7)
    assert logic.map_base_path('/tmp/maps', 'raupy_map', when) == \
        '/tmp/maps/raupy_map_20260916_140307'
    assert logic.map_base_path('', 'm', when) == \
        os.path.join(os.path.expanduser('~/raupy_maps'), 'm_20260916_140307')


def test_build_summary():
    s = logic.build_summary(
        reason='stagnation', duration_s=123.456, known_area=42.123, resolution=0.05,
        width=200, height=100, save_results={'map_saver': True, 'slam_serialize': False},
        base_path='/tmp/x')
    assert s['reason'] == 'stagnation'
    assert s['duration_s'] == 123.5
    assert s['known_area_m2'] == 42.12
    assert s['map'] == {'resolution_m': 0.05, 'width_cells': 200, 'height_cells': 100,
                        'width_m': 10.0, 'height_m': 5.0}
    assert s['steps_succeeded'] == {'map_saver': True, 'slam_serialize': False}
    assert s['files_base_path'] == '/tmp/x'
