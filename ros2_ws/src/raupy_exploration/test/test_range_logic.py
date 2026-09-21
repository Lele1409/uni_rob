import math

from raupy_exploration.range_logic import nearest_hit, to_range_values


def test_nan_means_nothing_seen():
    # costmap gets max_range (clears the cone), monitor gets +inf (ignored)
    assert to_range_values([math.nan], 0.01, 0.9) == (0.9, 0.9, math.inf)


def test_hit_goes_to_both():
    assert to_range_values([0.3], 0.01, 0.9) == (0.9, 0.3, 0.3)


def test_cutoff_turns_far_hits_into_nothing_seen():
    assert to_range_values([0.5], 0.01, 0.9, cutoff=0.35) == (0.35, 0.35, math.inf)
    assert to_range_values([0.2], 0.01, 0.9, cutoff=0.35) == (0.35, 0.2, 0.2)


def test_cutoff_above_sensor_max_is_ignored():
    assert to_range_values([math.nan], 0.01, 0.9, cutoff=2.0)[0] == 0.9


def test_out_of_span_and_inf_are_not_hits():
    assert nearest_hit([0.005, math.inf, -math.inf, 1.5], 0.01, 0.9) is None


def test_nearest_of_several_beams():
    assert nearest_hit([0.6, math.nan, 0.4], 0.01, 0.9) == 0.4


def test_empty_scan():
    assert to_range_values([], 0.01, 0.9) == (0.9, 0.9, math.inf)
