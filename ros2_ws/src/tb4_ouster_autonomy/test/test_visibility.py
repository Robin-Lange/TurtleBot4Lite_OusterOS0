"""Scan geometry and freshness regressions without ROS or synthetic ground truth."""

from dataclasses import replace
import math

import pytest

from tb4_ouster_autonomy.components import Sample, VisibilityConfig, VisibilityScan
from tb4_ouster_autonomy.visibility import scan_visibility_system, visibility_health_system

CONFIG = VisibilityConfig()


def scan(samples, angle_min=-0.03, increment=0.01, **overrides):
    values = dict(
        ranges=tuple(samples), angle_min=angle_min,
        angle_max=angle_min + (len(samples) - 1) * increment,
        angle_increment=increment, range_min=0.1, range_max=30.0,
    )
    values.update(overrides)
    return VisibilityScan(**values)


def observe(ranges, **kwargs):
    return scan_visibility_system(scan(ranges, **kwargs), CONFIG)


def test_door_edge_locates_near_beam_in_sensor_coordinates():
    result = observe([2., 2., 2., 5., 5., 5.])
    assert result.finite_count == result.beam_count == 6
    assert not result.full_circle
    edge, = result.edges
    assert edge.beam_index == 2
    assert edge.angle == pytest.approx(-0.01)
    assert (edge.near_range, edge.far_range) == (2., 5.)


def test_reversing_scan_order_preserves_physical_edge():
    forward = observe([2., 2., 5., 5.], angle_min=-0.01)
    reverse = observe([5., 5., 2., 2.], angle_min=0.02, increment=-0.01)
    assert forward.edges[0].angle == pytest.approx(reverse.edges[0].angle)
    assert forward.edges[0].near_range == reverse.edges[0].near_range


@pytest.mark.parametrize('missing', [math.inf, -math.inf, math.nan, 0., 0.05, 31.])
def test_invalid_returns_remain_unknown_and_do_not_bridge_gaps(missing):
    result = observe([2., 2., missing, 5., 5.])
    assert result.edges == ()
    assert result.finite_count == 4


def test_finite_range_limits_are_inclusive():
    result = observe([0.1, 30.])
    assert result.finite_count == 2
    assert result.edges[0].near_range == 0.1


def test_no_edges_on_constant_wall_or_distant_or_small_jump():
    assert not observe([2.] * 100).edges
    assert not observe([7., 10.]).edges
    assert not observe([2., 2.5]).edges


def test_near_beam_bordering_two_jumps_is_not_double_counted():
    result = observe([5., 2., 6.])
    edge, = result.edges
    assert edge.beam_index == 1 and edge.far_range == 6.


def test_partial_scan_does_not_join_its_endpoints():
    assert not observe([2., 2., 2.5, 3., 3.5, 4., 4.5, 5.]).edges


@pytest.mark.parametrize('duplicate_endpoint', [True, False])
@pytest.mark.parametrize('direction', [-1, 1])
def test_full_circle_scan_and_seam(direction, duplicate_endpoint):
    # All internal differences are < threshold; only the seam has a jump.
    ranges = [2. + 3. * index / 359 for index in range(360)]
    if duplicate_endpoint:
        ranges.append(2.)
    result = observe(ranges, angle_min=0., increment=direction * math.tau / 360)
    assert result.full_circle
    assert len(result.edges) == 1
    assert result.edges[0].near_range == 2.


def test_zero_return_is_unknown_even_with_zero_range_min():
    assert observe([0., 2.], range_min=0.).finite_count == 1


@pytest.mark.parametrize('overrides', [
    {'ranges': ()}, {'ranges': (2.,)}, {'angle_increment': 0.},
    {'angle_min': math.nan}, {'angle_max': math.inf}, {'range_min': -1.},
    {'range_max': math.inf}, {'range_min': 30.}, {'angle_max': 2.},
])
def test_malformed_scans_are_rejected(overrides):
    with pytest.raises(ValueError):
        scan_visibility_system(scan([2., 2., 2.], **overrides), CONFIG)


def test_multiple_revolutions_are_rejected():
    with pytest.raises(ValueError):
        observe([2.] * 1000, increment=0.01)


def test_coarse_resolution_is_explicitly_limited():
    result = observe([2., 5., 5., 5.], increment=math.tau / 4)
    assert result.edges == () and result.full_circle and not result.resolution_supported
    health = visibility_health_system(result, Sample(10., 100.), '', CONFIG, 10., 100.)
    assert health.state == 'LIMITED'
    assert 'spacing' in health.reason


@pytest.mark.parametrize('name,value', [
    ('range_jump_m', 0.), ('max_edge_range_m', math.inf),
    ('max_neighbor_angle_rad', -1.), ('timeout_s', math.nan),
    ('min_finite_fraction', 1.1), ('future_tolerance_s', 0.),
])
def test_bad_parameters_are_rejected(name, value):
    with pytest.raises(ValueError):
        replace(CONFIG, **{name: value})


@pytest.mark.parametrize('sample,now,ros_now', [
    (None, 10., 100.), (Sample(9., 100.), 10., 100.),
    (Sample(10., 99.), 10., 100.), (Sample(10., 100.2), 10., 100.),
    (Sample(10., 0.), 10., 100.), (Sample(10., 100.), 9., 100.),
    (Sample(10., 100.), 10., 98.),
])
def test_fresh_receipt_cannot_hide_bad_acquisition_or_clock_jumps(sample, now, ros_now):
    health = visibility_health_system(observe([2., 5.]), sample, '', CONFIG, now, ros_now)
    assert health.state == 'STALE' and not health.show_edges


def test_health_distinguishes_invalid_unknown_partial_edges_and_observed():
    def health(result, error=''):
        return visibility_health_system(result, Sample(10., 100.), error, CONFIG, 10., 100.)

    assert health(None, 'bad angles').state == 'INVALID'
    unknown = health(observe([math.inf] * 360, increment=math.tau / 360))
    assert unknown.state == 'LIMITED' and 'unknown' in unknown.reason
    assert health(observe([2.] * 30)).state == 'LIMITED'
    ranges = [2.] * 180 + [5.] * 180
    assert health(observe(ranges, increment=math.tau / 360)).state == 'EDGES'
    clear_edges = health(observe([5.] * 360, increment=math.tau / 360))
    assert clear_edges.state == 'OBSERVED' and 'does not establish safety' in clear_edges.reason
