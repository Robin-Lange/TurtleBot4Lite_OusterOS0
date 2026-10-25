"""Deterministic regressions over the actual runtime systems, without ROS or mocks."""

from copy import deepcopy
from dataclasses import replace
import math

import numpy as np
import pytest

from tb4_ouster_autonomy.components import Authority, Command, SafetyConfig, Sample, Velocity, World
from tb4_ouster_autonomy.systems import (
    contact_points_system, obstacle_filter_system, safety_system, sample_fresh, transform_points,
)


def healthy_world():
    return World(
        Authority(docked=False, deadman=True, deadman_received=10.0),
        {name: Sample(10.0, 1000.0) for name in ('dock', 'hazard', 'odom', 'tf', 'scan', 'cloud')},
        {'NAV2': Command(Velocity(0.12, 0.2), 10.0)},
    )


def decide(world, **overrides):
    return safety_system(world, SafetyConfig(allow_nav2=True, **overrides), 10.1, 1000.1)


def test_default_world_cannot_move():
    assert decide(World()).velocity == Velocity()


def test_healthy_autonomous_command_does_not_require_teleop_deadman():
    world = healthy_world()
    world.authority.deadman = False
    assert decide(world).velocity == Velocity(0.12, 0.2)


@pytest.mark.parametrize('entity', ['dock', 'hazard', 'odom', 'tf', 'scan', 'cloud'])
@pytest.mark.parametrize('failure', ['missing', 'receipt', 'stamp', 'future'])
def test_each_required_sensor_fails_closed(entity, failure):
    world = healthy_world()
    if failure == 'missing':
        del world.samples[entity]
    else:
        world.samples[entity] = {'receipt': Sample(1.0, 1000.0), 'stamp': Sample(10.0, 990.0),
                                 'future': Sample(10.0, 1001.0)}[failure]
    assert decide(world).velocity == Velocity()
    assert entity in decide(world).reason


@pytest.mark.parametrize('stamp', [0, math.nan, math.inf, -math.inf])
def test_invalid_acquisition_stamp(stamp):
    assert not sample_fresh(Sample(10, stamp), 10, 1000, 1, 0.1)


def test_receipt_and_ros_clock_have_separate_domains():
    assert sample_fresh(Sample(10, 1000), 10.1, 1000.1, 0.5, 0.1)
    assert not sample_fresh(Sample(11, 1000), 10.1, 1000.1, 0.5, 0.1)


def test_priority_manual_stop_does_not_fall_through_to_nav2():
    world = healthy_world()
    world.commands['TELEOP'] = Command(Velocity(), 10)
    assert decide(world).source == 'TELEOP'
    assert decide(world).velocity == Velocity()


def test_manual_override_remains_paused_after_command_expires():
    world = healthy_world()
    world.authority.autonomy_paused = True
    world.commands['TELEOP'] = Command(Velocity(0.1), 9)
    assert decide(world).velocity == Velocity()


def test_deadman_release_blocks_manual_without_falling_through():
    world = healthy_world()
    world.authority.deadman = False
    world.commands['TELEOP'] = Command(Velocity(0.1), 10)
    assert decide(world).state == 'INHIBITED_DEADMAN'


def test_explorer_proposal_preempts_nav2_only_while_fresh():
    world = healthy_world()
    world.commands['EXPLORER'] = Command(Velocity(0, 0.4), 10)
    assert decide(world).source == 'EXPLORER'
    world.commands['EXPLORER'] = Command(Velocity(0, 0.4), 9)
    assert decide(world).source == 'NAV2'


@pytest.mark.parametrize('velocity', [Velocity(math.nan), Velocity(math.inf), Velocity(0, -math.inf)])
def test_invalid_commands_do_not_become_full_speed(velocity):
    world = healthy_world()
    world.commands['NAV2'] = Command(velocity, 10)
    assert decide(world).state == 'INVALID_COMMAND'
    assert decide(world).velocity == Velocity()


def test_timeout_and_future_receipt_stop():
    world = healthy_world()
    for received in (9.0, 11.0, math.nan):
        world.commands['NAV2'] = Command(Velocity(0.1), received)
        assert decide(world).velocity == Velocity()


def test_clamp_and_system_does_not_mutate_world():
    world = healthy_world()
    world.commands['NAV2'] = Command(Velocity(-5, 5), 10)
    before = deepcopy(world)
    assert decide(world).velocity == Velocity(-0.15, 0.5)
    assert before == world


def test_latched_hazard_and_backup_limit():
    world = healthy_world()
    world.authority.fault = 'BUMP'
    assert decide(world).state == 'LATCHED_FAULT'
    world.authority.fault = ''
    world.authority.hazards = (0,)
    world.commands['NAV2'] = Command(Velocity(-0.06), 10)
    assert decide(world).state == 'INHIBITED_BACKUP_LIMIT'


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf])
def test_invalid_config_rejected(value):
    with pytest.raises(ValueError):
        replace(SafetyConfig(), cmd_timeout_s=value)


def test_optical_depth_becomes_forward_distance_before_height_filter():
    # REP-103 optical: right/down/forward -> body: forward/left/up.
    points = transform_points([[0, 0, 3], [0, 0.176, 2]], [-0.05, -0.005, 0.176], [-0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(points, [[2.95, -0.005, 0.176], [1.95, -0.005, 0]], atol=1e-6)
    kept = obstacle_filter_system(points, 0.1, 6, 0.04, 1.5, [-0.18, -0.18, -0.2], [0.18, 0.18, 0.25])
    assert len(kept) == 1  # Distant obstacle survives; floor does not.


def test_filter_empty_self_returns_nonfinite_and_ceiling():
    points = [[1, 0, 0.2], [0.15, 0, 0.1], [1, 0, 0], [1, 0, 2], [math.inf, 0, 0.2], [math.nan, 0, 0.2]]
    kept = obstacle_filter_system(points, 0.1, 6, 0.04, 1.5, [-0.18, -0.18, -0.2], [0.18, 0.18, 0.25])
    np.testing.assert_allclose(kept, [[1, 0, 0.2]])
    assert obstacle_filter_system([], 0.1, 6, 0.04, 1.5, [-1]*3, [1]*3).shape == (0, 3)


def test_invalid_tf_quaternion():
    with pytest.raises(ValueError):
        transform_points([[1, 2, 3]], [0, 0, 0], [0, 0, 0, 0])


def test_contacts_are_anchored_in_odom_at_acquisition():
    points = contact_points_system([0], 0.175, (1, 2, math.pi/2))
    np.testing.assert_allclose(points, [[1, 2.175, 0.04], [1, 2.175, 0.12]])
