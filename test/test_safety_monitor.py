"""
Unit tests for SafetyMonitor (rec_rep2/safety_monitor.py) -- the fault
latch that sits inside the compliant-torque control loop before every
torque command. Pure numpy, no ROS dependency.
"""

import numpy as np
import pytest

from rec_rep2.safety_monitor import FaultType, SafetyMonitor


def make_monitor(**overrides):
    kwargs = dict(
        tau_limits=[1.0, 1.0],
        vel_limits=[1.0, 1.0],
        max_missed_cycles=3,
        dt_nominal=0.005,
        overrun_factor=3.0,
    )
    kwargs.update(overrides)
    return SafetyMonitor(**kwargs)


def test_within_limits_passes_through_unmodified():
    mon = make_monitor()
    tau_cmd = np.array([0.5, -0.5])
    tau_safe, faulted, reason = mon.check_and_clamp(
        tau_cmd, q_dot=np.array([0.2, -0.2]), dt_actual=0.005,
    )
    assert not faulted
    assert reason is None
    assert np.array_equal(tau_safe, tau_cmd)
    assert not mon.is_faulted


def test_torque_saturation_clamps_without_faulting():
    mon = make_monitor()
    tau_safe, faulted, reason = mon.check_and_clamp(
        np.array([2.0, -3.0]), q_dot=np.zeros(2), dt_actual=0.005,
    )
    assert not faulted
    assert reason is None
    assert np.array_equal(tau_safe, np.array([1.0, -1.0]))
    assert not mon.is_faulted


def test_velocity_trip_latches_fault_and_zeroes_torque():
    mon = make_monitor()
    tau_safe, faulted, reason = mon.check_and_clamp(
        np.array([0.1, 0.1]), q_dot=np.array([0.2, 1.5]), dt_actual=0.005,
    )
    assert faulted
    assert 'Joint 1' in reason
    assert np.array_equal(tau_safe, np.zeros(2))
    assert mon.is_faulted
    assert mon.fault_record.fault_type is FaultType.VELOCITY_TRIP
    assert mon.fault_record.joint_index == 1


def test_fault_stays_latched_until_reset():
    mon = make_monitor()
    mon.check_and_clamp(np.zeros(2), q_dot=np.array([0.0, 5.0]), dt_actual=0.005)
    assert mon.is_faulted

    # Even a perfectly safe command is now rejected -- the fault is latched.
    tau_safe, faulted, _ = mon.check_and_clamp(
        np.array([0.1, 0.1]), q_dot=np.zeros(2), dt_actual=0.005,
    )
    assert faulted
    assert np.array_equal(tau_safe, np.zeros(2))

    mon.reset()
    assert not mon.is_faulted
    assert mon.fault_record is None

    tau_safe, faulted, _ = mon.check_and_clamp(
        np.array([0.1, 0.1]), q_dot=np.zeros(2), dt_actual=0.005,
    )
    assert not faulted


def test_loop_overrun_requires_consecutive_misses():
    mon = make_monitor(max_missed_cycles=3)
    over_dt = mon._dt_nominal * mon._overrun_factor * 2  # well over threshold

    for _ in range(2):
        _, faulted, _ = mon.check_and_clamp(
            np.zeros(2), q_dot=np.zeros(2), dt_actual=over_dt,
        )
        assert not faulted

    _, faulted, reason = mon.check_and_clamp(
        np.zeros(2), q_dot=np.zeros(2), dt_actual=over_dt,
    )
    assert faulted
    assert mon.fault_record.fault_type is FaultType.LOOP_OVERRUN


def test_good_cycle_resets_overrun_counter():
    mon = make_monitor(max_missed_cycles=3)
    over_dt = mon._dt_nominal * mon._overrun_factor * 2
    good_dt = mon._dt_nominal

    mon.check_and_clamp(np.zeros(2), q_dot=np.zeros(2), dt_actual=over_dt)
    mon.check_and_clamp(np.zeros(2), q_dot=np.zeros(2), dt_actual=over_dt)
    # A single good cycle should reset the miss counter to zero...
    mon.check_and_clamp(np.zeros(2), q_dot=np.zeros(2), dt_actual=good_dt)

    # ...so two more overruns are not enough to trip.
    for _ in range(2):
        _, faulted, _ = mon.check_and_clamp(
            np.zeros(2), q_dot=np.zeros(2), dt_actual=over_dt,
        )
        assert not faulted
    assert not mon.is_faulted


def test_trip_exception_latches_fault():
    mon = make_monitor()
    mon.trip_exception(RuntimeError('boom'))
    assert mon.is_faulted
    assert mon.fault_record.fault_type is FaultType.EXCEPTION

    _, faulted, _ = mon.check_and_clamp(
        np.zeros(2), q_dot=np.zeros(2), dt_actual=0.005,
    )
    assert faulted


@pytest.mark.parametrize('sign', [1.0, -1.0])
def test_velocity_trip_fires_on_either_sign(sign):
    mon = make_monitor()
    _, faulted, reason = mon.check_and_clamp(
        np.zeros(2), q_dot=np.array([0.0, sign * 2.0]), dt_actual=0.005,
    )
    assert faulted
    assert '±' in reason
