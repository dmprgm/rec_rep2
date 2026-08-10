"""
Unit tests for FrictionObserver (rec_rep2/friction_observer.py).

Pure numpy, no ROS dependency -- runnable with plain `pytest` (see
requirements.txt / readme.md "External driver stack").
"""

import numpy as np
import pytest

from rec_rep2.friction_observer import FrictionObserver


def test_reset_zeroes_state():
    obs = FrictionObserver(n_joints=3, L=[2.0, 2.0, 2.0], Lp=[1.0, 1.0, 1.0])
    g_q = np.zeros(3)
    b_qdot = np.zeros(3)
    for _ in range(50):
        obs.update(np.array([1.0, -1.0, 0.5]), g_q, b_qdot, dt=0.005)
    assert not np.allclose(obs._xi, 0.0)

    obs.reset()

    assert np.array_equal(obs._xi, np.zeros(3))


def test_zero_residual_stays_at_zero():
    """If tau_measured == g_q + b_qdot exactly, the residual is always zero,
    so the estimate never moves off its zero initial state."""
    obs = FrictionObserver(n_joints=2, L=[2.0, 5.0], Lp=[1.0, 0.5])
    g_q = np.array([0.7, -0.3])
    b_qdot = np.array([0.1, 0.2])
    tau_measured = g_q + b_qdot

    for _ in range(200):
        estimate = obs.update(tau_measured, g_q, b_qdot, dt=0.005)

    assert np.allclose(estimate, 0.0)


def test_converges_to_analytic_steady_state():
    """
    Continuous-time steady state of xi' = -(L+Lp)*xi + L*r (constant
    residual r) is xi_ss = L*r/(L+Lp). Forward-Euler integration with a
    small dt over many time constants should land close to it.
    """
    L = np.array([2.0, 4.0])
    Lp = np.array([1.0, 1.0])
    obs = FrictionObserver(n_joints=2, L=list(L), Lp=list(Lp))

    residual = np.array([1.0, -0.5])
    g_q = np.zeros(2)
    b_qdot = np.zeros(2)
    tau_measured = residual + g_q + b_qdot

    dt = 1e-3
    for _ in range(20_000):  # >> 1/(L+Lp) time constants for both joints
        estimate = obs.update(tau_measured, g_q, b_qdot, dt)

    xi_ss = L * residual / (L + Lp)
    assert estimate == pytest.approx(xi_ss, abs=1e-3)


def test_joints_evolve_independently():
    """A large residual on one joint must not perturb another joint's estimate."""
    obs = FrictionObserver(n_joints=2, L=[2.0, 2.0], Lp=[1.0, 1.0])
    g_q = np.zeros(2)
    b_qdot = np.zeros(2)

    for _ in range(500):
        estimate = obs.update(np.array([10.0, 0.0]), g_q, b_qdot, dt=0.005)

    assert estimate[1] == 0.0
    assert estimate[0] != 0.0


def test_update_returns_a_copy_not_internal_state():
    obs = FrictionObserver(n_joints=1, L=[2.0], Lp=[1.0])
    estimate = obs.update(np.array([1.0]), np.zeros(1), np.zeros(1), dt=0.005)
    estimate[0] = 999.0
    assert obs._xi[0] != 999.0
