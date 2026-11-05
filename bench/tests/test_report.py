"""bench/report.py: N@q interpolation and Kendall's tau."""
import math

from bench.report import kendall_tau, n_at_q


def test_n_at_q_interpolates_between_budgets():
    assert abs(n_at_q([5, 10, 20], [0.2, 0.5, 0.9], 0.7) - 15.0) < 1e-9
    assert n_at_q([5, 10], [0.8, 0.9], 0.5) == 5.0
    assert n_at_q([5, 10], [0.1, 0.2], 0.5) is None


def test_kendall_tau():
    assert kendall_tau(list("abcd"), list("abcd")) == 1.0
    assert kendall_tau(list("abcd"), list("dcba")) == -1.0
    assert math.isclose(kendall_tau(list("abc"), list("acb")), 1 / 3)
