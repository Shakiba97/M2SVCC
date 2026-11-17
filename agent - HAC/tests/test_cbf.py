"""Stand-alone checks of the CBF-QP filter (no SUMO). Run: python tests/test_cbf.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hrl.safety.cbf import CBFParams, cbf_filter, h_red  # noqa: E402


def advance(x, v, a, dt):
    """Ballistic update with stopping inside the step (same as SUMO ballistic)."""
    a = max(a, -v / dt)
    return x + v * dt + 0.5 * a * dt ** 2, max(0.0, v + a * dt)


def red_light_final_gap(d0, v0, d_s, gamma_c, steps=200):
    """CAV pushing a_max toward a red light; returns final distance to the line (negative = past)."""
    p = CBFParams(d_s=d_s, gamma_c=gamma_c)
    d, v = d0, v0
    for _ in range(steps):
        a, _ = cbf_filter(np.array([p.a_max]), np.array([v]), np.array([d]), np.array([True]),
                          np.array([False]), np.array([np.inf]), np.array([0.0]), np.array([0.0]), p)
        travelled_before = d
        x, v = advance(0.0, v, float(a[0]), p.dt)
        d = travelled_before - x
        if v == 0 and a[0] <= 0:
            break
    return d


def test_red_barrier():
    print("Red-light barrier (a_raw = a_max every step):")
    for d_s in (0.0, 0.5):
        worst = np.inf
        for gamma_c in (0.2, 0.5, 1.0):
            for v0 in np.linspace(0, 13, 14):
                for d0 in np.linspace(5, 120, 24):
                    p = CBFParams(d_s=d_s, gamma_c=gamma_c)
                    if h_red(d0, v0, p) < 0:
                        continue  # cannot stop: the activation rule keeps h_red inactive
                    worst = min(worst, red_light_final_gap(d0, v0, d_s, gamma_c))
        print(f"  d_s={d_s}: closest final distance to the line = {worst:+.3f} m")
        if d_s == 0.5:
            assert worst >= 0, "vehicle crossed the stop line with d_s = 0.5"


def test_green_barrier(p=None):
    p = p or CBFParams()
    worst_gap = np.inf
    rng = np.random.default_rng(0)
    for trial in range(300):
        gap, v, vl = 30.0, 13.0, 13.0
        al_prev = 0.0
        brake = rng.uniform(1.0, p.b_max)
        start = rng.integers(2, 20)
        for t in range(120):
            al = -brake if t >= start else 0.0
            a, _ = cbf_filter(np.array([p.a_max]), np.array([v]), np.array([np.inf]), np.array([False]),
                              np.array([True]), np.array([gap]), np.array([vl]), np.array([al_prev]), p)
            xe, v_new = advance(0.0, v, float(a[0]), p.dt)
            xl, vl_new = advance(0.0, vl, al, p.dt)
            gap += xl - xe
            v, vl, al_prev = v_new, vl_new, (vl_new - vl) / p.dt
            worst_gap = min(worst_gap, gap)
    print(f"Car-following barrier: smallest bumper gap = {worst_gap:.3f} m (d_0 = {p.d_0})")
    assert worst_gap >= p.d_0 - 1e-6, "gap fell below d_0"


def test_infeasible_applies_lower_bound():
    p = CBFParams()
    a, info = cbf_filter(np.array([2.0]), np.array([13.0]), np.array([3.0]), np.array([True]),
                         np.array([False]), np.array([np.inf]), np.array([0.0]), np.array([0.0]), p)
    assert info["infeasible"][0] and abs(a[0] - p.a_min) < 1e-9
    print("Infeasible red case: filter applies a_lower_bound and flags the event")


def gamma_sweep():
    print("gamma_c sweep (smallest bumper gap, headway barrier alone vs + h_stop backstop):")
    for g in (0.2, 0.5, 1.0):
        out = []
        for backstop in (False, True):
            try:
                test_green_barrier(CBFParams(gamma_c=g, use_stop_backstop=backstop))
                out.append("ok")
            except AssertionError:
                out.append("BELOW d_0")
        print(f"  gamma_c={g}: alone {out[0]}, with backstop {out[1]}")


if __name__ == "__main__":
    test_red_barrier()
    test_green_barrier()  # defaults: gamma_c = 0.5 with the h_stop backstop
    test_infeasible_applies_lower_bound()
    gamma_sweep()
    print("all CBF tests passed")
