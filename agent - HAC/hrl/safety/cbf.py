"""Discrete-time CBF-QP safety filter (Sec. 5), vectorised over CAVs.

For a scalar acceleration the QP  argmin ||a - a_raw||^2  s.t. barrier conditions
reduces to clipping a_raw between a lower and an upper bound (Sec. 5.1).

Conventions: distances in metres, gap = bumper-to-bumper distance to the leader,
d_stop = distance from the front bumper to the stop line (> 0 before the line).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class CBFParams:
    dt: float = 0.5
    gamma_c: float = 0.5
    b_max: float = 4.5
    d_s: float = 0.5
    tau_h: float = 1.0
    d_0: float = 2.0
    a_max: float = 3.0
    a_min: float = -4.5
    v_max: float = 13.0
    red_linear_approx: bool = False
    use_stop_backstop: bool = True
    a_lead_conservative: bool = False

    @classmethod
    def from_config(cls, cfg):
        return cls(dt=cfg.dt, gamma_c=cfg.gamma_c, b_max=cfg.b_max, d_s=cfg.d_s, tau_h=cfg.tau_h,
                   d_0=cfg.d_0, a_max=cfg.a_max, a_min=cfg.a_min, v_max=cfg.v_max,
                   red_linear_approx=cfg.red_linear_approx, use_stop_backstop=cfg.use_stop_backstop,
                   a_lead_conservative=cfg.a_lead_conservative)


def h_red(d_stop, v, p):
    return d_stop - v ** 2 / (2 * p.b_max) - p.d_s


def h_green(gap, v, p):
    return gap - p.tau_h * v - p.d_0


def can_stop(d_stop, v, p):
    """Activation test of Sec. 5.2: the vehicle can still stop before the line."""
    return h_red(d_stop, v, p) >= 0


def _quadratic_upper_root(c2, c1, c0):
    """Largest a with c2 a^2 + c1 a - c0 <= 0 (c2 > 0). NaN where no a satisfies it."""
    disc = c1 ** 2 + 4 * c2 * c0
    root = (-c1 + np.sqrt(np.maximum(disc, 0.0))) / (2 * c2)
    return np.where(disc >= 0, root, np.nan)


def a_red_bound(d_stop, v, p):
    dt, b = p.dt, p.b_max
    c2 = dt ** 2 / (2 * b)
    c1 = dt ** 2 / 2 + v * dt / b
    c0 = p.gamma_c * h_red(d_stop, v, p) - v * dt
    if p.red_linear_approx:
        return c0 / c1
    return _quadratic_upper_root(c2, c1, c0)


def a_green_bound(gap, v, v_lead, a_lead, p):
    dt = p.dt
    num = p.gamma_c * h_green(gap, v, p) + (v_lead - v) * dt + 0.5 * a_lead * dt ** 2
    return num / (p.tau_h * dt + 0.5 * dt ** 2)


def a_stop_bound(gap, v, v_lead, a_lead, p):
    """Optional stopping-distance backstop h_stop = gap + v_l^2/2b - v^2/2b - d_0.

    The discrete condition has the same quadratic form as h_red:
    c2 a^2 + c1 a - c0 <= 0 with c2 = dt^2/2b, c1 = dt^2/2 + v dt / b.
    """
    dt, b = p.dt, p.b_max
    h = gap + v_lead ** 2 / (2 * b) - v ** 2 / (2 * b) - p.d_0
    v_lead_next = np.maximum(v_lead + a_lead * dt, 0.0)
    const = (gap + (v_lead - v) * dt + 0.5 * a_lead * dt ** 2 + v_lead_next ** 2 / (2 * b)
             - v ** 2 / (2 * b) - p.d_0 - (1 - p.gamma_c) * h)
    c2 = dt ** 2 / (2 * b)
    c1 = dt ** 2 / 2 + v * dt / b
    return _quadratic_upper_root(c2, c1, const)


def cbf_filter(a_raw, v, d_stop, red_active, has_lead, gap, v_lead, a_lead_prev, p):
    """Project raw accelerations onto the phase-dependent safe set.

    All arguments are 1-D arrays of equal length (one entry per CAV).
    Returns (a_star, info) where info holds the bounds and event flags.
    """
    a_raw = np.asarray(a_raw, dtype=float)
    v = np.asarray(v, dtype=float)
    n = a_raw.shape[0]
    ub = np.minimum(p.a_max, (p.v_max - v) / p.dt)
    lb = np.maximum(p.a_min, -v / p.dt)

    a_lead = np.full(n, -p.b_max) if p.a_lead_conservative else np.clip(a_lead_prev, -p.b_max, p.a_max)
    has_lead = np.asarray(has_lead, dtype=bool)
    gap = np.where(has_lead, np.asarray(gap, dtype=float), 1e6)   # finite placeholder, masked below
    v_lead = np.where(has_lead, np.asarray(v_lead, dtype=float), 0.0)
    d_stop = np.where(np.asarray(red_active, dtype=bool), np.asarray(d_stop, dtype=float), 1e6)
    a_g = np.where(has_lead, a_green_bound(gap, v, v_lead, a_lead, p), np.inf)
    ub = np.minimum(ub, a_g)
    if p.use_stop_backstop:
        a_st = np.where(has_lead, a_stop_bound(gap, v, v_lead, a_lead, p), np.inf)
        ub = np.where(np.isnan(a_st), -np.inf, np.minimum(ub, np.nan_to_num(a_st, nan=np.inf)))

    red_active = np.asarray(red_active, dtype=bool)
    a_r = np.where(red_active, a_red_bound(d_stop, v, p), np.inf)
    red_infeasible = red_active & np.isnan(a_r)
    ub = np.where(red_infeasible, -np.inf, np.minimum(ub, np.nan_to_num(a_r, nan=np.inf)))

    infeasible = ub < lb
    a_star = np.where(infeasible, lb, np.clip(a_raw, lb, np.maximum(ub, lb)))
    info = {
        "lower": lb, "upper": ub, "infeasible": infeasible, "red_infeasible": red_infeasible,
        "intervened": np.abs(a_star - a_raw) > 1e-9,
        "correction": np.maximum(0.0, a_raw - a_star),
    }
    return a_star, info
