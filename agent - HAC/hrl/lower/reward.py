"""Lower reward (Sec. 7.2) and the window-end position target (Sec. 7.3).

Energy models use the same coefficients as the repo's metrics
(environment/single_intersection.py) and the GAMS faster-scale NLP.
"""
import numpy as np


def ice_fuel_rate(v, a):
    """Polynomial ICE fuel rate (M2SVCC Eq. 10); same polynomial as the repo metrics."""
    return (0.2736 + 0.0599 * v + 0.3547 * a - 0.0058 * v ** 2 + 0.0179 * v * a
            + 0.0663 * a ** 2 + 0.0002 * v ** 3 + 0.002 * v ** 2 * a + 0.0245 * v * a ** 2
            - 0.0489 * a ** 3)


def ev_power_kw(v, a):
    """EV traction power P = max(0, F v) (Eq. 8-9), repo coefficients (1266 kg, C_r = 0.006)."""
    return np.maximum(0.0, (1266 * v * a + 1266 * 9.8 * 0.006 * v + 1.3 * v ** 3) / 1000.0)


def lower_step_reward(cfg, *, is_ev, v, a, dist, gated, a_raw, a_star, gap_next, v_next, has_lead_next):
    """Per-step lower reward without the window-end term (Sec. 7.2). Arrays over CAVs."""
    dt = cfg.dt
    energy = np.where(is_ev, cfg.w_e * ev_power_kw(v, a) * dt, cfg.w_f * ice_fuel_rate(v, a) * dt)
    distance = np.where(gated, dist, 0.0)
    intervention = cfg.w_s * dt * np.maximum(0.0, a_raw - a_star)
    headway = np.where(has_lead_next,
                       cfg.lambda_safe * np.maximum(0.0, cfg.tau_h * v_next + cfg.d_0 - gap_next) ** 2, 0.0)
    total = -energy + distance - intervention - headway
    parts = {"energy": energy, "distance": distance, "intervention": intervention, "headway": headway}
    return total, parts


def window_goal_reward(cfg, *, s_ref, s_ego, gap, v_ego, has_lead):
    """Capped one-sided window-end target (Sec. 7.3).

    target = min(s_ref, s_lead - L_lead - d_0 - tau_h v_ego); with gap the bumper gap,
    s_lead - L_lead = s_ego + gap.
    """
    cap = np.where(has_lead, s_ego + gap - cfg.d_0 - cfg.tau_h * v_ego, np.inf)
    target = np.minimum(s_ref, cap)
    shortfall = np.maximum(0.0, target - s_ego)
    return -cfg.w_g * shortfall ** 2, shortfall


def goal_potential(cfg, *, u_goal, ds_ref, v_ego, tau_win):
    """Phi = -w_g max(0, s_ref - s_ego - v tau_win)^2 (Sec. 7.3, optional shaping)."""
    return np.where(u_goal > 0, -cfg.w_g * np.maximum(0.0, ds_ref - v_ego * tau_win) ** 2, 0.0)
