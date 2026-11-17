"""S_lower (Sec. 3.3), normalised to O(1).

Additions to the methodology's vector, needed because a phase change inserts a 7 s
clearance before the phase-pure window (see README): g_next (ego link green in the
selected phase k) and tau_clear (clearance time left). g_lane is the ego link's actual
right-of-way now, which is what gates the distance term m_t.
"""
import numpy as np

from ..common.encoding import one_hot

LOWER_DIM = 20


def lower_obs(cfg, *, d_stop, v, gap, has_lead, v_lead, a_lead, phase_k, tau_win, g_lane, u_goal, ds_ref,
              is_ev, g_next, tau_clear):
    n = len(v)
    out = np.zeros((n, LOWER_DIM), np.float32)
    if n == 0:
        return out
    gap_c = np.where(has_lead, np.minimum(gap, cfg.control_range), cfg.control_range)
    out[:, 0] = np.clip(d_stop, -cfg.L_exit, cfg.control_range) / 100.0
    out[:, 1] = v / cfg.v_max
    out[:, 2] = gap_c / 100.0
    out[:, 3] = np.where(has_lead, v_lead, cfg.v_max) / cfg.v_max
    out[:, 4] = np.where(has_lead, np.clip(a_lead, -cfg.b_max, cfg.a_max), 0.0) / cfg.a_max
    out[:, 5:13] = one_hot(phase_k)
    out[:, 13] = tau_win / (cfg.window + cfg.yellow_time + cfg.all_red_time)
    out[:, 14] = g_lane
    out[:, 15] = u_goal
    out[:, 16] = np.clip(ds_ref, -100, 100) / 50.0
    out[:, 17] = np.where(is_ev, 0.0, 1.0)            # delta_ego: 0 = EV, 1 = ICE
    out[:, 18] = g_next
    out[:, 19] = tau_clear / max(cfg.yellow_time + cfg.all_red_time, 1e-6)
    return out
