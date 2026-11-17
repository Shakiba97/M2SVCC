"""S_upper (Sec. 3.1), normalised to O(1).

Difference from the methodology's vector: n_ped is per served crossing (4 values) so the
policy can tell which phase would serve the waiting pedestrians; n_bike = 0 (no cyclists).
"""
import numpy as np

from ..common.encoding import one_hot


def upper_obs(cfg, plan, *, phase_k, g_elapsed, q, w, v, ped_counts, t_ped_elapsed, n_bike, t_cycle,
              weights=None):
    parts = [
        one_hot(phase_k),
        [g_elapsed / 60.0],
        np.asarray(q) / 20.0,
        np.asarray(w) / 60.0,
        np.asarray(v) / cfg.v_max,
        [ped_counts.get(c, 0) / 10.0 for c in plan.served_crossings],
        [t_ped_elapsed / 30.0],
        [n_bike / 10.0],
        [min(t_cycle, 240.0) / 120.0],
    ]
    if cfg.preference_conditioned:
        parts.append(weights if weights is not None else [cfg.alpha, cfg.beta, cfg.gamma_bike])
    return np.concatenate([np.asarray(p, np.float32).ravel() for p in parts])


def upper_dim(cfg, plan):
    return 8 + 1 + 3 * len(plan.lane_ids) + len(plan.served_crossings) + 3 + (3 if cfg.preference_conditioned else 0)
