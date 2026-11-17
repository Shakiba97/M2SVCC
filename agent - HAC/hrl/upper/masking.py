"""Phase timers and the upper action mask: minimum pedestrian green G_p and minimum green (Sec. 5.3)."""
import numpy as np

from .signal_plan import NUM_PHASES


class PhaseTracker:
    """g_elapsed, the pedestrian-green timer and t_cycle, plus the action mask (Sec. 5.3)."""

    def __init__(self, cfg, plan):
        self.cfg, self.plan = cfg, plan
        self.k, self.g_elapsed, self.gp, self.ped_elapsed, self.t_cycle = 0, 0.0, 0.0, 0.0, 0.0
        self.started = False

    def begin_green(self, k, ped_counts):
        if k != self.k or not self.started:
            self.k, self.g_elapsed, self.ped_elapsed, self.started = k, 0.0, 0.0, True
            self.gp = self.plan.min_ped_green(k, ped_counts, self.cfg.ped_speed)
            if k == 0:
                self.t_cycle = 0.0

    def advance(self, dt, green):
        self.t_cycle += dt
        if green:
            self.g_elapsed += dt
            self.ped_elapsed += dt

    @property
    def ped_active(self):
        return self.gp > 0

    def mask(self):
        m = np.ones(NUM_PHASES, np.float32)
        if (self.ped_active and self.ped_elapsed < self.gp) or self.g_elapsed < self.cfg.min_green:
            m[:] = 0.0
            m[self.k] = 1.0
        return m
