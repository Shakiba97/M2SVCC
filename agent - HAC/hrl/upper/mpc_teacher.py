"""MPC-MINLP phase teacher for the MPC warm start of the upper level.

At an upper decision the repo's MPC (agent/mpc_agent.py) solves only its slower-scale
problem, the signal plan over its 30 s horizon. The per-lane CAV problems are skipped:
the lower level is not taken from the MPC. The plan is a sequence of SUMO phase indices
with -1 for a clearance step, and the phase it is heading for is its first entry that is
not -1:

    [k, k, ...]        hold k (k = current phase)
    [-1, k2, ...]      switch to k2 (HAC inserts the yellow + all-red itself)

SUMO indices 0..7 are the HAC phases k (upper/signal_plan.py), so no mapping is needed.
The teacher is only consulted when more than one phase is legal (HACController asks it
after the G_p / min-green mask), and the MPC objective uses the same (alpha, beta) as r_upper.

Needs the GAMS Python API (import gams) and GAMS in /opt/gams (the path MpcAgent uses).
"""
import contextlib
import io
import sys
import time

import numpy as np

from .signal_plan import NUM_PHASES


def mpc_plan_to_phase(plan):
    """Green phase an MPC plan is heading for: its first entry that is not a clearance (-1)."""
    for k in plan:
        if k != -1:
            return int(k)
    return None


class MpcPhaseTeacher:
    """Picks the phase with the MPC; with probability epsilon applies a random legal phase
    instead (the MPC's own choice is still recorded as teacher_k for behaviour cloning)."""

    def __init__(self, paras, env, cfg, epsilon=0.0, seed=0):
        if cfg.preference_conditioned:
            raise ValueError("the MPC teacher uses fixed (alpha, beta); set preference_conditioned=False")
        try:
            from agent.mpc_agent import MpcAgent
        except ImportError as e:
            raise ImportError("the MPC teacher needs the GAMS Python API (import gams): install gamsapi in this "
                              "environment, matching the GAMS version in /opt/gams") from e
        # Same vehicle / pedestrian weights as r_upper (also what the MPC results files are named by).
        paras["weight(Vehicles/Pedestrians)"] = (cfg.alpha, cfg.beta)
        with contextlib.redirect_stdout(io.StringIO()):
            self.mpc = MpcAgent(paras, "unified_four_legs_three_lanes")
            self.mpc.clear_redundant_gams_files()
        self.paras, self.tls = paras, env.tls
        self.epsilon = epsilon
        self.rng = np.random.default_rng(seed)
        self.counts = {"mpc": 0, "noise": 0, "fallback": 0}
        self.n_solves = self.n_failed = self.n_invalid = 0
        self.solve_seconds = 0.0
        self.first_error = None

    def _solve(self, network_state):
        """The MPC's phase, or None (no vehicles, or the solve failed)."""
        state = network_state[self.tls]
        if state["num_vehicles_max"] <= 0:          # the MPC itself does not solve without vehicles
            return None
        t0 = time.time()
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                self.mpc.gather_static_paras_single_intersection(self.tls, self.paras)
                self.mpc.solve_slower_scale_problem_single_intersection(self.tls, state)
        except Exception as e:  # noqa: BLE001  (GAMS raises several exception types)
            self.n_failed += 1
            if self.first_error is None:
                self.first_error = f"{type(e).__name__}: {e}"
                print(f"MPC teacher: solve failed ({self.first_error}); holding the current phase",
                      file=sys.stderr)
            return None
        finally:
            self.solve_seconds += time.time() - t0
        self.n_solves += 1
        return mpc_plan_to_phase(self.mpc.slower_scale_solution["following_phases"])

    def choose(self, network_state, current_k, mask):
        """Returns (phase to apply, the MPC's legal phase or -1, source: mpc / noise / fallback)."""
        legal = np.flatnonzero(mask)
        teacher_k = self._solve(network_state)
        if teacher_k is not None and not (0 <= teacher_k < NUM_PHASES and mask[teacher_k] > 0):
            self.n_invalid += 1                      # e.g. the exclusive pedestrian phase (index 8)
            teacher_k = None
        if teacher_k is None:
            k, source = (current_k if mask[current_k] > 0 else int(legal[0])), "fallback"
        else:
            k, source = teacher_k, "mpc"
        if self.epsilon > 0 and self.rng.random() < self.epsilon:
            k, source = int(self.rng.choice(legal)), "noise"
        self.counts[source] += 1
        return int(k), (-1 if teacher_k is None else int(teacher_k)), source

    def summary(self):
        n = max(self.n_solves + self.n_failed, 1)
        return {"teacher_mpc": self.counts["mpc"], "teacher_noise": self.counts["noise"],
                "teacher_fallback": self.counts["fallback"], "teacher_solves": self.n_solves,
                "teacher_failed": self.n_failed, "teacher_invalid": self.n_invalid,
                "teacher_solve_s": round(self.solve_seconds / n, 2)}

    def close(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.mpc.clear_redundant_gams_files()
