"""World model used between the levels at each upper decision (Sec. 4.1, 4.3, 4.5, 7.3).

Builds the lane batch from SUMO observations, the per-road-user signal timeline of the
selected and predicted phases, pedestrian yield points and predicted green starts; rolls
the lanes out; and returns each CAV's window-end target s_ref. Also predicts phases for
windows 2..H_w and runs the optional upper lookahead.

WorldModelPlanner is mixed into HACController (hac_loop.py) and uses its attributes:
cfg, plan, wm, upper, tracker, weights, training, episode, n_window, n_yellow,
n_all_red, and _upper_state (from UpperDecisionMixin).
"""
import copy

import numpy as np
import torch

from ..upper.signal_plan import GREEN
from .rollout import BIG, TYPE_CAV, TYPE_HDV, LaneState


class WorldModelPlanner:
    @property
    def phase_prediction(self):
        if self.cfg.phase_prediction == "policy" and (not self.training or
                                                      self.episode >= self.cfg.phase_prediction_warmup_episodes):
            return "policy"
        return "hold"

    def _lane_batch(self, obs):
        rows = []
        for j in range(len(self.plan.lane_ids)):
            sel = np.where(obs["lane"] == j)[0] if len(obs["ids"]) else np.zeros(0, int)
            before = sel[obs["x"][sel] < 0]
            crossed = sel[obs["x"][sel] >= 0]
            order = list(before[np.argsort(-obs["x"][before])])
            if crossed.size:
                order = [int(crossed[np.argmin(obs["x"][crossed])])] + order
            rows.append(order)
        B, N = len(rows), max(1, max(len(r) for r in rows))
        x = np.full((B, N), -BIG, np.float32)
        v, L, wait = np.zeros((B, N), np.float32), np.zeros((B, N), np.float32), np.zeros((B, N), np.float32)
        vt = np.full((B, N), TYPE_HDV, np.int64)
        valid = np.zeros((B, N), bool)
        links = np.full((B, N), -1, np.int64)
        where = {}
        for b, order in enumerate(rows):
            for n, i in enumerate(order):
                x[b, n], v[b, n], L[b, n], wait[b, n] = obs["x"][i], obs["v"][i], obs["L"][i], obs["wait"][i]
                vt[b, n] = TYPE_CAV if obs["is_cav"][i] else TYPE_HDV
                valid[b, n] = True
                links[b, n] = obs["link"][i] if obs["x"][i] < 0 else -1
                where[obs["ids"][i]] = (b, n)
        t = torch.from_numpy
        return LaneState(t(x), t(v), t(L), t(vt), t(valid), t(wait)), links, where

    def _codes(self, timeline, links):
        """Signal code of every road user at every step of a state-index timeline -> [T, B, N]."""
        table = np.stack([self.plan.state_codes[self.plan.all_states[i]] for i in timeline])   # [T, n_links]
        safe = np.where(links >= 0, links, 0)
        codes = table[:, safe]
        codes[:, links < 0] = GREEN
        return torch.from_numpy(codes)

    def _yield(self, timeline, links, codes, ped_counts):
        """Virtual stop at the conflict crossing for permitted turns while pedestrians use it."""
        T = len(timeline)
        ya = np.zeros((T,) + links.shape, bool)
        yx = np.full(links.shape, BIG, np.float32)
        for (b, n), lk in np.ndenumerate(links):
            c = self.plan.link_conflict_crossing.get(int(lk)) if lk >= 0 else None
            if c is None or ped_counts.get(c, 0) == 0:
                continue
            yx[b, n] = self.plan.link_via_length.get(int(lk), 0.0)
            cl = self.plan.crossing_link[c]
            for t, idx in enumerate(timeline):
                ya[t, b, n] = self.plan.all_states[idx][cl] in "Gg" and codes[t, b, n] == GREEN
        return torch.from_numpy(ya), torch.from_numpy(yx)

    def _green_start(self, codes, st):
        T = codes.shape[0]
        green = codes == GREEN
        first = torch.where(green.any(0), green.float().argmax(0).float(), torch.full(st.x.shape, float(T)))
        gs = first * self.cfg.dt
        none = ~green.any(0)
        return torch.where(none, torch.full_like(gs, T * self.cfg.dt + self.cfg.window), gs)

    def _predicted_upper(self, st, tracker, ped_counts):
        before = st.valid & (st.x < 0)
        n = before.sum(1).clamp(min=1).float()
        q = (before & (st.v < 0.1)).sum(1).float().numpy()
        w = (torch.where(before, st.wait, torch.zeros_like(st.wait)).sum(1) / n).numpy()
        v = torch.where(before.any(1), torch.where(before, st.v, torch.zeros_like(st.v)).sum(1) / n,
                        torch.full((st.x.shape[0],), self.cfg.v_max)).numpy()
        return self._upper_state(tracker, q, w, v, ped_counts)

    def _advance_prediction(self, tracker, ped_counts, k, schedule, n_clear):
        tr = copy.copy(tracker)
        tr.begin_green(k, ped_counts)
        for p in range(len(schedule)):
            tr.advance(self.cfg.dt, p >= n_clear)
        served = {c: (0 if self.plan.crossing_green(k, c) else n) for c, n in ped_counts.items()}
        return tr, served

    @torch.no_grad()
    def _plan_sequence(self, st, links, ped_counts, k1, sched1, n_clear1):
        """Phases for windows 1..H_w: k1, then hold or the upper SAC applied to the rolled-out state."""
        seq = [(k1, sched1, n_clear1)]
        if self.cfg.H_w <= 1:
            return seq
        tracker, peds = self._advance_prediction(self.tracker, ped_counts, k1, sched1, n_clear1)
        state, prev = st, k1
        for _ in range(2, self.cfg.H_w + 1):
            if self.phase_prediction == "hold":
                k = prev
            else:
                timeline = seq[-1][1]
                codes = self._codes(timeline, links)
                ya, yx = self._yield(timeline, links, codes.numpy(), peds)
                _, _, state = self.wm.rollout(state, codes, ya, yx, self._green_start(codes, state), use_glosa=False)
                k = self.upper.act(self._predicted_upper(state, tracker, peds), tracker.mask(), deterministic=True)
            sched, n_clear = self.plan.option_schedule(prev, k, self.n_window, self.n_yellow, self.n_all_red)
            seq.append((k, sched, n_clear))
            tracker, peds = self._advance_prediction(tracker, peds, k, sched, n_clear)
            prev = k
        return seq

    @torch.no_grad()
    def compute_targets(self, obs, ped_counts, k1, sched1, n_clear1, zone):
        """s_ref for every zone CAV before the stop line: its predicted position at option end."""
        targets = {}
        cav_before = [i for i in zone if obs["x"][i] < 0]
        if not cav_before:
            return targets
        st, links, where = self._lane_batch(obs)
        seq = self._plan_sequence(st, links, ped_counts, k1, sched1, n_clear1)
        timeline = [idx for _, sched, _ in seq for idx in sched]
        codes = self._codes(timeline, links)
        ya, yx = self._yield(timeline, links, codes.numpy(), ped_counts)
        xs, _, _ = self.wm.rollout(st, codes, ya, yx, self._green_start(codes, st), use_glosa=True)
        T1 = len(sched1)
        for i in cav_before:
            b, n = where[obs["ids"][i]]
            targets[obs["ids"][i]] = float(obs["odo"][i] + (xs[T1, b, n] - st.x[b, n]))
        return targets

    @torch.no_grad()
    def _lookahead(self, obs, ped_counts, ped_delays, mask):
        """Sec. 4.5: score each legal phase by predicted r_upper + discounted upper value."""
        st, links, _ = self._lane_batch(obs)
        best, best_score = None, -np.inf
        for k in np.flatnonzero(mask):
            sched, n_clear = self.plan.option_schedule(self.tracker.k, int(k), self.n_window, self.n_yellow, self.n_all_red)
            codes = self._codes(sched, links)
            ya, yx = self._yield(sched, links, codes.numpy(), ped_counts)
            _, _, end = self.wm.rollout(st, codes, ya, yx, self._green_start(codes, st), use_glosa=True)
            tr, peds = self._advance_prediction(self.tracker, ped_counts, int(k), sched, n_clear)
            s_next = self._predicted_upper(end, tr, peds)
            d = len(sched) * self.cfg.dt
            before = end.valid & (end.x < 0)
            n = before.sum(1).clamp(min=1).float()
            q = (before & (end.v < 0.1)).sum(1).float()
            w = torch.where(before, end.wait, torch.zeros_like(end.wait)).sum(1) / n
            veh = float((q * w).sum()) * d
            ped = sum(peds[c] * (ped_delays.get(c, 0.0) + d) for c in peds) * d
            r = -self.cfg.upper_reward_scale * (self.weights[0] * veh + self.weights[1] * ped)
            score = r + self.cfg.gamma_up ** (d / self.cfg.window) * float(self.upper.value(s_next, tr.mask())[0])
            if score > best_score:
                best, best_score = int(k), score
        return best
