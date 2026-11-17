"""Upper-level decisions inside the HAC loop: S_upper, phase selection, and the upper
transition (reward, B_upper storage, updates) at the end of each option.

UpperDecisionMixin is mixed into HACController (hac_loop.py) and uses its attributes:
cfg, plan, upper, tracker, option, weights, training, episode, stats, losses, decisions,
_last_cost, _lookahead (from WorldModelPlanner), and for the MPC warm start teacher,
_network_state, _choice and demo_rows.
"""
import random

import numpy as np

from .state import upper_obs


class UpperDecisionMixin:
    def _upper_state(self, tracker, q, w, v, ped_counts):
        return upper_obs(self.cfg, self.plan, phase_k=tracker.k, g_elapsed=tracker.g_elapsed, q=q, w=w, v=v,
                         ped_counts=ped_counts, t_ped_elapsed=tracker.ped_elapsed if tracker.ped_active else 0.0,
                         n_bike=0, t_cycle=tracker.t_cycle, weights=self.weights)

    def _select_phase(self, s_up, mask, obs, ped_counts, ped_delays):
        """Phase k of the next option. Sets self._choice = (teacher_k, source) for the transition."""
        legal = np.flatnonzero(mask)
        if len(legal) == 1:
            self.stats["masked_decisions"] += 1
            self._choice = (-1, "forced")
            return int(legal[0])
        if self.teacher is not None:        # recording MPC demonstrations (record_mpc_upper.py)
            k, teacher_k, source = self.teacher.choose(self._network_state, self.tracker.k, mask)
            self._choice = (teacher_k, source)
            return k
        if self.training and not self.cfg.upper_init and self.episode < self.cfg.upper_random_episodes:
            self._choice = (-1, "random")
            return int(np.random.choice(legal))
        self._choice = (-1, "policy")
        if self.cfg.upper_lookahead:
            if self.training and random.random() < self.cfg.lookahead_epsilon:
                return int(np.random.choice(legal))
            return self._lookahead(obs, ped_counts, ped_delays, mask)
        return self.upper.act(s_up, mask, deterministic=not self.training)

    def _finish_option(self, s_next, mask_next, done):
        opt = self.option
        if opt is None:
            return
        d = opt.duration(self.cfg.dt)
        if self.cfg.upper_reward_mode == "end_of_window":
            veh, ped, bike = self._last_cost[0] * d, self._last_cost[1] * d, self._last_cost[2] * d
        else:
            veh, ped, bike = opt.veh_int, opt.ped_int, opt.bike_int
        a, b, g = self.weights
        r = -self.cfg.upper_reward_scale * (a * veh + b * ped + g * bike) - self.cfg.lambda_cbf * opt.corr2
        self.stats["r_upper_sum"] += r
        self.stats["upper_decisions"] += 1
        if self.demo_rows is not None:
            self.demo_rows.append({"s": np.asarray(opt.s_upper, np.float32), "a": int(opt.k), "r": float(r),
                                   "s2": np.asarray(s_next, np.float32), "done": float(done), "duration": float(d),
                                   "mask": np.asarray(opt.mask, np.float32),
                                   "mask2": np.asarray(mask_next, np.float32),
                                   "teacher_k": int(opt.teacher_k), "source": opt.source})
        if self.training:
            self.upper.buffer.add_batch(s=[opt.s_upper], a=[opt.k], r=[r], s2=[s_next], done=[float(done)],
                                        duration=[d], mask=[opt.mask], mask2=[mask_next])
            self.decisions += 1
            if (self.decisions % self.cfg.upper_update_every == 0
                    and self.upper.buffer.size >= max(self.cfg.batch_upper, self.cfg.upper_warmup_transitions)):
                # After an MPC warm start the actor stays as pre-trained while the critic
                # adapts to the agent's own transitions.
                frozen = bool(self.cfg.upper_init) and self.episode < self.cfg.upper_actor_freeze_episodes
                for _ in range(self.cfg.upper_updates_per):
                    self.losses.update(self.upper.update(self.cfg.batch_upper, update_actor=not frozen))
