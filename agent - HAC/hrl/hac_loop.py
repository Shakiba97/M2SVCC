"""Phase A of Sec. 6.2: one SUMO episode driven by the upper and lower SACs.

Timeline of one upper decision (an "option" in the Options-Framework sense):

    decision -> [yellow 5 s + all-red 2 s, only if the phase changes] -> 5 s phase-pure window

The upper transition spans the whole option and is discounted by its duration (SMDP).
The window-end target s_ref is the world model's prediction for the end of the option.

Each SUMO step (HACController.step):
  (1) per-step lower rewards of last step's transitions
  (2) at an option boundary: window-end targets, upper transition, new decision (upper/decision.py),
      new s_ref from the world model (world_model/planner.py)
      then the step's signal is applied before anything reacts to it
  (3) S_lower for every CAV in the control zone
  (4) store last step's lower transitions
  (5) lower SAC -> CBF-QP (safety/cbf.py) -> SUMO
  (6)-(7) phase timers and upper costs, (8) B_model frame, (9) SAC updates
"""
import numpy as np

from .lower.reward import goal_potential, lower_step_reward, window_goal_reward
from .lower.state import lower_obs
from .safety.cbf import CBFParams, can_stop, cbf_filter
from .upper.decision import UpperDecisionMixin
from .upper.masking import PhaseTracker
from .upper.option import Option
from .upper.reward import upper_cost_terms
from .upper.signal_plan import GREEN, RED, YELLOW, char_code
from .world_model.planner import WorldModelPlanner


class HACController(UpperDecisionMixin, WorldModelPlanner):
    def __init__(self, cfg, plan, lower, upper, world_model, training=True, episode=0, model_buffer=None,
                 teacher=None, record_upper=False):
        self.cfg, self.plan = cfg, plan
        self.lower, self.upper, self.wm = lower, upper, world_model
        self.training, self.episode = training, episode
        self.model_buffer = model_buffer
        # teacher: picks the phase instead of the upper SAC (upper/mpc_teacher.py).
        # record_upper: keep every upper transition in demo_rows (with how its phase was chosen).
        self.teacher = teacher
        self.demo_rows = [] if record_upper else None
        self._choice = (-1, "policy")
        self._network_state = None
        self.cbf = CBFParams.from_config(cfg)
        self.n_window = int(round(cfg.window / cfg.dt))
        self.n_yellow = int(round(cfg.yellow_time / cfg.dt))
        self.n_all_red = int(round(cfg.all_red_time / cfg.dt))
        self.tracker = PhaseTracker(cfg, plan)
        self.option = None
        self.pending = {}            # vid -> data of the lower transition started last step
        self.committed = set()       # CAVs that could not stop at yellow onset (h_red inactive)
        self.weights = (cfg.alpha, cfg.beta, cfg.gamma_bike)
        if cfg.preference_conditioned and training:
            w = np.random.dirichlet([1.0, 1.0])
            self.weights = (float(w[0]), float(w[1]), cfg.gamma_bike)
        self.step_count = 0
        self.decisions = 0
        self.stats = {k: 0.0 for k in (
            "lower_steps", "cbf_interventions", "cbf_infeasible", "red_infeasible", "red_light_violations",
            "sumo_overrides", "min_gap", "r_lower_sum", "r_upper_sum", "upper_decisions", "phase_changes",
            "goal_shortfall_sum", "goal_count", "masked_decisions", "gap_below_d0_new_leader",
            "gap_below_d0_same_leader", "sumo_overrides_stopline", "sumo_overrides_junction",
            "sumo_overrides_other")}
        self.stats["min_gap"] = np.inf
        self.losses = {}
        self.violation_log = []
        self._prev_state_string = None
        self._prev_leader = {}

    def _zone(self, obs):
        """Indices of RL-controlled CAVs: within control_range before the line, or crossed (< L_exit)."""
        x = obs["x"]
        return [i for i in range(len(obs["ids"])) if obs["is_cav"][i] and (x[i] >= 0 or -x[i] <= self.cfg.control_range)]

    def step(self, env, network_state):
        cfg, dt = self.cfg, self.cfg.dt
        self._network_state = network_state
        obs = env.observe()
        ped_counts, ped_delays = env.pedestrian_stats(network_state)
        q, w, vlane = env.lane_stats(obs)
        zone = self._zone(obs)
        zone_ids = [obs["ids"][i] for i in zone]
        has, gap, vl, al = env.leaders(obs, zone_ids)
        lead = {vid: (has[n], gap[n], vl[n], al[n]) for n, vid in enumerate(zone_ids)}
        if len(gap) and has.any():
            self.stats["min_gap"] = min(self.stats["min_gap"], float(gap[has].min()))
        # Gaps below d_0: a new leader this step (lane-change cut-in, or a merge after the
        # junction) is outside what a longitudinal barrier can prevent; same-leader events are not.
        leader_ids = dict(zip(zone_ids, env.last_leader_ids))
        for n_, vid in enumerate(zone_ids):
            if has[n_] and gap[n_] < cfg.d_0 - 1e-6:
                key = "gap_below_d0_new_leader" if self._prev_leader.get(vid) != leader_ids[vid] else "gap_below_d0_same_leader"
                self.stats[key] += 1
        self._prev_leader = leader_ids

        # (1) per-step reward of last step's transitions
        rewards, terminal = {}, {}
        for vid, p in self.pending.items():
            if vid in obs["index"] and vid in lead:
                i = obs["index"][vid]
                odo, v_next, x_next = obs["odo"][i], obs["v"][i], obs["x"][i]
                hl, g_next = lead[vid][0], lead[vid][1]
                terminal[vid] = False
            elif vid in obs["exited_kin"]:
                k = obs["exited_kin"][vid]
                odo, v_next, x_next, hl, g_next = k["odo"], k["v"], k["x"], False, np.inf
                terminal[vid] = True
            else:   # vanished: use the commanded kinematics
                a_s = max(p["a_star"], -p["v"] / dt)
                odo = p["odo"] + p["v"] * dt + 0.5 * a_s * dt * dt
                v_next, x_next, hl, g_next = max(0.0, p["v"] + a_s * dt), p["x"] + odo - p["odo"], False, np.inf
                terminal[vid] = True
            a_meas = (v_next - p["v"]) / dt
            if abs(a_meas - p["a_star"]) > 0.3 and a_meas < p["a_star"]:
                self.stats["sumo_overrides"] += 1
                if p["red_active"] and -p["x"] < 5.0:
                    self.stats["sumo_overrides_stopline"] += 1
                elif p["x"] >= 0 or p["permitted"]:
                    self.stats["sumo_overrides_junction"] += 1      # yielding inside the junction / permitted turn
                else:
                    self.stats["sumo_overrides_other"] += 1
            if p["x"] < 0 <= x_next and p["red"] and vid not in env.teleported:
                self.stats["red_light_violations"] += 1
                if len(self.violation_log) < 50:
                    self.violation_log.append({"vid": vid, "link": p["link"], "state": p["state"],
                                               "prev_state": p["prev_state"], "d_stop": -p["x"], "v": p["v"],
                                               "a_raw": p["a_raw"], "a_star": p["a_star"], "red_active": p["red_active"]})
            r, _ = lower_step_reward(cfg, is_ev=np.array([p["is_ev"]]), v=np.array([p["v"]]), a=np.array([a_meas]),
                                     dist=np.array([odo - p["odo"]]), gated=np.array([p["gated"]]),
                                     a_raw=np.array([p["a_raw"]]), a_star=np.array([p["a_star"]]),
                                     gap_next=np.array([g_next]), v_next=np.array([v_next]),
                                     has_lead_next=np.array([hl]))
            rewards[vid] = float(r[0])
            p["end"] = (odo, v_next, hl, g_next)

        # (2) option boundary: window-end targets, upper transition, new decision
        if self.option is None or self.option.done:
            if self.option is not None:
                for vid, s_ref in self.option.targets.items():
                    if vid in self.pending:
                        odo, v_end, hl, g_end = self.pending[vid]["end"]
                        rg, short = window_goal_reward(cfg, s_ref=np.array([s_ref]), s_ego=np.array([odo]),
                                                       gap=np.array([g_end]), v_ego=np.array([v_end]),
                                                       has_lead=np.array([hl]))
                        rewards[vid] += float(rg[0])
                        self.stats["goal_shortfall_sum"] += float(short[0])
                        self.stats["goal_count"] += 1
            s_up = self._upper_state(self.tracker, q, w, vlane, ped_counts)
            mask = self.tracker.mask()
            self._finish_option(s_up, mask, done=False)
            k = self._select_phase(s_up, mask, obs, ped_counts, ped_delays)
            sched, n_clear = self.plan.option_schedule(self.tracker.k, k, self.n_window, self.n_yellow, self.n_all_red)
            if n_clear:
                self.stats["phase_changes"] += 1
            self.option = Option(k=k, schedule=sched, n_clear=n_clear, s_upper=s_up, mask=mask,
                                 teacher_k=self._choice[0], source=self._choice[1])
            self.option.targets = self.compute_targets(obs, ped_counts, k, sched, n_clear, zone)
        opt = self.option

        # Apply this step's signal before anything reacts to it, and use the state in force
        # during the coming step for the CBF, the gating m_t, the violation check and B_model.
        idx = opt.schedule[opt.ptr]
        env.set_phase(idx)
        state_now = self.plan.all_states[idx]
        obs["code"] = np.array([char_code(state_now[lk]) if lk >= 0 else GREEN for lk in obs["link"]], np.int64)
        obs["state_string"] = state_now

        # (3) lower states for zone CAVs
        n = len(zone)
        x = obs["x"][zone]
        v = obs["v"][zone]
        code = obs["code"][zone]
        links = obs["link"][zone]
        crossed = x >= 0
        d_stop = -x
        stoppable = can_stop(d_stop, v, self.cbf)
        for m, vid in enumerate(zone_ids):
            if crossed[m]:
                self.committed.discard(vid)
            elif code[m] == YELLOW and not stoppable[m]:
                self.committed.add(vid)
        committed = np.array([vid in self.committed for vid in zone_ids], bool)
        gated = crossed | (code == GREEN) | ((code == YELLOW) & committed)
        red_active = (~crossed) & ~committed & ((code == RED) | ((code == YELLOW) & stoppable))
        tau_win = (len(opt.schedule) - opt.ptr) * dt
        tau_clear = max(0, opt.n_clear - opt.ptr) * dt
        g_next = np.array([crossed[m] or self.plan.link_green_in_phase(opt.k, int(links[m])) for m in range(n)], float)
        u_goal = np.array([vid in opt.targets for vid in zone_ids], float)
        ds_ref = np.array([opt.targets[vid] - obs["odo"][i] if vid in opt.targets else 0.0
                           for vid, i in zip(zone_ids, zone)])
        is_ev = obs["is_ev"][zone]
        hl = np.array([lead[vid][0] for vid in zone_ids], bool)
        gp = np.array([lead[vid][1] for vid in zone_ids], float)
        vlp = np.array([lead[vid][2] for vid in zone_ids], float)
        alp = np.array([lead[vid][3] for vid in zone_ids], float)
        vec = lower_obs(cfg, d_stop=d_stop, v=v, gap=gp, has_lead=hl, v_lead=vlp, a_lead=alp, phase_k=opt.k,
                        tau_win=tau_win, g_lane=gated.astype(float), u_goal=u_goal, ds_ref=ds_ref, is_ev=is_ev,
                        g_next=g_next, tau_clear=tau_clear)
        phi = goal_potential(cfg, u_goal=u_goal, ds_ref=ds_ref, v_ego=v, tau_win=tau_win) if n else np.zeros(0)

        # (4) store last step's transitions
        if self.training and self.pending:
            S, A, R, S2, D = [], [], [], [], []
            for vid, p in self.pending.items():
                r = rewards[vid]
                done = terminal[vid] or vid not in zone_ids
                if cfg.potential_shaping:
                    phi2 = 0.0 if done else phi[zone_ids.index(vid)]
                    r += cfg.gamma_lo * phi2 - p["phi"]
                S.append(p["vec"]); A.append([p["a_raw"]]); R.append(r)
                S2.append(p["vec"] if done else vec[zone_ids.index(vid)]); D.append(float(done))
            self.lower.buffer.add_batch(s=S, a=A, r=R, s2=S2, done=D)
        self.stats["r_lower_sum"] += sum(rewards.values())
        for vid in obs["exited"]:
            env.release(vid)
            self.committed.discard(vid)

        # (5) act: lower SAC -> CBF-QP -> SUMO
        self.pending = {}
        if n:
            if self.training and self.lower.buffer.size < cfg.lower_warmup_transitions:
                a_raw = self.lower.random_action(n)
            else:
                a_raw = self.lower.act(vec, deterministic=not self.training)
            a_star, info = cbf_filter(a_raw, v, d_stop, red_active, hl, gp, vlp, alp, self.cbf)
            env.command_accelerations(zone_ids, a_star, v)
            opt.corr2 += float((info["correction"] ** 2).sum())
            self.stats["lower_steps"] += n
            self.stats["cbf_interventions"] += int(info["intervened"].sum())
            self.stats["cbf_infeasible"] += int(info["infeasible"].sum())
            self.stats["red_infeasible"] += int(info["red_infeasible"].sum())
            for m, vid in enumerate(zone_ids):
                i = zone[m]
                self.pending[vid] = {"vec": vec[m], "a_raw": float(a_raw[m]), "a_star": float(a_star[m]),
                                     "v": float(v[m]), "odo": float(obs["odo"][i]), "x": float(x[m]),
                                     "gated": bool(gated[m]), "is_ev": bool(is_ev[m]),
                                     "red": bool(code[m] == RED and not crossed[m] and not committed[m]),
                                     "phi": float(phi[m]), "link": int(links[m]), "red_active": bool(red_active[m]),
                                     "permitted": bool(links[m] >= 0 and state_now[int(links[m])] == "g"),
                                     "state": obs["state_string"], "prev_state": self._prev_state_string}

        self._prev_state_string = obs["state_string"]

        # (6) phase timers for this step (the signal itself was applied above)
        if opt.ptr == opt.n_clear:
            self.tracker.begin_green(opt.k, ped_counts)
        self.tracker.advance(dt, opt.ptr >= opt.n_clear)
        opt.ptr += 1

        # (7) upper cost accumulated over the option
        veh, ped, bike = upper_cost_terms(q, w, [ped_counts[c] for c in self.plan.crossing_ids],
                                          [ped_delays[c] for c in self.plan.crossing_ids])
        self._last_cost = (veh, ped, bike)
        opt.veh_int += veh * dt
        opt.ped_int += ped * dt
        opt.bike_int += bike * dt

        # (8) B_model frame
        if self.model_buffer is not None:
            self.model_buffer.add_frame(obs["ids"], obs["lane"], obs["x"], obs["v"], obs["L"], env.frame_types(obs),
                                        obs["code"], obs["wait"])

        # (9) updates (Phase B, interleaved)
        self.step_count += 1
        if (self.training and self.step_count % cfg.lower_update_every == 0
                and self.lower.buffer.size >= max(cfg.batch_lower, cfg.lower_warmup_transitions)):
            for _ in range(cfg.lower_updates_per):
                self.losses.update(self.lower.update(cfg.batch_lower))

    def finish(self, env):
        """End of simulation. Open transitions are truncations: no next state was observed, so
        they are dropped rather than stored as terminal (every stored transition bootstraps
        unless the CAV actually left the control zone)."""
        self.pending = {}
        s = dict(self.stats)
        s["cbf_intervention_rate"] = s["cbf_interventions"] / max(s["lower_steps"], 1)
        s["mean_goal_shortfall"] = s["goal_shortfall_sum"] / max(s["goal_count"], 1)
        s["collisions"] = env.stats["collisions"]
        s["teleports"] = env.stats["teleports"]
        s.update(self.losses)
        s["violation_log"] = self.violation_log
        return s
