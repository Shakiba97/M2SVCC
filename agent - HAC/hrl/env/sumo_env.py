"""SUMO interface for the HAC agent.

Subclasses the repo's SingleIntersection so routes, the network graph, pedestrian
demand estimates, conflict counting and the performance metrics are exactly the
ones used for the MPC and actuated runs. Differences:
  * routes (generated exactly as in main.py), tripinfo and queue outputs go to the run
    folder, so HAC never reads or writes the shared network_model/single_intersection.rou.xml
    that main.py uses (a baseline and a HAC run can share the machine);
  * optional ballistic position update (matches the CBF / world-model kinematics);
  * SUMO runs through libsumo unless HAC_SUMO_BACKEND=traci (hrl/common/paths.py); the first
    start() of a process prints the SUMO version and warns if it is not cfg.sumo_version;
  * close() only closes this run's SUMO;
  * per-vehicle tracking in odometer coordinates: x = position relative to the stop line,
    kept after the vehicle crosses so the lower level can run to L_exit.
"""
import os

import numpy as np
import traci
from sumolib import checkBinary

from ..common import paths  # noqa: F401  (sys.path setup)
from environment.single_intersection import SingleIntersection
from ..common.paths import NETWORK_DIR
from ..upper.signal_plan import GREEN, SignalPlan, char_code
from ..world_model.rollout import TYPE_CAV, TYPE_HDV

PROGRAM_CFG = {
    "hrl": "single_intersection_multi_scale_Concurrent.sumocfg",
    "actuated": "single_intersection_actuated.sumocfg",
}


class HRLIntersection(SingleIntersection):
    _version_reported = False

    def __init__(self, paras, cfg, run_dir, label):
        self.sim_dir = os.path.join(run_dir, "sim")
        os.makedirs(self.sim_dir, exist_ok=True)
        self.route_file = os.path.join(self.sim_dir, f"routes_{label}.rou.xml")
        super().__init__(paras, route_file=self.route_file)
        if paras["ped_phasing"] != "Concurrent":
            raise ValueError("Phase I uses the 8-phase Concurrent program (set ped_phasing = 'Concurrent').")
        self.cfg, self.label = cfg, label
        self.tls = next(iter(self.network_graph))
        self.plan = SignalPlan(self.network_graph, self.tls)
        self.queue_file = os.path.join(self.sim_dir, f"queues_{label}.xml")
        self.trip_file = os.path.join(self.sim_dir, f"tripinfo_{label}.xml")
        self.tracked = {}            # vid -> {"lane", "s_stop", "link", "on_lane"}
        self.controlled = set()      # CAVs currently under RL speed control
        self.cav_ids = paras["cav_ids"]
        self.ev_ids = paras["veh_id_with_ev"]["cav_ev"] | paras["veh_id_with_ev"]["hdv_ev"]
        self.sim_step = 0
        self.stats = {"collisions": 0, "teleports": 0}
        self.teleported = set()      # SUMO teleports jump vehicles across the junction (not red running)
        self._current_index = None

    # ------------------------------------------------------------------ lifecycle
    def start(self, program="hrl"):
        cfg = self.cfg
        if cfg.sumo_gui and traci.isLibsumo():
            raise SystemExit("sumo_gui needs the TraCI backend: rerun with HAC_SUMO_BACKEND=traci")
        cmd = [checkBinary("sumo-gui" if cfg.sumo_gui else "sumo"),
               "-c", os.path.join(NETWORK_DIR, PROGRAM_CFG[program]),
               "--seed", str(self.paras["sumo_seed"]),
               "--route-files", self.route_file,    # replaces the .sumocfg's shared route file
               "--queue-output", self.queue_file, "--tripinfo-output", self.trip_file,
               "--step-length", str(cfg.dt), "--no-step-log", "true", "--verbose", "false",
               "--no-warnings", "true", "--start"]
        if cfg.ballistic:
            cmd += ["--step-method.ballistic", "true"]
        cmd += ["--time-to-teleport", str(cfg.time_to_teleport)]
        if program == "actuated":
            # The actuated tlLogic writes detector output to a file named "NULL" next to the
            # config (a tracked repo file); point it into this run's folder instead.
            with open(os.path.join(NETWORK_DIR, "single_intersection.add_actuated.xml")) as f:
                add = f.read().replace('value="NULL"', f'value="{os.path.join(self.sim_dir, "detectors.xml")}"')
            add_path = os.path.join(self.sim_dir, "actuated.add.xml")
            with open(add_path, "w") as f:
                f.write(add)
            cmd += ["--additional-files", add_path]
        if cfg.sumo_gui:
            cmd += ["--quit-on-end"]
        traci.start(cmd)
        if not HRLIntersection._version_reported:
            HRLIntersection._version_reported = True
            version = traci.getVersion()[1]
            print(f"{version} via {'libsumo' if traci.isLibsumo() else 'TraCI'}")
            if cfg.sumo_version and cfg.sumo_version not in version:
                print(f"WARNING: running {version}, but sumo_version = {cfg.sumo_version} "
                      "(the version the MPC/actuated baselines must also use)")
        if program == "actuated":
            traci.trafficlight.setProgram(self.tls, "actuated_program")
        else:
            self.set_phase(0)

    def close(self):
        traci.close()

    def set_phase(self, sumo_index):
        if sumo_index != self._current_index:
            traci.trafficlight.setPhase(self.tls, sumo_index)
            self._current_index = sumo_index
        # Hold indefinitely; the controller decides every step (the static program would
        # otherwise advance on its own durations).
        traci.trafficlight.setPhaseDuration(self.tls, 1e5)

    def simulation_step(self):
        traci.simulationStep()
        self.sim_step += 1
        self.stats["collisions"] += traci.simulation.getCollidingVehiclesNumber()
        tele = traci.simulation.getStartingTeleportIDList()
        self.stats["teleports"] += len(tele)
        self.teleported.update(tele)

    # ------------------------------------------------------------------ observation
    def observe(self):
        """Kinematics of every tracked road user (all approach lanes + crossed, until L_exit).

        Returns a dict of arrays plus 'ids', 'index', 'exited' (ids that left tracking this
        step) and 'exited_kin' (final odometer/speed of those that passed L_exit).
        """
        state = traci.trafficlight.getRedYellowGreenState(self.tls)
        on_lane = {}
        for j, lane in enumerate(self.plan.lane_ids):
            for vid in traci.lane.getLastStepVehicleIDs(lane):
                on_lane[vid] = j
        present = set(traci.vehicle.getIDList())
        ids, lane, x, v, acc, length, code, wait, link, odo = [], [], [], [], [], [], [], [], [], []
        exited, exited_kin = [], {}
        for vid in list(self.tracked) + [i for i in on_lane if i not in self.tracked]:
            if vid not in present:
                self.tracked.pop(vid, None)
                exited.append(vid)
                continue
            dist_driven = traci.vehicle.getDistance(vid)
            tr = self.tracked.setdefault(vid, {})
            if vid in on_lane:
                lnk, d_stop = -1, None
                for tls_id, lidx, d, _ in traci.vehicle.getNextTLS(vid):
                    if tls_id == self.tls:
                        lnk, d_stop = lidx, d
                        break
                if d_stop is None:
                    lane_id = self.plan.lane_ids[on_lane[vid]]
                    d_stop = self.plan.lane_length[lane_id] - traci.vehicle.getLanePosition(vid)
                tr.update(lane=on_lane[vid], s_stop=dist_driven + d_stop, link=lnk, on_lane=True)
            else:
                tr["on_lane"] = False
            xi = dist_driven - tr["s_stop"]
            if not tr["on_lane"] and xi > self.cfg.L_exit:
                self.tracked.pop(vid)
                exited.append(vid)
                exited_kin[vid] = {"odo": dist_driven, "v": traci.vehicle.getSpeed(vid), "x": xi}
                continue
            ids.append(vid)
            lane.append(tr["lane"])
            x.append(xi)
            v.append(traci.vehicle.getSpeed(vid))
            acc.append(traci.vehicle.getAcceleration(vid))
            length.append(traci.vehicle.getLength(vid))
            wait.append(traci.vehicle.getWaitingTime(vid))
            lk = tr["link"] if tr["on_lane"] else -1
            link.append(lk)
            code.append(char_code(state[lk]) if lk >= 0 else GREEN)
            odo.append(dist_driven)
        arr = lambda a, t=float: np.asarray(a, dtype=t)  # noqa: E731
        return {"ids": ids, "index": {vid: i for i, vid in enumerate(ids)}, "exited": exited,
                "exited_kin": exited_kin,
                "lane": arr(lane, np.int64), "x": arr(x), "v": arr(v), "a": arr(acc), "L": arr(length),
                "wait": arr(wait), "link": arr(link, np.int64), "code": arr(code, np.int64), "odo": arr(odo),
                "is_cav": np.array([vid in self.cav_ids for vid in ids], dtype=bool),
                "is_ev": np.array([vid in self.ev_ids for vid in ids], dtype=bool),
                "state_string": state}

    def leaders(self, obs, vids, lookahead=250.0):
        """Bumper gap, speed and last acceleration of each vehicle's leader."""
        n = len(vids)
        has, gap, vl, al = np.zeros(n, bool), np.full(n, np.inf), np.zeros(n), np.zeros(n)
        self.last_leader_ids = [None] * n
        for i, vid in enumerate(vids):
            res = traci.vehicle.getLeader(vid, lookahead)
            if not res or res[0] == "":
                continue
            lid, g = res
            self.last_leader_ids[i] = lid
            has[i] = True
            gap[i] = g + traci.vehicle.getMinGap(vid)   # SUMO's gap excludes the ego minGap
            if lid in obs["index"]:
                j = obs["index"][lid]
                vl[i], al[i] = obs["v"][j], obs["a"][j]
            else:
                vl[i], al[i] = traci.vehicle.getSpeed(lid), traci.vehicle.getAcceleration(lid)
        return has, gap, vl, al

    def lane_stats(self, obs):
        """q_j (halting vehicles), w_j (mean waiting time), v_j (mean speed) per approach lane."""
        J = len(self.plan.lane_ids)
        q, w, v = np.zeros(J), np.zeros(J), np.full(J, self.cfg.v_max)
        for j, lane in enumerate(self.plan.lane_ids):
            q[j] = traci.lane.getLastStepHaltingNumber(lane)
            v[j] = min(traci.lane.getLastStepMeanSpeed(lane), self.cfg.v_max)
            sel = (obs["lane"] == j) & (obs["x"] < 0) if len(obs["ids"]) else np.zeros(0, bool)
            if sel.any():
                w[j] = float(obs["wait"][sel].mean())
        return q, w, v

    def pedestrian_stats(self, network_state):
        """N_m waiting pedestrians and D_m their mean waiting time, per crossing."""
        demand = network_state[self.tls]["pedestrian_demand_current"]
        counts, delays = {}, {}
        for c in self.plan.crossing_ids:
            peds = list(demand.get(c, ()))
            counts[c] = len(peds)
            delays[c] = float(np.mean([traci.person.getWaitingTime(p) for p in peds])) if peds else 0.0
        return counts, delays

    # ------------------------------------------------------------------ control
    def command_accelerations(self, vids, a_star, v_now):
        for vid, a, v in zip(vids, a_star, v_now):
            if vid not in self.controlled:
                traci.vehicle.setSpeedMode(vid, self.cfg.cav_speed_mode)
                self.controlled.add(vid)
            traci.vehicle.setSpeed(vid, max(0.0, float(v + a * self.cfg.dt)))

    def release(self, vid):
        if vid in self.controlled:
            self.controlled.discard(vid)
            if vid in traci.vehicle.getIDList():
                traci.vehicle.setSpeed(vid, -1)
                traci.vehicle.setSpeedMode(vid, 31)

    def frame_types(self, obs):
        return np.array([TYPE_CAV if vid in self.controlled else TYPE_HDV for vid in obs["ids"]], dtype=np.int64)

    # ------------------------------------------------------------------ results
    def write_results(self, path, extra=None):
        """Same lines as the repo's performance_results (so the figure scripts can parse them)."""
        import matplotlib.pyplot as plt
        self.get_average_delay_endtime(self.trip_file)
        self.get_average_queue_length_endtime(self.queue_file)
        self.get_average_phase_duration([], [], "hrl")
        plt.close("all")
        ct = "hrl"
        p = self.paras
        n_ice_cav = max(len(p["veh_id_with_ev"]["cav_ice"]), 1)
        n_ice_hdv = max(len(p["veh_id_with_ev"]["hdv_ice"]), 1)
        n_ev_cav = max(len(p["veh_id_with_ev"]["cav_ev"]), 1)
        n_ev_hdv = max(len(p["veh_id_with_ev"]["hdv_ev"]), 1)
        lines = [
            f"average fuel consumption (external model) for CAVs {ct} (in mg): {self.fuel_total_cav_external_model / n_ice_cav}",
            f"average fuel consumption (external model) for HDVs {ct} (in mg): {self.fuel_total_hdv_external_model / n_ice_hdv}",
            f"average power consumption (external model) for CAVs {ct} (in Kw.s): {self.power_total_cav_external_model / n_ev_cav}",
            f"average power consumption (external model) for HDVs {ct} (in Kw.s): {self.power_total_hdv_external_model / n_ev_hdv}",
            f"average waiting time for {ct} scenario (in s): {self.waiting_time_avg}",
            f"average time loss for {ct} scenario (in s): {self.lost_time_avg}",
            f"average queue length for {ct} scenario (in m): {self.queue_avg}",
            f"average pedestrian time loss for {ct} scenario (in s): {self.lost_time_avg_ped}",
            f"number of right-turn conflicts between vehicles and pedestrians: {self.right_turn_conflicts_measure()}",
            f"number of CAVs passing through the specific intersection in {ct} scenario: {len(p['cav_ids'])}",
            f"number of pedestrians passing through the specific intersection in {ct} scenario: {len(p['ped_ids'])}",
            f"The time of simulation termination in {ct} scenario: {self.sim_step * self.cfg.dt}",
            f"average phase lengths:  {self.phase_avg}",
            f"number of times each phase happened: {self.phase_ntimes}",
        ]
        for k, val in (extra or {}).items():
            lines.append(f"{k}: {val}")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
