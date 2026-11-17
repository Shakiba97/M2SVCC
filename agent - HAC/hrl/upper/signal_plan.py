"""Signal-plan lookups for the Phase I 8-phase Concurrent program.

Everything is read from the same SUMO files the simulation loads, so the
phase -> lane (A, Eq. 5e) and phase -> crossing (B, Eq. 5f) lookups cannot drift
from the network:

    SUMO index 0..7   green phases k = 1..8 of the methodology (Sec. 3.2)
    SUMO index 8      exclusive pedestrian phase (not used in Phase I)
    SUMO index 9..16  yellow of each green phase (found by matching, not assumed)
    SUMO index 17     all-red clearance
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

from ..common.paths import NETWORK_DIR

GREEN, YELLOW, RED = 0, 1, 2
NUM_PHASES = 8


def char_code(c):
    if c in "Gg":
        return GREEN
    if c in "yY":
        return YELLOW
    return RED


class SignalPlan:
    def __init__(self, network_graph, tls_id="1",
                 net_file=os.path.join(NETWORK_DIR, "single_intersection_pedestrian_X.net.xml"),
                 add_file=os.path.join(NETWORK_DIR, "single_intersection_Concurrent.add.xml")):
        self.tls_id = tls_id
        graph = network_graph[tls_id]
        # Lane order j = 1..J used by the repo (and by the GAMS model).
        self.lane_ids = list(graph["incoming_veh"].keys())
        self.lane_index = {lane: j for j, lane in enumerate(self.lane_ids)}
        self.lane_length = {lane: graph["incoming_veh"][lane]["length"] for lane in self.lane_ids}
        self.crossing_ids = list(graph["crossing"].keys())
        self.crossing_geom = {c: (graph["crossing"][c]["length"], graph["crossing"][c]["width"])
                              for c in self.crossing_ids}
        self._read_net(net_file)
        self._read_program(add_file)

    # ------------------------------------------------------------------ parsing
    def _read_net(self, net_file):
        root = ET.parse(net_file).getroot()
        internal_len = {}
        crossing_edges = {}
        for edge in root.findall("edge"):
            if edge.get("function") == "crossing":
                crossing_edges[edge.get("id")] = edge.get("crossingEdges", "").split()
            for lane in edge.findall("lane"):
                internal_len[lane.get("id")] = float(lane.get("length"))
        self.lane_links = {lane: [] for lane in self.lane_ids}
        self.link_lane, self.link_to_edge, self.link_via_length = {}, {}, {}
        self.crossing_link = {}
        for con in root.findall("connection"):
            if con.get("tl") != self.tls_id:
                continue
            link = int(con.get("linkIndex"))
            frm, to = con.get("from"), con.get("to")
            if to in crossing_edges:            # walking area -> crossing
                self.crossing_link[to] = link
                continue
            lane = f"{frm}_{con.get('fromLane')}"
            if lane in self.lane_links:
                self.lane_links[lane].append(link)
                self.link_lane[link] = lane
                self.link_to_edge[link] = to
                self.link_via_length[link] = internal_len.get(con.get("via"), 0.0)
        # Crossing a turning vehicle meets on its exit leg (for the rollout's virtual stop).
        self.link_conflict_crossing = {}
        for link, to_edge in self.link_to_edge.items():
            for c, edges in crossing_edges.items():
                if to_edge in edges and c in self.crossing_link and len(edges) == 2:
                    self.link_conflict_crossing[link] = c
        self.n_links = 1 + max(list(self.link_lane) + list(self.crossing_link.values()))

    def _read_program(self, add_file):
        root = ET.parse(add_file).getroot()
        logic = next(t for t in root.findall("tlLogic") if t.get("id") == self.tls_id)
        states = [p.get("state") for p in logic.findall("phase")]
        self.all_states = states
        self.green_states = states[:NUM_PHASES]
        self.all_red_index = next(i for i, s in enumerate(states) if set(s) == {"r"})
        self.all_red_state = states[self.all_red_index]
        # Yellow of green phase k sits at k + (number of green phases incl. the exclusive
        # one), the same convention as the repo's apply_control_commands.
        n_green_total = sum(1 for s in states if "G" in s or "g" in s)
        self.yellow_index = {k: k + n_green_total for k in range(NUM_PHASES)}
        # Vehicle links that are green in phase k but not yellow in its yellow phase go
        # straight from green to red (a property of the repo's add.xml, kept as is).
        vehicle_links = set(self.link_lane)
        self.links_without_yellow = {}
        for k, g in enumerate(self.green_states):
            y = states[self.yellow_index[k]]
            greens = {i for i, c in enumerate(g) if c in "Gg" and i in vehicle_links}
            yellows = {i for i, c in enumerate(y) if c in "yY"}
            if not yellows <= greens:
                raise ValueError(f"Yellow state {self.yellow_index[k]} does not belong to green phase {k}")
            if greens - yellows:
                self.links_without_yellow[k] = sorted(greens - yellows)
        self.n_links = max(self.n_links, len(self.green_states[0]))
        # Code arrays (n_links,) for every state string used by the controller.
        self.state_codes = {s: np.array([char_code(c) for c in s.ljust(self.n_links, "r")], dtype=np.int64)
                            for s in states}
        # A (phase x lane) and B (phase x crossing) lookups of Eq. 5e-5f.
        self.A = np.array([[int(any(self.green_states[k][l] in "Gg" for l in self.lane_links[lane]))
                            for lane in self.lane_ids] for k in range(NUM_PHASES)])
        self.B = np.array([[int(self.green_states[k][self.crossing_link[c]] in "Gg") if c in self.crossing_link else 0
                            for c in self.crossing_ids] for k in range(NUM_PHASES)])
        # Crossings served by at least one of the 8 phases (the diagonals are not).
        self.served_crossings = [c for i, c in enumerate(self.crossing_ids) if self.B[:, i].any()]

    # ------------------------------------------------------------------ lookups
    def link_code(self, sumo_state_index, link):
        if link is None or link < 0:
            return GREEN
        return int(self.state_codes[self.all_states[sumo_state_index]][link])

    def link_green_in_phase(self, k, link):
        if link is None or link < 0:
            return True
        return self.green_states[k][link] in "Gg"

    def crossing_green(self, k, crossing):
        return self.green_states[k][self.crossing_link[crossing]] in "Gg"

    def option_schedule(self, current_phase, new_phase, n_window, n_yellow, n_all_red):
        """SUMO state indices for each 0.5 s step of one upper option.

        A change of phase is preceded by the yellow of the current phase and an
        all-red interval (Sec. 3.2); the 5 s window that follows is phase-pure.
        """
        steps = []
        if new_phase != current_phase:
            steps += [self.yellow_index[current_phase]] * n_yellow
            steps += [self.all_red_index] * n_all_red
        steps += [new_phase] * n_window
        n_clear = len(steps) - n_window
        return steps, n_clear

    def min_ped_green(self, k, ped_counts, ped_speed):
        """G_p = 3.2 + L/S_p + 2.7 N_p / W for the crossings served by phase k (Eq. 6).

        Width in feet as in the repo's MPC agent. Returns 0 when no served crossing
        has waiting pedestrians.
        """
        gp = 0.0
        for c in self.crossing_ids:
            if c in self.crossing_link and self.crossing_green(k, c) and ped_counts.get(c, 0) > 0:
                length, width = self.crossing_geom[c]
                gp = max(gp, 3.2 + length / ped_speed + 2.7 * ped_counts[c] / (width * 3.28))
        return gp
