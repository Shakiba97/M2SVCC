"""B_model: logged SUMO trajectories of every road user (Sec. 4.6), and batches of
K-step windows built from them for residual training and validation."""
import numpy as np
import torch

from ..upper.signal_plan import GREEN
from .rollout import BIG, TYPE_BIKE, TYPE_HDV, LaneState


class ModelBuffer:
    """B_model: logged SUMO trajectories of every road user with its signal state (Sec. 4.6).

    One frame per 0.5 s step: arrays over the road users present, each tagged with the
    approach lane it came from (kept after it crosses the stop line).
    """

    def __init__(self, max_episodes=20):
        self.max_episodes = max_episodes
        self.episodes = []

    def start_episode(self):
        self.episodes.append([])
        if len(self.episodes) > self.max_episodes:
            self.episodes.pop(0)

    def add_frame(self, ids, lane, x, v, length, vtype, code, wait):
        self.episodes[-1].append({
            "ids": list(ids), "index": {vid: i for i, vid in enumerate(ids)},
            "lane": np.asarray(lane, dtype=np.int64), "x": np.asarray(x, dtype=np.float32),
            "v": np.asarray(v, dtype=np.float32), "L": np.asarray(length, dtype=np.float32),
            "type": np.asarray(vtype, dtype=np.int64), "code": np.asarray(code, dtype=np.int64),
            "wait": np.asarray(wait, dtype=np.float32),
        })

    def n_frames(self):
        return sum(len(e) for e in self.episodes)

    # ------------------------------------------------------------ windows
    @staticmethod
    def lane_members(frame, lane):
        """Road users of one lane at one frame, front to back (closest crossed leader first)."""
        sel = np.where(frame["lane"] == lane)[0]
        if sel.size == 0:
            return []
        xs = frame["x"][sel]
        before = sel[xs < 0]
        crossed = sel[xs >= 0]
        order = list(before[np.argsort(-frame["x"][before])])
        if crossed.size:
            order = [crossed[np.argmin(frame["x"][crossed])]] + order
        return [frame["ids"][i] for i in order]

    def sample_index(self, K, require_type=(TYPE_HDV, TYPE_BIKE), n_lanes=12):
        idx = []
        for e, frames in enumerate(self.episodes):
            for t in range(0, len(frames) - K, 2):
                fr = frames[t]
                for lane in range(n_lanes):
                    m = (fr["lane"] == lane) & (fr["x"] < 0) & np.isin(fr["type"], require_type)
                    if m.any():
                        idx.append((e, t, lane))
        return idx

    def build_batch(self, items, K):
        rows = []
        for e, t, lane in items:
            frames = self.episodes[e]
            ids = self.lane_members(frames[t], lane)
            rows.append((frames, t, ids))
        N = max(1, max(len(r[2]) for r in rows))
        B = len(rows)
        x0 = np.full((B, N), -BIG, np.float32)
        v0 = np.zeros((B, N), np.float32)
        L = np.zeros((B, N), np.float32)
        vt = np.full((B, N), TYPE_HDV, np.int64)
        valid = np.zeros((B, N), bool)
        wait = np.zeros((B, N), np.float32)
        codes = np.full((K, B, N), GREEN, np.int64)
        tx = np.zeros((K, B, N), np.float32)
        tv = np.zeros((K, B, N), np.float32)
        tm = np.zeros((K, B, N), bool)
        for b, (frames, t, ids) in enumerate(rows):
            f0 = frames[t]
            for n, vid in enumerate(ids):
                i = f0["index"][vid]
                x0[b, n], v0[b, n], L[b, n] = f0["x"][i], f0["v"][i], f0["L"][i]
                vt[b, n], valid[b, n], wait[b, n] = f0["type"][i], True, f0["wait"][i]
                for k in range(K):
                    fk = frames[t + k]          # signal applied during step k
                    fn = frames[t + k + 1]      # resulting state
                    if vid in fk["index"]:
                        codes[k, b, n] = fk["code"][fk["index"][vid]]
                    if vid in fn["index"]:
                        j = fn["index"][vid]
                        tx[k, b, n], tv[k, b, n], tm[k, b, n] = fn["x"][j], fn["v"][j], True
        tt = torch.from_numpy
        st = LaneState(tt(x0), tt(v0), tt(L), tt(vt), tt(valid), tt(wait))
        return st, tt(codes), tt(tx), tt(tv), tt(tm)
