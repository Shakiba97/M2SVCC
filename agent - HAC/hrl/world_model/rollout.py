"""Joint lane-rollout world model (Sec. 4).

Every approach lane is one row of a batch; road users in a row are ordered front
(index 0) to back, padded at the end. Positions x are front-bumper positions
relative to the stop line (x < 0 before the line). One call advances all rows
synchronously with the kinematics of Sec. 4.3; the rollout is written in torch so
the same code serves inference (no_grad) and the K-step residual training (Sec. 4.4).
"""
from dataclasses import dataclass

import torch
import torch.nn as nn

from ..upper.signal_plan import RED, YELLOW

TYPE_CAV, TYPE_HDV, TYPE_BIKE = 0, 1, 2
BIG = 1e4


@dataclass
class LaneState:
    x: torch.Tensor        # [B, N] position relative to stop line (front bumper)
    v: torch.Tensor        # [B, N]
    length: torch.Tensor   # [B, N]
    vtype: torch.Tensor    # [B, N] long
    valid: torch.Tensor    # [B, N] bool
    wait: torch.Tensor     # [B, N] waiting time (SUMO definition: time with v < 0.1)

    def clone(self):
        return LaneState(self.x.clone(), self.v.clone(), self.length, self.vtype, self.valid, self.wait.clone())


class ResidualNet(nn.Module):
    N_FEATURES = 7

    def __init__(self, hidden=64, scale=2.0):
        super().__init__()
        self.scale = scale
        self.net = nn.Sequential(nn.Linear(self.N_FEATURES, hidden), nn.Tanh(),
                                 nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, 1))
        nn.init.zeros_(self.net[-1].weight)   # start exactly at the physics backbone
        nn.init.zeros_(self.net[-1].bias)

    def forward(self, f):
        return self.scale * torch.tanh(self.net(f)).squeeze(-1)


class LaneWorldModel(nn.Module):
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        # One HDV residual for both signal regimes; the regime is an input feature (see features()).
        # This replaces the methodology's separate T_red / T_green nets: it shares the
        # car-following correction common to both regimes instead of learning it twice.
        self.res_hdv = ResidualNet(cfg.residual_hidden, cfg.residual_scale)
        self.res_bike = ResidualNet(cfg.residual_hidden, cfg.residual_scale)

    # ------------------------------------------------------------ components
    def _leaders(self, st, red, yield_active, yield_x):
        x, v, L, valid = st.x, st.v, st.length, st.valid
        B = x.shape[0]
        pad = torch.full((B, 1), BIG, dtype=x.dtype)
        x_lead = torch.cat([pad, x[:, :-1]], 1)
        L_lead = torch.cat([torch.zeros_like(pad), L[:, :-1]], 1)
        v_lead = torch.cat([torch.zeros_like(pad), v[:, :-1]], 1)
        valid_lead = torch.cat([torch.zeros((B, 1), dtype=torch.bool), valid[:, :-1]], 1) & valid
        gap_real = torch.where(valid_lead, x_lead - L_lead - x, torch.full_like(x, BIG))
        gap_stop = torch.where(red, -x, torch.full_like(x, BIG))
        gap_yield = torch.where(yield_active & (x < yield_x), yield_x - x, torch.full_like(x, BIG))
        gap_virtual = torch.minimum(gap_stop, gap_yield)
        gap = torch.minimum(gap_real, gap_virtual)
        virt = gap_virtual < gap_real
        v_l = torch.where(virt, torch.zeros_like(v), v_lead)
        has_l = gap < BIG / 2
        return gap, v_l, has_l, virt

    def _idm(self, v, gap, v_l, has_l):
        c = self.cfg
        s_star = c.idm_s0 + torch.relu(v * c.idm_T + v * (v - v_l) / (2 * (c.a_max * c.idm_b_comf) ** 0.5))
        inter = torch.where(has_l, (s_star / torch.clamp(gap, min=0.1)) ** 2, torch.zeros_like(v))
        return c.a_max * (1 - (torch.clamp(v, min=0) / c.idm_v0) ** c.idm_delta - inter)

    def _bike_ftl(self, v, gap, has_l):
        """Placeholder follow-the-leader desired-speed rule; replace with M2SVCC Eq. 1-2."""
        c = self.cfg
        v_target = torch.where(has_l, torch.clamp(torch.relu(gap - c.bike_s0) / c.bike_T, max=c.bike_v_des),
                               torch.full_like(v, c.bike_v_des))
        return v_target - v

    @staticmethod
    def features(st, gap, v_l, has_l, virt, red, v0):
        """Residual inputs. `red` (the red_regime flag) is the only signal information: without it
        a road user queued behind a real leader looks the same under red and green."""
        return torch.stack([
            st.v / v0,
            torch.clamp(gap, 0, 100) / 100,
            (st.v - v_l) / v0 * has_l,
            torch.clamp(st.x, -200, 50) / 100,
            has_l.float(),
            virt.float(),
            red.float(),
        ], dim=-1)

    def red_regime(self, st, code):
        """Lane red, or yellow and the road user can still stop (Sec. 4.2, 5.2)."""
        crossed = st.x >= 0
        can_stop = (-st.x) >= st.v ** 2 / (2 * self.cfg.b_max) + self.cfg.d_s
        return (~crossed) & ((code == RED) | ((code == YELLOW) & can_stop))

    # ------------------------------------------------------------ dynamics
    def step(self, st, code, yield_active, yield_x, green_start, t_now,
             use_glosa=True, use_residual=True, forced=None):
        c = self.cfg
        dt = c.dt
        red = self.red_regime(st, code)
        gap, v_l, has_l, virt = self._leaders(st, red, yield_active, yield_x)
        a_idm = self._idm(st.v, gap, v_l, has_l)

        # CAV reference controller: min(IDM car-following, GLOSA tracking toward the predicted
        # green start). While GLOSA is active it handles the stop line, so the IDM term only
        # follows real leaders and pedestrian yield points; the red hold below stays as a backstop.
        a_cav = a_idm
        if use_glosa:
            t_to_green = green_start - t_now
            need = (st.x < 0) & (t_to_green > 0)
            gap_cf, vl_cf, has_cf, _ = self._leaders(st, torch.zeros_like(red), yield_active, yield_x)
            a_cf = self._idm(st.v, gap_cf, vl_cf, has_cf)
            d = torch.clamp(-st.x - c.idm_s0, min=0)
            v_tgt = torch.clamp(d / torch.clamp(t_to_green, min=dt), max=c.idm_v0)
            a_cav = torch.where(need, torch.minimum(a_cf, (v_tgt - st.v) / c.glosa_tau), a_idm)

        a_hdv, a_bike = a_idm, self._bike_ftl(st.v, gap, has_l)
        if use_residual:
            f = self.features(st, gap, v_l, has_l, virt, red, c.idm_v0)
            a_hdv = a_hdv + self.res_hdv(f)
            a_bike = a_bike + self.res_bike(f)

        a = torch.where(st.vtype == TYPE_CAV, a_cav, torch.where(st.vtype == TYPE_HDV, a_hdv, a_bike))
        a = torch.clamp(a, -c.rollout_brake, c.a_max)
        a_eff = torch.maximum(a, -st.v / dt)               # stop inside the step, never reverse
        x_new = st.x + st.v * dt + 0.5 * a_eff * dt * dt
        v_new = torch.relu(st.v + a_eff * dt)
        x_new = torch.where(red, torch.minimum(x_new, torch.zeros_like(x_new)), x_new)

        if forced is not None:                              # teacher-forced road users (training)
            f_mask, f_x, f_v = forced
            x_new = torch.where(f_mask, f_x, x_new)
            v_new = torch.where(f_mask, f_v, v_new)

        # No overlap with the road user ahead: x_i <= x_{i-1} - L_{i-1}.
        Lv = st.length * st.valid
        cum_l = torch.cumsum(Lv, 1) - Lv
        z = torch.where(st.valid, x_new + cum_l, torch.full_like(x_new, -BIG))
        x_new = torch.where(st.valid, torch.cummin(z, 1).values - cum_l, st.x)
        v_new = torch.where(st.valid, v_new, st.v)
        wait = torch.where(v_new < 0.1, st.wait + dt, torch.zeros_like(st.wait))
        return LaneState(x_new, v_new, st.length, st.vtype, st.valid, wait)

    def rollout(self, st, codes, yield_active, yield_x, green_start, t0=0.0,
                use_glosa=True, use_residual=True, forced=None):
        """Roll T = codes.shape[0] steps. Returns trajectories x, v [T+1, B, N] and the final state."""
        xs, vs = [st.x], [st.v]
        for t in range(codes.shape[0]):
            f = None if forced is None else (forced[0][t], forced[1][t], forced[2][t])
            st = self.step(st, codes[t], yield_active[t], yield_x, green_start, t0 + t * self.cfg.dt,
                           use_glosa, use_residual, f)
            xs.append(st.x)
            vs.append(st.v)
        return torch.stack(xs), torch.stack(vs), st
