"""Residual training with the K-step unrolled loss, and validation against SUMO (Sec. 4.4)."""
import random

import numpy as np
import torch

from ..upper.signal_plan import GREEN, RED
from .data import ModelBuffer
from .rollout import BIG, TYPE_CAV


def _no_yield(K, st):
    return torch.zeros((K,) + st.x.shape, dtype=torch.bool), torch.full_like(st.x, BIG)


def unrolled_loss(model, buffer, items, K, lambda_v):
    """L(eps) = sum_k ||s_k^model - s_k^SUMO||^2 + lambda_v ||v_k^model - v_k^SUMO||^2 (Sec. 4.4).

    CAVs and teacher-forced road users follow their logged trajectory; the loss is taken
    over HDVs and cyclists only. A single HDV residual covers both signal regimes; the
    red_regime flag is one of its inputs.
    """
    st, codes, tx, tv, tm = buffer.build_batch(items, K)
    is_cav = (st.vtype == TYPE_CAV).unsqueeze(0).expand_as(tm)
    forced = (is_cav & tm, tx, tv)
    ya, yx = _no_yield(K, st)
    xs, vs, _ = model.rollout(st, codes, ya, yx, torch.full_like(st.x, BIG), use_glosa=False, forced=forced)
    mask = tm & ~is_cav & st.valid.unsqueeze(0)
    if mask.sum() == 0:
        return None
    err = (xs[1:] - tx) ** 2 + lambda_v * (vs[1:] - tv) ** 2
    return (err * mask).sum() / mask.sum()


def train_residuals(model, buffer, cfg, steps, log_every=200, printer=print):
    params = list(model.res_hdv.parameters()) + list(model.res_bike.parameters())
    opt = torch.optim.Adam(params, lr=cfg.wm_lr)
    index = buffer.sample_index(cfg.wm_K)
    if not index:
        printer("world model: no HDV/cyclist windows in B_model yet; residuals unchanged")
        return []
    history = []
    for it in range(steps):
        loss = unrolled_loss(model, buffer, random.sample(index, min(cfg.wm_batch, len(index))), cfg.wm_K, cfg.wm_lambda_v)
        if loss is None:
            continue
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, 1.0)
        opt.step()
        history.append(loss.item())
        if log_every and (it + 1) % log_every == 0:
            printer(f"  world model step {it + 1}/{steps}: K-step loss {np.mean(history[-log_every:]):.4f}")
    return history


@torch.no_grad()
def validate(model, buffer, horizons=(10, 20, 30), n_samples=400, seed=0):
    """Position/speed RMSE of rollouts from logged SUMO states at 5, 10, 15 s (Sec. 4.4)."""
    K = max(horizons)
    index = buffer.sample_index(K)
    if not index:
        return {}
    rng = random.Random(seed)
    items = rng.sample(index, min(n_samples, len(index)))
    report = {}
    for use_res in (False, True):
        acc = {h: {"x": [], "v": []} for h in horizons}
        for start in range(0, len(items), 64):
            st, codes, tx, tv, tm = buffer.build_batch(items[start:start + 64], K)
            is_cav = (st.vtype == TYPE_CAV).unsqueeze(0).expand_as(tm)
            ya, yx = _no_yield(K, st)
            xs, vs, _ = model.rollout(st, codes, ya, yx, torch.full_like(st.x, BIG), use_glosa=False,
                                      use_residual=use_res, forced=(is_cav & tm, tx, tv))
            for h in horizons:
                m = tm[h - 1] & ~is_cav[h - 1] & st.valid
                acc[h]["x"].append(((xs[h] - tx[h - 1]) ** 2)[m])
                acc[h]["v"].append(((vs[h] - tv[h - 1]) ** 2)[m])
        key = "backbone+residual" if use_res else "backbone"
        report[key] = {f"{h * model.cfg.dt:g}s": {
            "pos_rmse": float(torch.cat(acc[h]["x"]).mean().sqrt()) if acc[h]["x"] else float("nan"),
            "speed_rmse": float(torch.cat(acc[h]["v"]).mean().sqrt()) if acc[h]["v"] else float("nan"),
            "n": int(sum(t.numel() for t in acc[h]["x"]))} for h in horizons}
    report["discharge"] = discharge_check(model, buffer)
    return report


@torch.no_grad()
def discharge_check(model, buffer, window_steps=20, min_queue=3):
    """Queue discharge in the first 10 s of green: SUMO vs world model (veh/h per lane)."""
    sumo_rates, model_rates = [], []
    for frames in buffer.episodes:
        for t in range(1, len(frames) - window_steps):
            for lane in range(12):
                ids = [vid for vid in ModelBuffer.lane_members(frames[t], lane)
                       if frames[t]["x"][frames[t]["index"][vid]] < 0]
                if len(ids) < min_queue:
                    continue
                f_prev, f_now = frames[t - 1], frames[t]
                front = ids[0]
                if front not in f_prev["index"]:
                    continue
                if not (f_prev["code"][f_prev["index"][front]] == RED and f_now["code"][f_now["index"][front]] == GREEN):
                    continue
                stopped = [vid for vid in ids if f_now["v"][f_now["index"][vid]] < 0.1]
                if len(stopped) < min_queue:
                    continue
                st, codes, tx, tv, tm = buffer.build_batch([(buffer.episodes.index(frames), t, lane)], window_steps)
                ya, yx = _no_yield(window_steps, st)
                is_cav = (st.vtype == TYPE_CAV).unsqueeze(0).expand_as(tm)
                xs, _, _ = model.rollout(st, codes, ya, yx, torch.full_like(st.x, BIG), use_glosa=False,
                                         forced=(is_cav & tm, tx, tv))
                before = st.valid & (st.x < 0)
                n_sumo = int((before & tm[-1] & (tx[-1] >= 0)).sum() + (before & ~tm[-1]).sum())
                n_model = int((before & (xs[-1] >= 0)).sum())
                seconds = window_steps * model.cfg.dt
                sumo_rates.append(n_sumo / seconds * 3600)
                model_rates.append(n_model / seconds * 3600)
    if not sumo_rates:
        return {"events": 0}
    return {"events": len(sumo_rates), "sumo_veh_per_h": float(np.mean(sumo_rates)),
            "model_veh_per_h": float(np.mean(model_rates))}
