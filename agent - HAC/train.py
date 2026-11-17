"""Phase I training loop (Sec. 6): Phase A (SUMO experience) with Phase B updates interleaved.

    python stage0_world_model.py --run-name phase1      # once: world model
    python train.py --run-name phase1                   # then: both SACs

MPC warm start of the upper level (record_mpc_upper.py, pretrain_upper_from_mpc.py):

    python train.py --run-name phase1 --init-upper runs/phase1/upper_mpc_init.pt

Writes runs/<run>/train_log.csv, checkpoints/ep_XXXX.pt, latest.pt and per-episode
results files in the repo's format under runs/<run>/results/.
"""
import csv
import os
import time

import torch

from common_cli import parser, setup
from hrl.runner import build_agents, load_checkpoint, run_episode, save_checkpoint
from hrl.world_model.data import ModelBuffer
from hrl.world_model.training import train_residuals

LOG_FIELDS = ["episode", "seed", "ped_gamma", "sim_steps", "wall_s", "veh_waiting_time", "veh_time_loss",
              "ped_time_loss", "queue_m", "rt_conflicts", "fuel_cav_mg", "r_lower_sum", "r_upper_sum",
              "upper_decisions", "phase_changes", "masked_decisions", "lower_steps", "cbf_intervention_rate",
              "cbf_infeasible", "red_infeasible", "red_light_violations", "sumo_overrides", "min_gap", "gap_below_d0_new_leader",
              "gap_below_d0_same_leader", "sumo_overrides_stopline", "sumo_overrides_junction", "sumo_overrides_other",
              "collisions", "teleports", "mean_goal_shortfall", "lower_q_loss", "lower_alpha", "upper_q_loss",
              "upper_alpha", "wm_loss", "lower_buffer", "upper_buffer"]  ## columns of train_log.csv  (one row per episode)


def main():
    p = parser(__doc__)
    p.add_argument("--resume", action="store_true", help="continue from runs/<run>/checkpoints/latest.pt")
    p.add_argument("--world-model", help="world_model.pt from Stage 0 (default: runs/<run>/world_model.pt)")
    p.add_argument("--init-upper", help="MPC warm start of the upper level: upper_mpc_init.pt from "
                                        "pretrain_upper_from_mpc.py (lower level unchanged)")
    args = p.parse_args()
    cfg, run_dir = setup(args)
    if args.init_upper:
        cfg.upper_init = os.path.abspath(args.init_upper)
    ck_dir = os.path.join(run_dir, "checkpoints")
    os.makedirs(ck_dir, exist_ok=True)
    plan, lower, upper, wm = build_agents(cfg)

    start = 0
    latest = os.path.join(ck_dir, "latest.pt")
    if args.resume and os.path.exists(latest):
        ck = load_checkpoint(latest, lower, upper, wm)
        start = ck["episode"] + 1
        if not cfg.upper_init and ck["config"].get("upper_init"):
            # Keep the warm-start behaviour (no random-phase episodes, actor freeze) on resume.
            cfg.upper_init = ck["config"]["upper_init"]
        print(f"resumed from {latest} at episode {start} (replay buffers start empty)")
    else:
        wm_path = args.world_model or os.path.join(run_dir, "world_model.pt")
        if os.path.exists(wm_path):
            wm.load_state_dict(torch.load(wm_path))
            print(f"loaded Stage 0 world model {wm_path}")
        else:
            print("WARNING: no Stage 0 world model found; HDV residuals start at zero (pure IDM backbone)")
        if cfg.upper_init:
            init = torch.load(cfg.upper_init, weights_only=False)
            upper.load_state_dict(init["upper"])
            agree = init.get("bc", {}).get("held_out_agreement")
            agree = f"{agree:.0%}" if agree is not None else "n/a"
            print(f"MPC warm start: upper SAC from {cfg.upper_init} (held-out agreement with the MPC {agree}, "
                  f"alpha {float(upper.alpha.detach()):.3g}); no random-phase episodes, "
                  f"actor frozen for the first {cfg.upper_actor_freeze_episodes} episodes")
    if cfg.upper_init and cfg.upper_init_preload:
        if os.path.exists(cfg.upper_init):
            upper.buffer.add_batch(**torch.load(cfg.upper_init, weights_only=False)["rows"])
            print(f"B_upper starts with {upper.buffer.size} recorded MPC transitions")
        else:
            print(f"WARNING: {cfg.upper_init} not found; B_upper starts empty")
    cfg.save(os.path.join(run_dir, "config.json"))

    model_buffer = ModelBuffer(cfg.wm_buffer_episodes)
    b_model_path = os.path.join(run_dir, "b_model.pt")
    if os.path.exists(b_model_path):
        model_buffer.episodes = torch.load(b_model_path, weights_only=False)[-cfg.wm_buffer_episodes:]

    log_path = os.path.join(run_dir, "train_log.csv")
    new_log = not os.path.exists(log_path) or start == 0
    with open(log_path, "w" if new_log else "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction="ignore")
        if new_log:
            writer.writeheader()
        for ep in range(start, cfg.episodes):
            seed = cfg.base_seed + ep
            ped = cfg.ped_demand_levels[ep % len(cfg.ped_demand_levels)]
            t0 = time.time()
            stats = run_episode(cfg, run_dir, "train", seed, ped, lower=lower, upper=upper, world_model=wm,
                                training=True, episode=ep, model_buffer=model_buffer)
            # World-model fine-tuning on B_model (Sec. 6.3), N_model-spaced steps of this episode.
            n_wm = cfg.wm_finetune_steps * max(1, stats["sim_steps"] // cfg.wm_finetune_every)
            hist = train_residuals(wm, model_buffer, cfg, n_wm, log_every=0, printer=lambda *_: None)
            stats["wm_loss"] = sum(hist[-20:]) / len(hist[-20:]) if hist else ""
            stats.update(episode=ep, wall_s=round(time.time() - t0, 1),
                         lower_buffer=lower.buffer.size, upper_buffer=upper.buffer.size)
            writer.writerow(stats)
            f.flush()
            print(f"ep {ep:4d} seed {seed} ped {ped}: veh TL {stats['veh_time_loss']:.1f}s  ped TL "
                  f"{stats['ped_time_loss']:.1f}s  fuel {stats['fuel_cav_mg']:.1f}  CBF {stats['cbf_intervention_rate']:.2%}"
                  f"  red viol {stats['red_light_violations']}  coll {stats['collisions']}  ({stats['wall_s']}s)")
            save_checkpoint(latest, lower, upper, wm, cfg, ep)
            if (ep + 1) % cfg.checkpoint_every == 0:
                save_checkpoint(os.path.join(ck_dir, f"ep_{ep:04d}.pt"), lower, upper, wm, cfg, ep)


if __name__ == "__main__":
    main()
