"""Stage 0 (Sec. 6.1): fill B_model with baseline SUMO runs, train and validate the residuals.

Baseline: the repo's actuated signal program, every vehicle driven by SUMO (Krauss).
Outputs in runs/<run>/:  world_model.pt, b_model.pt, world_model_validation.json

    python stage0_world_model.py --run-name phase1 --episodes 6
"""
import json
import os

import torch

from common_cli import parser, setup
from hrl.runner import build_agents, run_episode
from hrl.world_model.data import ModelBuffer
from hrl.world_model.training import train_residuals, validate


def main():
    p = parser(__doc__)
    p.add_argument("--episodes", type=int, default=6, help="baseline episodes to log")
    p.add_argument("--steps", type=int, help="residual gradient steps (default: wm_pretrain_steps)")
    args = p.parse_args()
    cfg, run_dir = setup(args)
    _, _, _, wm = build_agents(cfg)
    buffer = ModelBuffer(max_episodes=args.episodes)
    for ep in range(args.episodes):
        seed = 10_000 + cfg.base_seed + ep
        ped = cfg.ped_demand_levels[ep % len(cfg.ped_demand_levels)]
        print(f"[stage 0] baseline episode {ep + 1}/{args.episodes} (seed {seed}, ped {ped})")
        run_episode(cfg, run_dir, f"stage0_{ep}", seed, ped, program="actuated", model_buffer=buffer,
                    results_dir=os.path.join(run_dir, "stage0_results"))
        print(f"  B_model frames so far: {buffer.n_frames()}")

    # Hold out the last episode for validation when there is more than one.
    if len(buffer.episodes) > 1:
        held = ModelBuffer()
        held.episodes = [buffer.episodes.pop()]
    else:
        held = buffer
    before = validate(wm, held)
    steps = args.steps or cfg.wm_pretrain_steps
    print(f"[stage 0] training residuals for {steps} steps (K = {cfg.wm_K})")
    train_residuals(wm, buffer, cfg, steps)
    after = validate(wm, held)
    report = {"before_training": before, "after_training": after}
    with open(os.path.join(run_dir, "world_model_validation.json"), "w") as f:
        json.dump(report, f, indent=2)
    if held is not buffer:
        buffer.episodes.append(held.episodes[0])
    torch.save(wm.state_dict(), os.path.join(run_dir, "world_model.pt"))
    torch.save(buffer.episodes, os.path.join(run_dir, "b_model.pt"))
    print(json.dumps(after, indent=2))
    print(f"[stage 0] saved world model and B_model to {run_dir}")


if __name__ == "__main__":
    main()
