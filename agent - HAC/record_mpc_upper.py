"""MPC warm start, step 1 of 3: record the MPC's phase decisions as upper-level transitions.

At every upper decision the repo's MPC-MINLP (slower-scale problem of agent/mpc_agent.py)
picks the phase instead of the upper SAC. HAC's own code builds S_upper, runs the option
(clearance + 5 s window), sums r_upper and writes the transition, so each row is exactly
what the upper SAC stores in B_upper during training. The lower level acts uniformly at
random and nothing is trained, as at the start of training.

    python record_mpc_upper.py --run-name phase1 --episodes 6          # needs GAMS
    python pretrain_upper_from_mpc.py --run-name phase1
    python train.py --run-name phase1 --init-upper runs/phase1/upper_mpc_init.pt

Needs the GAMS Python API next to torch/traci/sumolib (pip install gamsapi matching /opt/gams).
Writes runs/<run>/mpc_upper_demos.pt (rows + per-episode summary) and repo-format results
files under runs/<run>/mpc_upper_results/. Seeds are 20000 + base_seed + episode, apart from
the training seeds (base_seed + episode) and Stage 0 (10000 + ...).
"""
import os
import time

import torch

from common_cli import parser, setup
from hrl.runner import build_agents, run_episode
from hrl.upper.mpc_teacher import MpcPhaseTeacher

SEED_OFFSET = 20_000


def main():
    p = parser(__doc__)
    p.add_argument("--episodes", type=int, default=6, help="MPC episodes to record (ped levels cycle)")
    p.add_argument("--epsilon", type=float, default=0.1,
                   help="probability of applying a random legal phase instead of the MPC's (critic coverage)")
    p.add_argument("--out", help="default: runs/<run>/mpc_upper_demos.pt")
    p.add_argument("--append", action="store_true", help="add episodes to an existing demos file")
    p.add_argument("--world-model", help="world_model.pt for the CAV targets (default: runs/<run>/world_model.pt)")
    args = p.parse_args()
    cfg, run_dir = setup(args)
    out = args.out or os.path.join(run_dir, "mpc_upper_demos.pt")
    data = {"rows": [], "episodes": [], "config": cfg.to_dict(), "epsilon": args.epsilon}
    if os.path.exists(out):
        if not args.append:
            raise SystemExit(f"{out} exists; pass --append to add episodes or --out for a new file")
        data = torch.load(out, weights_only=False)
    first = len(data["episodes"])

    # Record only: the lower level stays random and neither SAC is updated.
    cfg.lower_warmup_transitions = 10 ** 12
    cfg.upper_warmup_transitions = 10 ** 12
    _, lower, upper, wm = build_agents(cfg)
    wm_path = args.world_model or os.path.join(run_dir, "world_model.pt")
    if os.path.exists(wm_path):
        wm.load_state_dict(torch.load(wm_path))
        print(f"loaded world model {wm_path}")
    else:
        print("no Stage 0 world model; CAV targets use the IDM backbone (does not affect the recorded phases)")

    results_dir = os.path.join(run_dir, "mpc_upper_results")
    for ep in range(first, first + args.episodes):
        seed = SEED_OFFSET + cfg.base_seed + ep
        ped = cfg.ped_demand_levels[ep % len(cfg.ped_demand_levels)]
        t0 = time.time()
        stats = run_episode(cfg, run_dir, f"mpc_upper_{ep}", seed, ped, lower=lower, upper=upper, world_model=wm,
                            training=True, episode=0, results_dir=results_dir, record_upper=True,
                            teacher_factory=lambda paras, env, s=seed: MpcPhaseTeacher(
                                paras, env, cfg, epsilon=args.epsilon, seed=s))
        rows = stats.pop("upper_rows")
        for r in rows:
            r["episode"] = ep
        data["rows"].extend(rows)
        summary = {k: v for k, v in stats.items() if not isinstance(v, (list, dict))}
        summary.update(episode=ep, n_rows=len(rows), wall_s=round(time.time() - t0, 1))
        data["episodes"].append(summary)
        torch.save(data, out)
        print(f"[mpc upper] ep {ep} seed {seed} ped {ped}: {len(rows)} transitions "
              f"(mpc {stats['teacher_mpc']}, noise {stats['teacher_noise']}, fallback {stats['teacher_fallback']}, "
              f"failed solves {stats['teacher_failed']}, {stats['teacher_solve_s']} s/solve)  veh TL "
              f"{stats['veh_time_loss']:.1f}s  ped TL {stats['ped_time_loss']:.1f}s  ({summary['wall_s']}s)")
    print(f"{len(data['rows'])} transitions from {len(data['episodes'])} episodes in {out}")


if __name__ == "__main__":
    main()
