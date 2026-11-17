"""Evaluate a trained HAC agent with deterministic policies.

Writes one results file per (pedestrian level, seed) in the repo's format, so they can
sit next to the MPC and actuated results and be parsed by the same figure scripts:

    python evaluate.py --run-name phase1 --seeds 10 1000 10000 23423 30000
"""
import csv
import os

from common_cli import parser, setup
from hrl.runner import PED_LABEL, build_agents, load_checkpoint, run_episode


def main():
    p = parser(__doc__)
    p.add_argument("--checkpoint", help="default: runs/<run>/checkpoints/latest.pt")
    p.add_argument("--seeds", type=int, nargs="+", default=[10, 1000, 10000, 23423, 30000])
    p.add_argument("--ped", type=float, nargs="+", default=[0.01, 0.04, 0.08])
    args = p.parse_args()
    cfg, run_dir = setup(args)
    plan, lower, upper, wm = build_agents(cfg)
    ck = args.checkpoint or os.path.join(run_dir, "checkpoints", "latest.pt")
    load_checkpoint(ck, lower, upper, wm)
    out_dir = os.path.join(run_dir, "eval")
    rows = []
    for ped in args.ped:
        for seed in args.seeds:
            s = run_episode(cfg, run_dir, f"eval_{seed}", seed, ped, lower=lower, upper=upper, world_model=wm,
                            training=False, results_dir=out_dir)
            rows.append(s)
            print(f"{PED_LABEL.get(ped, ped)} seed {seed}: veh TL {s['veh_time_loss']:.1f}s  ped TL "
                  f"{s['ped_time_loss']:.1f}s  fuel {s['fuel_cav_mg']:.1f}  conflicts {s['rt_conflicts']}  "
                  f"red viol {s['red_light_violations']}  collisions {s['collisions']}")
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r}))
        w.writeheader()
        w.writerows(rows)
    print(f"results in {out_dir}")


if __name__ == "__main__":
    main()
