"""MPC warm start, step 2 of 3: pre-train the upper SAC on the recorded MPC decisions.

  1. Behaviour cloning: the actor's masked softmax is fitted to the MPC's phase
     (cross-entropy) on every decision where the MPC was consulted.
  2. Critic pre-training: SAC critic updates on all recorded transitions with the actor
     frozen, so the critic is not random when online training starts. alpha is set to
     upper_init_alpha first and saved with the weights.

Reports how often the cloned actor picks the MPC's phase on a held-out episode, and writes
runs/<run>/upper_mpc_init.pt (weights, alpha and the transitions) for train.py --init-upper.
No SUMO or GAMS needed.

    python pretrain_upper_from_mpc.py --run-name phase1
"""
import math
import os

import numpy as np
import torch

from common_cli import parser, setup
from hrl.runner import build_agents

FIELDS = {"s": np.float32, "a": np.int64, "r": np.float32, "s2": np.float32, "done": np.float32,
          "duration": np.float32, "mask": np.float32, "mask2": np.float32}
# Fields that change r_upper or the SMDP discount: recorded rewards are only valid if they match.
REWARD_FIELDS = ("alpha", "beta", "gamma_bike", "upper_reward_scale", "upper_reward_mode", "lambda_cbf",
                 "window", "yellow_time", "all_red_time", "dt", "gamma_up", "preference_conditioned")


def columns(rows):
    return {k: np.asarray([r[k] for r in rows], dtype=t) for k, t in FIELDS.items()}


def bc_step(upper, s, m, tk, smoothing):
    """Cross-entropy of the masked policy against the MPC's phase, and the agreement rate."""
    p, logp = upper._policy(s, m)
    nll = -logp.gather(1, tk.unsqueeze(1)).squeeze(1)
    if smoothing > 0:                     # spread `smoothing` of the target over the legal phases
        nll = (1 - smoothing) * nll - smoothing * (logp * m).sum(-1) / m.sum(-1)
    return nll.mean(), float((p.argmax(-1) == tk).float().mean())


def main():
    p = parser(__doc__)
    p.add_argument("--demos", help="default: runs/<run>/mpc_upper_demos.pt")
    p.add_argument("--out", help="default: runs/<run>/upper_mpc_init.pt")
    p.add_argument("--bc-steps", type=int, default=3000)
    p.add_argument("--bc-lr", type=float, default=1e-3)
    p.add_argument("--critic-steps", type=int, default=3000)
    p.add_argument("--smoothing", type=float, default=0.0, help="label smoothing of the cloned policy")
    args = p.parse_args()
    cfg, run_dir = setup(args)
    demos = args.demos or os.path.join(run_dir, "mpc_upper_demos.pt")
    out = args.out or os.path.join(run_dir, "upper_mpc_init.pt")
    data = torch.load(demos, weights_only=False)
    rows = data["rows"]
    if not rows:
        raise SystemExit(f"no transitions in {demos}")
    rec = data["config"]
    diff = {k: (rec.get(k), getattr(cfg, k)) for k in REWARD_FIELDS if rec.get(k) != getattr(cfg, k)}
    if diff:
        print(f"WARNING: recorded with different reward settings (recorded, now): {diff}")

    _, _, upper, _ = build_agents(cfg)
    if upper.actor[0].in_features != len(rows[0]["s"]):
        raise SystemExit(f"S_upper has {len(rows[0]['s'])} entries in {demos} but {upper.actor[0].in_features} "
                         "with this config (preference_conditioned?)")

    # Behaviour cloning on the decisions where the MPC was consulted; the last episode is held out.
    episodes = sorted({r["episode"] for r in rows})
    held_ep = episodes[-1] if len(episodes) > 1 else None
    bc_rows = [r for r in rows if r["teacher_k"] >= 0]
    train_rows = [r for r in bc_rows if r["episode"] != held_ep]
    held_rows = [r for r in bc_rows if r["episode"] == held_ep]
    if not train_rows:
        raise SystemExit("no decisions with an MPC phase to clone")

    def tensors(rs):
        return (torch.as_tensor(np.asarray([r["s"] for r in rs], np.float32)),
                torch.as_tensor(np.asarray([r["mask"] for r in rs], np.float32)),
                torch.as_tensor(np.asarray([r["teacher_k"] for r in rs], np.int64)))

    S, M, K = tensors(train_rows)
    held = tensors(held_rows) if held_rows else None
    hold_rate = float(np.mean([int(np.argmax(r["s"][:8])) == r["teacher_k"] for r in bc_rows]))
    print(f"{len(rows)} transitions, {len(bc_rows)} with an MPC phase ({len(train_rows)} train, "
          f"{len(held_rows)} held out = episode {held_ep}); the MPC holds the current phase in {hold_rate:.0%}")

    opt = torch.optim.Adam(upper.actor.parameters(), lr=args.bc_lr)
    batch = min(cfg.batch_upper, len(train_rows))
    for it in range(args.bc_steps):
        idx = torch.randint(0, len(train_rows), (batch,))
        loss, _ = bc_step(upper, S[idx], M[idx], K[idx], args.smoothing)
        opt.zero_grad()
        loss.backward()
        opt.step()
        if (it + 1) % 500 == 0 or it + 1 == args.bc_steps:
            with torch.no_grad():
                tr_loss, tr_acc = bc_step(upper, S, M, K, args.smoothing)
                ho = f"  held-out agreement {bc_step(upper, *held, args.smoothing)[1]:.1%}" if held else ""
            print(f"  BC step {it + 1}/{args.bc_steps}: loss {float(tr_loss):.3f}  train agreement {tr_acc:.1%}{ho}")

    # Critic pre-training on every recorded transition (MPC, noise, fallback and forced decisions).
    with torch.no_grad():
        upper.log_alpha.fill_(math.log(cfg.upper_init_alpha))
    cols = columns(rows)
    upper.buffer.add_batch(**cols)
    q_hist = []
    for it in range(args.critic_steps):
        q_hist.append(upper.update(cfg.batch_upper, update_actor=False)["upper_q_loss"])
        if (it + 1) % 500 == 0 or it + 1 == args.critic_steps:
            print(f"  critic step {it + 1}/{args.critic_steps}: Q loss {np.mean(q_hist[-500:]):.4f}")

    with torch.no_grad():
        bc = {"train_agreement": bc_step(upper, S, M, K, 0.0)[1],
              "held_out_agreement": bc_step(upper, *held, 0.0)[1] if held else None,
              "mpc_hold_rate": hold_rate, "alpha": cfg.upper_init_alpha}
    torch.save({"upper": upper.state_dict(), "rows": cols, "config": cfg.to_dict(), "demos": demos, "bc": bc}, out)
    print(f"saved {out}: train agreement {bc['train_agreement']:.1%}"
          + (f", held-out {bc['held_out_agreement']:.1%}" if held else "")
          + f" (always holding would agree {hold_rate:.0%}), alpha {cfg.upper_init_alpha}")


if __name__ == "__main__":
    main()
