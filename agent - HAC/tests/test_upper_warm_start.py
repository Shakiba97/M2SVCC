"""Stand-alone checks of the upper-level MPC warm start (no SUMO, no GAMS).
Run: python tests/test_upper_warm_start.py"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hac_config import HACConfig  # noqa: E402
from hrl.upper.mpc_teacher import MpcPhaseTeacher, mpc_plan_to_phase  # noqa: E402
from hrl.upper.sac import UpperSAC  # noqa: E402
from pretrain_upper_from_mpc import bc_step  # noqa: E402

DIM, N = 20, 8


def test_plan_to_phase():
    assert mpc_plan_to_phase([3, 3, -1, 5]) == 3            # hold the current phase
    assert mpc_plan_to_phase([-1, 5, 5, 2]) == 5            # clearance, then switch to 5
    assert mpc_plan_to_phase([-1, -1]) is None


def fake_teacher(phase, epsilon=0.0):
    t = object.__new__(MpcPhaseTeacher)                      # skip the GAMS set-up
    t.epsilon, t.rng = epsilon, np.random.default_rng(0)
    t.counts = {"mpc": 0, "noise": 0, "fallback": 0}
    t.n_invalid = 0
    t._solve = lambda network_state: phase
    return t


def test_teacher_choice():
    mask = np.ones(N, np.float32)
    assert fake_teacher(4).choose(None, 2, mask) == (4, 4, "mpc")
    assert fake_teacher(None).choose(None, 2, mask) == (2, -1, "fallback")      # failed solve: hold
    t = fake_teacher(8)                                                         # exclusive ped phase
    assert t.choose(None, 2, mask) == (2, -1, "fallback") and t.n_invalid == 1
    k, teacher_k, source = fake_teacher(4, epsilon=1.0).choose(None, 2, mask)
    assert source == "noise" and teacher_k == 4                                 # label kept for BC


def filled_sac(n=600):
    torch.manual_seed(0)
    cfg = HACConfig()
    sac = UpperSAC(DIM, N, cfg)
    rng = np.random.default_rng(0)
    s = rng.normal(size=(n, DIM)).astype(np.float32)
    teacher = np.argmax(s[:, :N], axis=1)                    # a deterministic "MPC"
    sac.buffer.add_batch(s=s, a=teacher, r=rng.normal(size=n), s2=rng.normal(size=(n, DIM)),
                         done=np.zeros(n), duration=np.full(n, 5.0), mask=np.ones((n, N)), mask2=np.ones((n, N)))
    return sac, s, teacher


def test_critic_only_update_keeps_actor():
    sac, _, _ = filled_sac()
    actor0 = [p.detach().clone() for p in sac.actor.parameters()]
    q0 = [p.detach().clone() for p in sac.q1.parameters()]
    alpha0 = float(sac.log_alpha.detach())
    for _ in range(20):
        out = sac.update(64, update_actor=False)
    assert "upper_pi_loss" not in out
    assert all(torch.equal(a, b) for a, b in zip(actor0, sac.actor.parameters()))
    assert float(sac.log_alpha.detach()) == alpha0
    assert not all(torch.equal(a, b) for a, b in zip(q0, sac.q1.parameters()))


def test_behaviour_cloning_learns_teacher():
    sac, s, teacher = filled_sac()
    S, M, K = torch.as_tensor(s), torch.ones(len(s), N), torch.as_tensor(teacher)
    opt = torch.optim.Adam(sac.actor.parameters(), lr=1e-3)
    for _ in range(600):
        idx = torch.randint(0, len(s), (128,))
        loss, _ = bc_step(sac, S[idx], M[idx], K[idx], 0.0)
        opt.zero_grad()
        loss.backward()
        opt.step()
    _, acc = bc_step(sac, S, M, K, 0.0)
    assert acc > 0.9, acc


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok  {name}")
