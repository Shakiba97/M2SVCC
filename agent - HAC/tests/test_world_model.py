"""Stand-alone checks of the lane rollout (no SUMO). Run: python tests/test_world_model.py"""
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hac_config import HACConfig  # noqa: E402
from hrl.upper.signal_plan import GREEN, RED  # noqa: E402
from hrl.world_model.rollout import BIG, TYPE_CAV, TYPE_HDV, LaneState, LaneWorldModel  # noqa: E402


def make_lane(xs, vs, types):
    n = len(xs)
    t = lambda a, d=torch.float32: torch.tensor([a], dtype=d)  # noqa: E731
    return LaneState(t(xs), t(vs), torch.full((1, n), 5.0), t(types, torch.long),
                     torch.ones((1, n), dtype=torch.bool), torch.zeros((1, n)))


def test_red_then_green_discharge():
    cfg = HACConfig()
    wm = LaneWorldModel(cfg)
    st = make_lane([-20.0, -40.0, -60.0, -80.0], [13.0] * 4, [TYPE_HDV] * 4)
    T_red, T_green = 40, 40
    codes = torch.tensor([RED] * T_red + [GREEN] * T_green).view(-1, 1, 1).expand(-1, 1, 4)
    ya = torch.zeros(codes.shape, dtype=torch.bool)
    with torch.no_grad():
        xs, vs, _ = wm.rollout(st, codes, ya, torch.full((1, 4), BIG), torch.full((1, 4), BIG), use_glosa=False)
    front_at_red_end = xs[T_red, 0, 0].item()
    assert -cfg.idm_s0 - 1.0 <= front_at_red_end <= 0.0, front_at_red_end
    gaps = xs[:, 0, :-1] - 5.0 - xs[:, 0, 1:]
    assert gaps.min().item() >= -1e-4, "road users overlapped"
    crossed = (xs[-1, 0] >= 0).sum().item()
    print(f"queue held {-front_at_red_end:.2f} m before the line on red; {crossed}/4 crossed after 20 s of green; "
          f"min gap {gaps.min().item():.2f} m")
    assert crossed >= 3


def test_glosa_coasts_on_red():
    cfg = HACConfig()
    wm = LaneWorldModel(cfg)
    st = make_lane([-150.0], [13.0], [TYPE_CAV])
    codes = torch.tensor([RED] * 30 + [GREEN] * 30).view(-1, 1, 1)
    ya = torch.zeros(codes.shape, dtype=torch.bool)
    green_start = torch.full((1, 1), 15.0)
    with torch.no_grad():
        xs, vs, _ = wm.rollout(st, codes, ya, torch.full((1, 1), BIG), green_start, use_glosa=True)
        xs2, vs2, _ = wm.rollout(st, codes, ya, torch.full((1, 1), BIG), green_start, use_glosa=False)
    print(f"CAV speed when green starts: GLOSA {vs[30, 0, 0]:.2f} m/s vs IDM-only {vs2[30, 0, 0]:.2f} m/s")
    assert vs[30, 0, 0] > vs2[30, 0, 0] + 1.0, "GLOSA reference should arrive moving instead of stopping"


def test_gradients_reach_residuals():
    cfg = HACConfig()
    wm = LaneWorldModel(cfg)
    st = make_lane([-10.0, -30.0], [5.0, 8.0], [TYPE_HDV, TYPE_HDV])
    codes = torch.full((10, 1, 2), GREEN)
    ya = torch.zeros(codes.shape, dtype=torch.bool)
    xs, vs, _ = wm.rollout(st, codes, ya, torch.full((1, 2), BIG), torch.full((1, 2), BIG), use_glosa=False)
    xs[-1].sum().backward()
    g = sum(p.grad.abs().sum().item() for p in wm.res_hdv.parameters() if p.grad is not None)
    assert g > 0, "no gradient reached the HDV residual"
    print("gradients reach the HDV residual through the 10-step rollout")


def test_residual_sees_regime():
    """A follower queued behind a real leader differs between red and green only in the regime flag."""
    cfg = HACConfig()
    wm = LaneWorldModel(cfg)
    st = make_lane([-10.0, -17.0], [0.0, 0.0], [TYPE_HDV] * 2)   # follower 2 m behind a stopped leader
    no_yield, yield_x = torch.zeros((1, 2), dtype=torch.bool), torch.full((1, 2), BIG)
    f = {}
    for name, code in (("red", RED), ("green", GREEN)):
        red = wm.red_regime(st, torch.full((1, 2), code))
        gap, v_l, has_l, virt = wm._leaders(st, red, no_yield, yield_x)
        f[name] = wm.features(st, gap, v_l, has_l, virt, red, cfg.idm_v0)[0, 1]
    differing = (f["red"] != f["green"]).nonzero().flatten().tolist()
    assert differing == [6], f"follower features should differ only in the regime flag, got {differing}"
    torch.manual_seed(0)
    torch.nn.init.normal_(wm.res_hdv.net[-1].weight)
    with torch.no_grad():
        r_red, r_green = wm.res_hdv(f["red"]).item(), wm.res_hdv(f["green"]).item()
    assert r_red != r_green
    print(f"queued follower: residual {r_red:+.3f} m/s^2 on red vs {r_green:+.3f} on green (random last layer)")


if __name__ == "__main__":
    test_red_then_green_discharge()
    test_glosa_coasts_on_red()
    test_gradients_reach_residuals()
    test_residual_sees_regime()
    print("all world-model tests passed")
