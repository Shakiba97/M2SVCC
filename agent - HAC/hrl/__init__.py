"""Phase I hierarchical RL agent (upper SAC signal controller, lower SAC CAV controller).

Layout (one sub-package per part of the methodology's architecture):
    upper/        signal controller: phase plan, masking, S_upper, r_upper, discrete SAC, decisions
    lower/        CAV controller: S_lower, r_lower and window-end target, continuous SAC
    safety/       CBF-QP filter
    world_model/  lane rollout, B_model, residual training, planner (targets, phase prediction, lookahead)
    env/          SUMO interface
    common/       paths, networks, replay buffer, encodings
    hac_loop.py   one SUMO step for both levels (Phase A of Sec. 6.2)
    runner.py     one episode, agent construction, checkpoints
"""
from .common import paths  # noqa: F401  (puts the repo root on sys.path before anything imports it)
