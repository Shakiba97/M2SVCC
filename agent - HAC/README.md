# agent - HAC: Phase I hierarchical RL agent

Implementation of *Phase I Methodology v3* (8 Nov 2026): an upper SAC that picks one of
8 signal phases every 5 s and a lower SAC that commands each CAV's acceleration every
0.5 s through a phase-dependent CBF-QP filter, coupled by a world-model lane rollout
that gives each CAV a window-end target `s_ref`.

It runs on the same SUMO network, routes, pedestrian model and metric code as the MPC
(`agent/mpc_agent.py`) and actuated runs, and writes results files in the same format.

## Running

Use the `HAC-M2SVCC` conda env (it has torch, traci and sumolib; GAMS is not needed).

```bash
cd "agent - HAC"
PY=~/miniconda3/envs/HAC-M2SVCC/bin/python

$PY tests/test_cbf.py                  # barrier checks, no SUMO
$PY tests/test_world_model.py          # rollout checks, no SUMO

$PY stage0_world_model.py --run-name phase1 --episodes 6   # Stage 0: B_model + residuals
$PY train.py --run-name phase1                             # Phase A/B training
$PY evaluate.py --run-name phase1 --seeds 10 1000 10000 23423 30000
```

Any `HACConfig` field ([hac_config.py](hac_config.py)) can be overridden with
`--set name=value` (repeatable) or `--config file.json`. Outputs go to `runs/<run-name>/`:
`config.json`, `train_log.csv`, `checkpoints/`, `results/` and `eval/` (repo-format results
files), `world_model.pt`, `b_model.pt`, `world_model_validation.json`, and `sim/` (each run's SUMO route file and tripinfo/queue outputs; HAC does not touch
`environment/network_model/single_intersection.rou.xml`, which `main.py` uses).

Smoke-tested on short (120–180 s) simulations; a full 900 s episode takes several minutes
of wall time early in training (an untrained lower policy crawls).

### SUMO version and backend

Every run that is compared (HAC, MPC, actuated) should use **SUMO 1.26.0**, the system
install that `SUMO_HOME=/usr/share/sumo` points to. Earlier MPC/actuated outputs were made
with SUMO 1.24.0, so re-run the baselines on 1.26.0 before comparing. `sumo_version` in
[hac_config.py](hac_config.py) is the expected version; each run prints the version it got
and warns on a mismatch.

The HAC scripts run SUMO through **libsumo** by default: SUMO runs inside the Python process
instead of behind a TraCI socket, which made a training episode 3.1× faster with identical
results. libsumo has no GUI, so GUI runs use TraCI:

```bash
HAC_SUMO_BACKEND=traci $PY evaluate.py --run-name phase1 --set sumo_gui=true
```

The HAC env has `libsumo`, `traci`, `sumolib` and `sumo-data`, all 1.26.0. When upgrading these
with pip, check `$PY -c "import sumolib.net"` afterwards: replacing sumolib 1.24 deleted the new
version's files here, and `pip install --force-reinstall --no-deps <the four packages>==1.26.0` fixed it.

### MPC warm start (upper level only)

The upper SAC can start from the repo's MPC-MINLP signal policy instead of random phases.
The lower level is unchanged: random warm-up, then normal training.

```bash
# 1. Record: the MPC (slower-scale problem only) picks every phase; HAC writes the transitions.
#    Needs GAMS: install gamsapi (matching /opt/gams) into this env first.
$PY record_mpc_upper.py --run-name phase1 --episodes 6
# 2. Pre-train: clone the MPC's phases (actor), then fit the critic with the actor frozen.
$PY pretrain_upper_from_mpc.py --run-name phase1
# 3. Train on top of it.
$PY train.py --run-name phase1 --init-upper runs/phase1/upper_mpc_init.pt
$PY tests/test_upper_warm_start.py     # warm-start checks, no SUMO / GAMS
```

With `--init-upper`, the 5 random-phase episodes are skipped, α starts at `upper_init_alpha`
(0.05), the upper actor is frozen (critic-only updates) for the first
`upper_actor_freeze_episodes` (3), and the recorded MPC transitions are put in `B_upper` at
the start (`upper_init_preload`). Step 2 prints how often the cloned policy picks the MPC's
phase on a held-out episode, next to how often simply holding the current phase would.

## Where each part of the methodology lives

```
agent - HAC/
├── hac_config.py              all hyperparameters, grouped by section of the methodology
├── stage0_world_model.py      Stage 0 (Sec. 6.1)
├── train.py                   Phase A + B training loop (Sec. 6.2-6.3)
├── evaluate.py                deterministic evaluation, repo-format results
├── record_mpc_upper.py        MPC warm start 1/3: MPC phase decisions as upper transitions (GAMS)
├── pretrain_upper_from_mpc.py MPC warm start 2/3: behaviour cloning + critic pre-training
├── common_cli.py              command-line options shared by the scripts
├── tests/                     test_cbf.py, test_world_model.py, test_upper_warm_start.py (no SUMO)
└── hrl/
    ├── upper/                 UPPER LEVEL - signal controller
    │   ├── signal_plan.py     8-phase plan, A/B lookups (Eq. 5e-5f), clearance schedule, G_p (Eq. 6)
    │   ├── masking.py         PhaseTracker: G_p and min-green action mask (Sec. 5.3)
    │   ├── option.py          Option: one upper decision = [clearance] + 5 s window
    │   ├── state.py           S_upper (Sec. 3.1)
    │   ├── reward.py          r_veh / r_ped / r_bike sums (Sec. 7.1)
    │   ├── sac.py             UpperSAC: discrete, masked, SMDP-discounted (Sec. 6.3)
    │   ├── decision.py        phase selection and the upper transition at each option end
    │   └── mpc_teacher.py     MPC-MINLP phase teacher for the warm start
    ├── lower/                 LOWER LEVEL - CAV controller
    │   ├── state.py           S_lower (Sec. 3.3)
    │   ├── reward.py          energy models, r_lower, window-end target, potential shaping (Sec. 7.2-7.3)
    │   └── sac.py             LowerSAC: continuous, shared by all CAVs (Sec. 3.4, 6.3)
    ├── safety/
    │   └── cbf.py             CBF-QP filter: h_red + activation rule, h_green, h_stop, bounds (Sec. 5.1-5.2)
    ├── world_model/           COUPLING between the levels
    │   ├── rollout.py         LaneWorldModel: IDM, HDV residual, cyclist, CAV reference (Sec. 4.2-4.3)
    │   ├── planner.py         s_ref targets, predicted phases, upper lookahead (Sec. 4.1, 4.3, 4.5, 7.3)
    │   ├── data.py            ModelBuffer = B_model (Sec. 4.6)
    │   └── training.py        K-step unrolled loss, validation, discharge check (Sec. 4.4)
    ├── env/
    │   └── sumo_env.py        SUMO interface (subclass of the repo's SingleIntersection)
    ├── common/                paths, networks (mlp), ReplayBuffer (B_upper, B_lower), one-hot encoding
    ├── hac_loop.py            HACController.step: one SUMO step for both levels (Phase A, Sec. 6.2)
    └── runner.py              one episode, agent construction, checkpoints
```

`HACController` (hac_loop.py) combines `UpperDecisionMixin` (upper/decision.py) and
`WorldModelPlanner` (world_model/planner.py); each mixin's docstring lists the controller
attributes it uses.

## Decisions not fixed by the methodology

These are implementation choices; each is a config flag or a single place in the code.

1. **Phase changes and the 5 s window.** A change of phase needs 5 s yellow + 2 s all-red
   (the repo's Concurrent settings), which is longer than a window, while Sec. 6.2 requires
   phase-pure windows. One upper action is therefore an *option* = clearance (only when
   the phase changes) + a 5 s phase-pure window. The upper bootstrap is discounted by
   `gamma_up ** (duration / 5 s)` (SMDP), and the window-end target is set for the end of
   the option. Because the lower level also acts during the clearance, `S_lower` gets two
   extra features: `g_next` (ego link green in the selected phase) and `tau_clear`.
   `g_lane` is the ego link's actual right-of-way now, which is what gates `m_t`.
2. **CBF γ_c and the h_stop backstop.** The methodology does not give γ_c. `tests/test_cbf.py`
   shows that with leaders braking at b_max the headway barrier alone keeps gap ≥ d_0 at
   γ_c = 0.2 but not at 0.5 (1.81 m) or 1.0 (collision). Defaults: `gamma_c = 0.5` with the
   optional h_stop backstop on (`use_stop_backstop`), which keeps gap ≥ d_0 in that test.
   The red barrier reproduces Sec. 5.2: d_s = 0 overshoots by 0.14 m, d_s = 0.5 stops 0.36 m short.
3. **Gaps are bumper-to-bumper** everywhere (CBF, IDM, S_lower Δs); d_0 = 2 m (SUMO minGap).
   The target cap `s_lead − L_lead − d_0 − τ_h v` is the same quantity.
4. **r_ped** is read as Σ_m N_m · D_m (pedestrians waiting for crossing m × their mean wait).
   In Sec. 3.2 `q_m^p` denotes the binary crossing permission; reading it that way would make
   r_upper penalise serving pedestrians.
5. **r_upper integration.** `upper_reward_mode = "integrated"` (default) sums Σ q_j w_j Δt over
   the option; `"end_of_window"` uses the window-end values × duration, the literal formula.
6. **S_upper:** n_ped is a vector over the 4 crossings the 8 phases serve; `t_cycle` is the
   time since phase 1 (SUMO index 0) last started; `n_bike = 0`.
7. **CAV reference controller.** While GLOSA is active it handles the stop line, so its IDM
   term follows real leaders only (otherwise IDM's virtual stopped vehicle makes the reference
   brake to a stop, and GLOSA has no effect). A hard stop-line hold stays as a backstop.
8. **Signals are applied before control each step**, so the CBF sees the signal in force
   during the step it controls.
9. **Episode bookkeeping.** Lower transitions open at the end of the simulation are dropped
   (truncations without an observed next state). CAVs leaving the control zone are terminal.
10. **Ballistic SUMO update** (`ballistic = True`) so SUMO uses the CBF's kinematics.
    MPC/actuated comparisons should use the same setting.

## Departures from the methodology

1. **One HDV residual instead of T_red / T_green (Sec. 4.2).** `LaneWorldModel.res_hdv` corrects
   IDM in both signal regimes, with the `red_regime` flag as a 7th input feature. The residual's
   other inputs carry no signal state, so the flag is what lets it tell a queue under red from one
   under green. One net shares the car-following correction common to both regimes (e.g. the
   mean effect of Krauss `sigma = 1` dawdling) instead of learning it twice from half the data.
   The cyclist residual gets the same flag. World models saved with `res_red`/`res_green` no
   longer load.

## Known limitations

- **No cyclists in the network.** The cyclist model, residual and r_bike exist but receive no
  data. The follow-the-leader rule in `LaneWorldModel._bike_ftl` is a placeholder for
  M2SVCC Eq. 1–2.
- **No HDVs at the default penetration (1.0).** Stage 0 trains the HDV residuals on SUMO-driven
  vehicles from the actuated baseline (all vehicles are SUMO-driven there). Phase A
  fine-tuning only adds data when `penetration < 1` in `configs/set_parameters.py`.
- **SUMO safety stays on** (`cav_speed_mode = 31`). In smoke runs SUMO decelerated harder than
  the command on about 1–5% of lower steps. About half of those are at a red stop line,
  because SUMO stops 1.0 m short of the line and the CBF allows d_s = 0.5 m. The rest are
  junction yielding (not modelled by the CBF) and car following. Use `cav_speed_mode = 0` to
  test the CBF alone.
- **Teleports** (SUMO default `time-to-teleport` 300 s) move starving vehicles across the
  junction and hide starvation in the delay metrics. They are counted (`teleports`) and
  excluded from `red_light_violations`. Use `--set time_to_teleport=-1` to disable.
- **Yellow phase quirk.** In the repo's Concurrent program, the yellow of phase 2
  (SUMO index 10) does not include link 4 (East right turn), so that movement goes straight
  from green to red. `SignalPlan.links_without_yellow` reports it.
- **Pedestrian yield in the rollout** is approximate: a virtual stop at the first internal
  segment of a permitted turn while the conflicting crossing is green and had waiting
  pedestrians at the decision.
- **Weights and scales marked TUNE** in `hac_config.py` (w_e, λ_safe, w_g, λ_cbf, the upper
  reward scale, γ_c) need the sweeps described in Sec. 7.2.
- **Throughput.** Every vehicle is queried through individual TraCI calls; TraCI
  subscriptions would speed training up if wall time becomes a constraint.

## Comparing with the MPC and actuated results

Results files are named `hrl_penetration(1)_EVratio(0)_(α, β)_<Ped>_seed(N).txt` and contain
the same lines as the repo's results, plus `HAC …` safety lines. To include them in the
figure scripts under `Results/Concurrent figurs/`, add `hrl` to their
`(?:multi_scale|actuated)` regex alternatives.

Nothing outside this folder is modified. Vehicle arrivals use the repo's fixed
`random_seed = 1`, and only the SUMO seed varies, exactly as in the MPC and actuated runs,
so `seed(N)` means the same traffic in all three. Routes are generated the same way
`main.py` generates them, into `environment/network_model/single_intersection.rou.xml`; its
pedestrian rates follow the episode's pedestrian level, as with `main.py`.
