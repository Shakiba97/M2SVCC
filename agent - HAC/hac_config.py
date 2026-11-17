"""Hyperparameters for the Phase I hierarchical (HAC-style) RL agent.

Section numbers refer to "Phase I Methodology v3" (8 Nov 2026).
Values marked TUNE are not fixed by the methodology and need a sweep.

Override any field from the command line with  --set name=value  (repeatable),
or with a JSON file passed as  --config file.json .
"""
import json
from dataclasses import asdict, dataclass


@dataclass
class HACConfig:
    # ---------------------------------------------------------------- run
    run_name: str = "phase1"
    episodes: int = 300
    base_seed: int = 0
    # Pedestrian Poisson rates sampled per episode (repo: 0.01 Low, 0.04 Med, 0.08 High).
    ped_demand_levels: tuple = (0.01, 0.04, 0.08)
    volume_type: str = "asymmetric"
    simulation_duration: int = 900          # s of vehicle/pedestrian demand (repo default)
    sumo_gui: bool = False                  # needs HAC_SUMO_BACKEND=traci (libsumo has no GUI)
    # SUMO version for every run that is compared (HAC, MPC, actuated); any other version warns.
    sumo_version: str = "1.26.0"
    # SUMO ballistic position update matches the kinematics used by the CBF and the
    # world model (Sec. 4.3, 5.2). Set False to use SUMO's default Euler update.
    ballistic: bool = True
    # SUMO speed mode for RL-controlled CAVs. 31 keeps SUMO's own safety checks as a
    # backstop (they are counted as "sumo_overrides"); 0 leaves safety to the CBF alone.
    cav_speed_mode: int = 31
    # SUMO default (also used by the repo's MPC/actuated runs). A vehicle stuck this long is
    # teleported across the junction, which hides starvation in the delay metrics; use -1 to
    # disable (episodes are then capped by run_episode's max_extra_seconds).
    time_to_teleport: int = 300
    torch_threads: int = 8
    device: str = "cpu"

    # ---------------------------------------------------------- timescales (Sec. 2)
    dt: float = 0.5                          # lower step = SUMO step [s]
    window: float = 5.0                      # upper window ΔT_s [s]
    yellow_time: float = 5.0                 # repo Concurrent setting
    all_red_time: float = 2.0                # repo Concurrent setting
    min_green: float = 5.0                   # Eq. 5i: no two transitions in a row -> one window
    ped_speed: float = 1.0                   # S_p in G_p (repo value) [m/s]

    # ------------------------------------------------------- vehicle limits (Sec. 5)
    a_max: float = 3.0                       # SUMO vType accel
    a_min: float = -4.5                      # = -b_max
    b_max: float = 4.5
    v_max: float = 13.0                      # speed limit / vType maxSpeed
    control_range: float = 200.0             # repo communication_range [m]
    L_exit: float = 30.0                     # lower episode ends this far past the stop line [m]
    # "symmetric": a_raw = tanh * a_max (Sec. 3.4).  "full": tanh mapped to [a_min, a_max].
    lower_action_range: str = "symmetric"

    # ------------------------------------------------------------- CBF (Sec. 5)
    # TUNE (0 < gamma_c <= 1). tests/test_cbf.py: with leaders braking at b_max the headway
    # barrier alone keeps gap >= d_0 at gamma_c = 0.2 but not at 0.5 (1.81 m) or 1.0 (collision).
    gamma_c: float = 0.5
    d_s: float = 0.5                         # standstill margin of h_red [m]
    tau_h: float = 1.0                       # safe time headway [s] (repo tau_cf)
    d_0: float = 2.0                         # bumper-to-bumper jam gap [m] (SUMO minGap)
    red_linear_approx: bool = False          # use a_red ≈ c0/c1 instead of the exact root
    use_stop_backstop: bool = True           # add h_stop (Sec. 5.2); needed for gap >= d_0 at gamma_c = 0.5
    a_lead_conservative: bool = False        # use a_lead = -b_max instead of a_lead_prev

    # ---------------------------------------------------- world model (Sec. 4)
    H_w: int = 3                             # windows rolled out
    idm_v0: float = 13.0
    idm_T: float = 1.0
    idm_s0: float = 2.0
    idm_delta: float = 4.0
    idm_b_comf: float = 1.67                 # repo comf_acc
    rollout_brake: float = 5.0               # braking limit inside the rollout (SUMO decel)
    glosa_tau: float = 2.0                   # speed-tracking relaxation time [s]
    bike_v_des: float = 5.0                  # cyclist placeholder (no cyclists in the network yet)
    bike_T: float = 1.0
    bike_s0: float = 1.0
    residual_hidden: int = 64
    residual_scale: float = 2.0              # max |residual| [m/s^2]
    wm_K: int = 10                           # K-step unrolled loss (Sec. 4.4)
    wm_lambda_v: float = 1.0
    wm_lr: float = 1e-3
    wm_batch: int = 64
    wm_pretrain_steps: int = 3000
    wm_finetune_every: int = 200             # N_model (env steps)
    wm_finetune_steps: int = 20
    wm_buffer_episodes: int = 20
    # Predicted phases for windows 2..H_w (Sec. 6.2): "policy" applies the upper SAC to the rolled-out
    # state once phase_prediction_warmup_episodes have passed (the current phase is held before
    # that); "hold" always holds the current phase.
    phase_prediction: str = "policy"
    phase_prediction_warmup_episodes: int = 30
    upper_lookahead: bool = False            # Sec. 4.5 optional extension
    lookahead_epsilon: float = 0.1

    # ------------------------------------------------------- rewards (Sec. 7)
    # Upper: r_upper = -scale * [alpha r_veh + beta r_ped + gamma r_bike] - lambda_cbf * sum corr^2
    alpha: float = 0.25                      # repo weight(Vehicles/Pedestrians) default
    beta: float = 0.75
    gamma_bike: float = 0.0
    upper_reward_scale: float = 1e-3         # TUNE
    upper_reward_mode: str = "integrated"    # "integrated" over the option, or "end_of_window"
    lambda_cbf: float = 0.01                 # TUNE
    preference_conditioned: bool = False     # append (alpha, beta, gamma) to S_upper and sample per episode
    # Lower (Guo et al. 2021 units: metres)
    w_f: float = 4.5                         # ICE fuel weight (Guo et al.)
    w_e: float = 0.15                        # EV energy weight, TUNE
    w_s: float = 1.0                         # intervention weight (Guo et al.)
    lambda_safe: float = 0.1                 # TUNE
    w_g: float = 0.05                        # window-end target weight, TUNE
    potential_shaping: bool = False          # Sec. 7.3 optional dense guide

    # ---------------------------------------------------------- SAC (Sec. 6.3)
    gamma_lo: float = 0.975                  # per 0.5 s step
    gamma_up: float = 0.95                   # per 5 s window (SMDP: gamma_up ** (duration / window))
    lr_lower: float = 3e-4
    lr_upper: float = 3e-4
    hidden: int = 256
    tau_target: float = 0.005
    batch_lower: int = 256
    batch_upper: int = 128
    buffer_lower: int = 1_000_000
    buffer_upper: int = 100_000
    lower_update_every: int = 1              # N_lower (env steps)
    lower_updates_per: int = 1
    upper_update_every: int = 4              # N_upper (upper decisions)
    upper_updates_per: int = 2
    lower_warmup_transitions: int = 5_000
    upper_warmup_transitions: int = 500
    upper_random_episodes: int = 5           # uniform legal phases while B_upper fills
    lower_target_entropy: float = -1.0
    upper_target_entropy_ratio: float = 0.5  # target = ratio * log(number of legal phases)
    checkpoint_every: int = 10

    # ------------------------------------- MPC warm start, upper level only
    # record_mpc_upper.py -> pretrain_upper_from_mpc.py -> train.py --init-upper <file>.
    # upper_init is set by --init-upper. With it: no random-phase episodes, critic-only upper
    # updates (actor frozen) for the first upper_actor_freeze_episodes, and the recorded MPC
    # transitions are put in B_upper at the start (upper_init_preload). The lower level is unchanged.
    upper_init: str = ""
    upper_init_alpha: float = 0.05           # starting alpha written by pretrain_upper_from_mpc.py
    upper_actor_freeze_episodes: int = 3
    upper_init_preload: bool = True

    def update(self, overrides: dict):
        for key, value in overrides.items():
            if not hasattr(self, key):
                raise KeyError(f"Unknown config field: {key}")
            current = getattr(self, key)
            if isinstance(current, bool) and isinstance(value, str):
                value = value.lower() in ("1", "true", "yes")
            elif isinstance(current, tuple) and isinstance(value, str):
                value = tuple(float(v) for v in value.split(","))
            elif isinstance(current, (int, float)) and not isinstance(current, bool) and isinstance(value, str):
                value = type(current)(value)
            setattr(self, key, value)
        return self

    def to_dict(self):
        return asdict(self)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)


def load_config(json_path=None, set_args=()):
    cfg = HACConfig()
    if json_path:
        with open(json_path) as f:
            cfg.update(json.load(f))
    overrides = {}
    for item in set_args or ():
        key, _, value = item.partition("=")
        overrides[key.strip()] = value.strip()
    return cfg.update(overrides)
