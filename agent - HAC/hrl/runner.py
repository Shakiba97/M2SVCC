"""Episode runner shared by train.py, evaluate.py and stage0_world_model.py."""
import os

from .common import paths  # noqa: F401
from configs.set_parameters import set_parameters, set_volume_paras
from .hac_loop import HACController
from .env.sumo_env import HRLIntersection

PED_LABEL = {0.01: "LowPed", 0.04: "MedPed", 0.08: "HighPed"}


def make_paras(cfg, seed, ped_gamma):
    paras = set_parameters("single_intersection", cfg.volume_type)
    if paras["ped_phasing"] != "Concurrent":
        raise ValueError("configs/set_parameters.py must use ped_phasing = 'Concurrent' for Phase I")
    if cfg.simulation_duration != paras["simulation_duration"]:
        d = cfg.simulation_duration
        paras["simulation_duration"] = d
        paras["simulation_steps"] = d // paras["delta_T"]
        paras["time_interval_seconds"] = int(d / 6)
        set_volume_paras(paras)
    # Vehicle arrivals keep the repo's fixed random_seed (configs/set_parameters.py), so they are
    # identical to the MPC and actuated runs; like those runs, only the SUMO seed varies
    # (driver imperfection and pedestrian arrivals), and seed(N) in result names is the SUMO seed.
    paras["sumo_seed"] = seed
    paras["poisson_gamma_pedestrian"] = ped_gamma
    return paras


def results_name(control, paras, cfg):
    ped = PED_LABEL.get(paras["poisson_gamma_pedestrian"], f"ped{paras['poisson_gamma_pedestrian']}")
    w = f"_({cfg.alpha}, {cfg.beta})" if control == "hrl" else ""
    return (f"{control}_penetration({paras['penetration']})_EVratio({paras['ratio_ev']})"
            f"{w}_{ped}_seed({paras['sumo_seed']}).txt")


def run_episode(cfg, run_dir, label, seed, ped_gamma, *, lower=None, upper=None, world_model=None,
                training=False, episode=0, model_buffer=None, program="hrl", results_dir=None,
                max_extra_seconds=1800, printer=print, teacher_factory=None, record_upper=False):
    """teacher_factory(paras, env), called once SUMO runs, returns an upper-level teacher that
    picks the phases (record_mpc_upper.py). record_upper returns the upper transitions in
    stats["upper_rows"]."""
    paras = make_paras(cfg, seed, ped_gamma)
    env = HRLIntersection(paras, cfg, run_dir, label)
    env.start(program)
    ctrl = teacher = None
    if program == "hrl":
        teacher = teacher_factory(paras, env) if teacher_factory is not None else None
        ctrl = HACController(cfg, env.plan, lower, upper, world_model, training=training, episode=episode,
                             model_buffer=model_buffer, teacher=teacher, record_upper=record_upper)
    if model_buffer is not None:
        model_buffer.start_episode()
    max_steps = int((cfg.simulation_duration + max_extra_seconds) / cfg.dt)
    step = 0
    try:
        while env.is_active() and step < max_steps:
            network_state = env.get_state_cur_intersection(step)
            if ctrl is not None:
                ctrl.step(env, network_state)
            elif model_buffer is not None:
                obs = env.observe()
                model_buffer.add_frame(obs["ids"], obs["lane"], obs["x"], obs["v"], obs["L"], env.frame_types(obs),
                                       obs["code"], obs["wait"])
            env.calculate_extra_metrics()
            env.simulation_step()
            step += 1
        stats = ctrl.finish(env) if ctrl is not None else {"collisions": env.stats["collisions"],
                                                           "teleports": env.stats["teleports"]}
    finally:
        env.close()
        if teacher is not None:
            teacher.close()
    if ctrl is not None and record_upper:
        stats["upper_rows"] = ctrl.demo_rows
    if teacher is not None:
        stats.update(teacher.summary())
    results_dir = results_dir or os.path.join(run_dir, "results")
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, results_name("hrl" if program == "hrl" else program, paras, cfg))
    safety = {k: stats[k] for k in ("red_light_violations", "collisions", "teleports", "cbf_infeasible",
                                    "cbf_intervention_rate", "min_gap", "gap_below_d0_new_leader",
                                    "gap_below_d0_same_leader") if k in stats}
    env.write_results(path, extra={f"HAC {k}": v for k, v in safety.items()})
    stats.update({
        "seed": seed, "ped_gamma": ped_gamma, "sim_steps": step,
        "veh_waiting_time": env.waiting_time_avg, "veh_time_loss": env.lost_time_avg,
        "ped_time_loss": env.lost_time_avg_ped, "queue_m": env.queue_avg,
        "rt_conflicts": env.right_turn_conflicts_measure(),
        "fuel_cav_mg": env.fuel_total_cav_external_model / max(len(paras["veh_id_with_ev"]["cav_ice"]), 1),
        "results_file": path,
    })
    return stats


def make_plan(cfg):
    from environment.sumo_network_reader import SumoNetworkReader
    from .upper.signal_plan import SignalPlan
    paras = make_paras(cfg, 0, cfg.ped_demand_levels[0])
    return SignalPlan(SumoNetworkReader(paras).read())


def build_agents(cfg):
    import torch
    from .lower.sac import LowerSAC
    from .lower.state import LOWER_DIM
    from .upper.sac import UpperSAC
    from .upper.signal_plan import NUM_PHASES
    from .upper.state import upper_dim
    from .world_model.rollout import LaneWorldModel
    torch.set_num_threads(cfg.torch_threads)
    plan = make_plan(cfg)
    return plan, LowerSAC(LOWER_DIM, cfg), UpperSAC(upper_dim(cfg, plan), NUM_PHASES, cfg), LaneWorldModel(cfg)


def save_checkpoint(path, lower, upper, world_model, cfg, episode):
    import torch
    torch.save({"lower": lower.state_dict(), "upper": upper.state_dict(), "world_model": world_model.state_dict(),
                "config": cfg.to_dict(), "episode": episode}, path)


def load_checkpoint(path, lower=None, upper=None, world_model=None):
    import torch
    ck = torch.load(path, weights_only=False)
    if lower is not None:
        lower.load_state_dict(ck["lower"])
    if upper is not None:
        upper.load_state_dict(ck["upper"])
    if world_model is not None and "world_model" in ck:
        world_model.load_state_dict(ck["world_model"])
    return ck
