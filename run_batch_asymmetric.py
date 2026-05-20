"""Batch runner: actuated + multi_scale (1,0,0) + multi_scale (0.33,0.33,0.33),
asymmetric volume, LowPed, 5 phasing settings x 3 scenarios x 10 seeds = 150 runs.
"""
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection
from agent.mpc_agent import MpcAgent

SEEDS = [1212, 23423, 600, 700, 900, 1000, 2000]

NETWORK_TYPE = "single_intersection"
VOLUME_TYPE  = "asymmetric"

# (ped_phasing, ped_subsetting)
PHASING_SETTINGS = [
    ("Exclusive",  "permitted_right"),
]

SCENARIOS = [
    # ("actuated",    None),
    # ("multi_scale", (1, 0, 0)),
    ("multi_scale", (0.2, 0.4, 0.4)),
]


def run_scenario(control_type, weight, seed, ped_phasing, ped_subsetting):
    print(f"\n{'='*60}")
    print(f"Running: phasing={ped_phasing}/{ped_subsetting}, control={control_type}, weight={weight}, seed={seed}")
    print(f"{'='*60}")

    paras = set_parameters(NETWORK_TYPE, VOLUME_TYPE)
    paras["random_seed"] = seed
    paras["sumo_seed"]   = seed
    paras["ped_phasing"] = ped_phasing
    paras["ped_subsetting"] = ped_subsetting
    paras["poisson_gamma_pedestrian"] = 0.01  # LowPed
    # Recompute LPI/FDW based on updated phasing
    paras["ped_LPI"] = 0
    paras["ped_FDW"] = 0
    if ped_phasing in ("Concurrent", "Hybrid"):
        paras["yellow_time"] = 3
        paras["all_red_time"] = 2
        paras["ped_FDW"] = 5
        if ped_subsetting == "LPI":
            paras["ped_LPI"] = 5
        elif ped_subsetting == "delayed_turn":
            paras["ped_LPI"] = 5
    else:
        paras["yellow_time"] = 3
        paras["all_red_time"] = 0

    if weight is not None:
        paras["weight(Vehicles/Pedestrians/Bikes)"] = weight

    env = SingleIntersection(paras)
    env.start_sumo(False, control_type, NETWORK_TYPE, VOLUME_TYPE)

    phase_list_multi    = []
    duration_list_multi = []
    step = 0

    if control_type == "multi_scale":
        agent = MpcAgent(paras, "unified_four_legs_three_lanes")
        agent.clear_redundant_gams_files()

        while env.is_active():
            print(f"----Get network state at step {step}")
            network_state = env.get_state_cur_intersection(step)
            (_, phase_list_multi, duration_list_multi,
             should_update_signal, next_signal_phase, speed_commands) = (
                agent.get_control_commands(network_state, step)
            )
            env.apply_control_commands(should_update_signal, next_signal_phase, speed_commands)
            env.calculate_extra_metrics()
            env.move_one_step_forward()
            step += 1
            print("-------------------------------")

        env.close_sumo_simulation()
        env.performance_results(phase_list_multi, duration_list_multi,
                                NETWORK_TYPE, VOLUME_TYPE, control_type, step)
        agent.clear_redundant_gams_files()

    else:  # actuated
        while env.is_active():
            print(f"----step {step}")
            network_state = env.get_state_cur_intersection(step)
            env.calculate_extra_metrics()
            env.move_one_step_forward()
            step += 1

        env.close_sumo_simulation()
        env.performance_results([], [], NETWORK_TYPE, VOLUME_TYPE, control_type, step)


if __name__ == "__main__":
    total = len(PHASING_SETTINGS) * len(SCENARIOS) * len(SEEDS)
    done  = 0
    for ped_phasing, ped_subsetting in PHASING_SETTINGS:
        for control_type, weight in SCENARIOS:
            for seed in SEEDS:
                done += 1
                print(f"\n[{done}/{total}] phasing={ped_phasing}/{ped_subsetting}, control={control_type}, weight={weight}, seed={seed}")
                run_scenario(control_type, weight, seed, ped_phasing, ped_subsetting)

    print("\nAll LowPed batch runs complete.")
