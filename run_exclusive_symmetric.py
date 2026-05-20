"""
Batch runner: M2SVCC with Exclusive pedestrian phasing, Symmetric volume.
Weights: (0.8,0.2) → (0.1,0.9)  [no (1.0,0.0)]
Ped levels: LowPed / MedPed / HighPed
Seeds: 10, 1000, 23423
Results moved to Results/Exclusive-Symmetric/
"""

import os
import shutil
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection
from agent.mpc_agent import MpcAgent

WEIGHTS = [
    (0.8, 0.2),
    (0.7, 0.3),
    (0.6, 0.4),
    (0.5, 0.5),
    (0.4, 0.6),
    (0.3, 0.7),
    (0.2, 0.8),
    (0.1, 0.9),
]

PED_SCENARIOS = [
    ("LowPed",  0.01),
    ("MedPed",  0.04),
    ("HighPed", 0.08),
]

SEEDS = [10, 1000, 23423]

NETWORK_TYPE = "single_intersection"
VOLUME_TYPE  = "symmetric"
CONTROL_TYPE = "multi_scale"
PED_PHASING  = "Exclusive"

DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Results", "Exclusive-Symmetric")


def run_scenario(weight, ped_label, poisson_gamma, seed):
    print(f"\n{'='*60}")
    print(f"Running: Exclusive | weight={weight} | {ped_label} | seed={seed}")
    print(f"{'='*60}")

    paras = set_parameters(NETWORK_TYPE, VOLUME_TYPE)
    paras["ped_phasing"]                 = PED_PHASING
    paras["weight(Vehicles/Pedestrians)"] = weight
    paras["poisson_gamma_pedestrian"]     = poisson_gamma
    paras["sumo_seed"]                    = seed

    env = SingleIntersection(paras)
    env.start_sumo(False, CONTROL_TYPE, NETWORK_TYPE, VOLUME_TYPE)

    agent = MpcAgent(paras, "unified_four_legs_three_lanes")
    agent.clear_redundant_gams_files()

    phase_list_multi    = []
    duration_list_multi = []
    step = 0
    while env.is_active():
        print(f"----step {step}")
        network_state = env.get_state_cur_intersection(step)
        (next_global_step, phase_list_multi, duration_list_multi,
         should_update_signal, next_signal_phase, speed_commands) = (
            agent.get_control_commands(network_state, step)
        )
        env.apply_control_commands(should_update_signal, next_signal_phase, speed_commands)
        env.calculate_extra_metrics()
        env.move_one_step_forward()
        step += 1

    env.close_sumo_simulation()
    env.performance_results(phase_list_multi, duration_list_multi,
                            NETWORK_TYPE, VOLUME_TYPE, CONTROL_TYPE, step)
    agent.clear_redundant_gams_files()

    # Move result file to Exclusive-Symmetric/
    generated = (
        f"Results/{CONTROL_TYPE}_penetration({paras['penetration']})"
        f"_EVratio({paras['ratio_ev']})_{PED_PHASING}_{weight}_{ped_label}_seed({seed}).txt"
    )
    if os.path.exists(generated):
        os.makedirs(DEST_DIR, exist_ok=True)
        dest = os.path.join(DEST_DIR, os.path.basename(generated))
        shutil.move(generated, dest)
        print(f"  -> Saved to {dest}")
    else:
        print(f"  WARNING: expected output file not found: {generated}")


if __name__ == "__main__":
    total = len(WEIGHTS) * len(PED_SCENARIOS) * len(SEEDS)
    done  = 0
    for weight in WEIGHTS:
        for ped_label, poisson_gamma in PED_SCENARIOS:
            for seed in SEEDS:
                done += 1
                print(f"\n[{done}/{total}]")
                run_scenario(weight, ped_label, poisson_gamma, seed)

    print("\nAll Exclusive-Symmetric runs complete.")
