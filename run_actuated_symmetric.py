"""
Batch runner: actuated control, symmetric volume.
Seeds: 10, 1000, 23423  (matching seeds present in Results/Concurrent-Symmetric/).
Ped levels: LowPed, MedPed, HighPed.

After each run, the result file is moved from Results/ into Results/Concurrent-Symmetric/.
"""

import os
import shutil
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection

PED_SCENARIOS = [
    ("LowPed",  0.01),
    ("MedPed",  0.04),
    ("HighPed", 0.08),
]

SEEDS = [10, 1000, 23423]

NETWORK_TYPE = "single_intersection"
VOLUME_TYPE  = "symmetric"
CONTROL_TYPE = "actuated"

DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Results", "Concurrent-Symmetric")


def run_scenario(ped_label, poisson_gamma, seed):
    print(f"\n{'='*60}")
    print(f"Running: actuated  |  {ped_label}  |  seed={seed}")
    print(f"{'='*60}")

    paras = set_parameters(NETWORK_TYPE, VOLUME_TYPE)
    paras["poisson_gamma_pedestrian"] = poisson_gamma
    paras["sumo_seed"]                = seed

    env = SingleIntersection(paras)
    env.start_sumo(False, CONTROL_TYPE, NETWORK_TYPE, VOLUME_TYPE)

    step = 0
    while env.is_active():
        print(f"----step {step}")
        network_state = env.get_state_cur_intersection(step)
        for inter_id in network_state.keys():
            env.pedestrian_actuation(inter_id)
        env.calculate_extra_metrics()
        env.move_one_step_forward()
        step += 1

    env.close_sumo_simulation()
    env.performance_results([], [], NETWORK_TYPE, VOLUME_TYPE, CONTROL_TYPE, step)

    # Move generated file to Results/Concurrent-Symmetric/
    generated = (
        f"Results/{CONTROL_TYPE}_penetration({paras['penetration']})"
        f"_EVratio({paras['ratio_ev']})_{ped_label}_seed({seed}).txt"
    )
    if os.path.exists(generated):
        os.makedirs(DEST_DIR, exist_ok=True)
        dest = os.path.join(DEST_DIR, os.path.basename(generated))
        shutil.move(generated, dest)
        print(f"  -> Moved to {dest}")
    else:
        print(f"  WARNING: expected output file not found: {generated}")


if __name__ == "__main__":
    total = len(PED_SCENARIOS) * len(SEEDS)
    done  = 0
    for ped_label, poisson_gamma in PED_SCENARIOS:
        for seed in SEEDS:
            done += 1
            print(f"\n[{done}/{total}]")
            run_scenario(ped_label, poisson_gamma, seed)

    print("\nAll actuated symmetric runs complete.")