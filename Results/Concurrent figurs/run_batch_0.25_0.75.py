"""Batch runner for all (0.25, 0.75) weight scenarios: Low/Med/High ped demand x all seeds."""
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection
from agent.mpc_agent import MpcAgent

SCENARIOS = [
    # (ped_label, poisson_gamma, seed)
    # HighPed
    ("HighPed", 0.08, 10),
    ("HighPed", 0.08, 1000),
    ("HighPed", 0.08, 10000),
    ("HighPed", 0.08, 20000),
    ("HighPed", 0.08, 23423),
    ("HighPed", 0.08, 30000),
    # LowPed
    ("LowPed", 0.01, 10),
    ("LowPed", 0.01, 1000),
    ("LowPed", 0.01, 10000),
    ("LowPed", 0.01, 20000),
    ("LowPed", 0.01, 23423),
    ("LowPed", 0.01, 30000),
    # MedPed
    ("MedPed", 0.04, 10),
    ("MedPed", 0.04, 1000),
    ("MedPed", 0.04, 10000),
    ("MedPed", 0.04, 20000),
    ("MedPed", 0.04, 23423),
    ("MedPed", 0.04, 30000),
]

WEIGHT = (0.8, 0.2)
NETWORK_TYPE = "single_intersection"
VOLUME_TYPE = "asymmetric"
CONTROL_TYPE = "multi_scale"


def run_scenario(ped_label, poisson_gamma, seed):
    print(f"\n{'='*60}")
    print(f"Running: weight={WEIGHT}, {ped_label}, seed={seed}")
    print(f"{'='*60}")

    paras = set_parameters(NETWORK_TYPE, VOLUME_TYPE)
    paras["weight(Vehicles/Pedestrians)"] = WEIGHT
    paras["poisson_gamma_pedestrian"] = poisson_gamma
    paras["sumo_seed"] = seed

    env = SingleIntersection(paras)
    env.start_sumo(False, CONTROL_TYPE, NETWORK_TYPE, VOLUME_TYPE)

    agent = MpcAgent(paras, "unified_four_legs_three_lanes")
    agent.clear_redundant_gams_files()

    phase_list_multi = []
    duration_list_multi = []
    step = 0
    while env.is_active():
        print(f"----Get network state at step {step}")
        network_state = env.get_state_cur_intersection(step)

        (next_global_step, phase_list_multi, duration_list_multi,
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
                            NETWORK_TYPE, VOLUME_TYPE, CONTROL_TYPE, step)
    agent.clear_redundant_gams_files()


if __name__ == "__main__":
    for ped_label, poisson_gamma, seed in SCENARIOS:
        run_scenario(ped_label, poisson_gamma, seed)

    print("\nAll scenarios complete.")
