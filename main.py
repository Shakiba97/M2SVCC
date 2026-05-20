## The entry point of the Multiscale SVCC algorithm.
import argparse
import traci
from configs.set_parameters import set_parameters
from environment.single_intersection import SingleIntersection
from agent.mpc_agent import MpcAgent


def main(network_type, volume_type, control_type, seed=None):
    print("----Get parameters...")
    paras = set_parameters(network_type, volume_type)
    if seed is not None:
        paras["random_seed"] = seed
        paras["sumo_seed"] = seed

    print("----Build single intersection environments...")
    env_single_intersection = SingleIntersection(paras)

    print("----Start SUMO...")
    env_single_intersection.start_sumo(True, control_type, network_type, volume_type)

    print("----Initializing the agent...")
    agent_unified_four_legs_three_lanes = MpcAgent(paras, "unified_four_legs_three_lanes")
    agent_unified_four_legs_three_lanes.clear_redundant_gams_files()

    phase_list_multi = []
    duration_list_multi = []
    step = 0
    # while step <100:
    while env_single_intersection.is_active():

        print(f"----Get network state at step {step}")
        network_state = env_single_intersection.get_state_cur_intersection(step)

        if control_type == "multi_scale":
            # print("----Get control commands from the agent")
            (next_global_step_to_re_solve_the_network, phase_list_multi, duration_list_multi, should_update_signal, next_signal_phase, speed_commands) = (
                agent_unified_four_legs_three_lanes.get_control_commands(
                    network_state, step
                )
            )
            env_single_intersection.apply_control_commands(
                should_update_signal, next_signal_phase, speed_commands
            )

        # elif control_type == "actuated":
        #     for inter_id in network_state.keys():
        #         env_single_intersection.pedestrian_actuation(inter_id)

            #env_single_intersection.pedestrian_movement_control()
        env_single_intersection.calculate_extra_metrics()
        env_single_intersection.move_one_step_forward()
        step += 1
        print(f"-------------------------------")

    env_single_intersection.close_sumo_simulation()
    env_single_intersection.performance_results(phase_list_multi, duration_list_multi, network_type, volume_type, control_type, step)
    agent_unified_four_legs_three_lanes.clear_redundant_gams_files()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=None, help="Random seed (overrides config default)")
    parser.add_argument("--control", type=str, default="multi_scale", help="Control type: multi_scale, actuated, fixed_time")
    parser.add_argument("--volume", type=str, default="asymmetric", help="Volume type: symmetric, asymmetric")
    args = parser.parse_args()
    main("single_intersection", args.volume, args.control, seed=args.seed)
    # control_type: "multi_scale", "actuated", "fixed_time"
    # volume_type: symmetric, asymmetric