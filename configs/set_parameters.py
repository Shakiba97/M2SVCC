## The topology settings for a single intersection
import collections
import math

def set_common_paras(paras):
    ## Optimization Model parameters
    # Prediction horizon of the MPC problem, i.e., T^{s}.
    paras["num_predict_steps"] = 6
    # Number of slower steps used to construct the faster scale problem, i.e., h in Equation (24).
    paras["faster_steps_from_slower"] = 3
    # Length of each slower step, i.e., \Delta T_{s}.
    paras["delta_T"] = 5
    # Length of each faster step, i.e., \Delta T_{f}.
    paras["delta_T_faster"] = 0.5
    # Time headway of the car-following model, i.e., \tau_{h}.
    paras["tau_cf"] = 1
    # Safety distance of the car-following model, i.e., d_{0}.
    paras["d_0_cf"] = 7
    # Discount ratio used to stabilize the MPC problem.
    paras["discount_ratio"] = 0.95
    # weighting factor for each mode (for the slower scale problem optimization)(must add up to 1)
    paras["weight(Vehicles/Pedestrians)"] = (0.5, 0.5)

    ## IDM model parameters, see Equation (11) in the second paper.
    # Maximum acceleration that the vehicles can reach, in m/s^{2}.
    paras["max_acc"] = 3.5
    # Minimum acceleration, i.e., the maximum braking capability, in m/s^{2}.
    paras["min_acc"] = -5
    # Comfortable acceleration for each vehicle.
    paras["comf_acc"] = 1.67
    # The constant acceleration exponent.
    paras["delta_idm"] = 4

    ## Simulation parameters.
    # Penetration rate of CAVs.
    paras["penetration"] = 1
    # Concurrent or Exclusive Pedestrian phasing
    paras["ped_phasing"] = "Concurrent" #"Concurrent" or "Exclusive"
    # Random seed used to generate the volume.
    paras["random_seed"] = 1
    # simulation duration.
    paras["simulation_duration"] = 900
    # Simulation steps.
    paras["simulation_steps"] = paras["simulation_duration"] // paras["delta_T"]
    # Signal yellow time added between conflicting phases.
    if paras["ped_phasing"] == "Concurrent":
        paras["yellow_time"] = 5
        paras["all_red_time"] = 2
    else:
        paras["yellow_time"] = 3
        paras["all_red_time"] = 0
    # Speed limit of all roads, in m/s.
    paras["speed_limit"] = 13
    # Vehicle length.
    paras["vehicle_length"] = 5
    # Communication range, in m.
    paras["communication_range"] = 200
    # We simulate the volumes in a wave feature. This parameter represents the half-period of such waves. In seconds.
    paras["time_interval_seconds"] = int(paras["simulation_duration"]/6)

    # The lowest volume, in veh/h.
    paras["low_volume_veh"] = 400
    # The highest volume, in veh/h.
    paras["high_volume_veh"] = 600
    # Left turn and right turn ratios. We only explicitly set the main road, i,e, through traffic, volumes. The volume of other movements are set as ratios to their main movements.
    paras["left_right_ratio"] = 1 / 6
    # Ratio of Electric Vehicles (between 0 and 1)
    paras["ratio_ev"] = 0
    # Poisson gamma for pedestrian demand
    paras["poisson_gamma_pedestrian"] = 0.04 # high:0.08 medium=0.04 low=0.01
    paras["ped_demand_symmetry"] = "Asymmetric" # Asymmetric or Symmetric pedestrian demand

    ## pedestrian parameters:
    paras['ped_speed']=1 #m/s average speed assumed for pedestrians


def set_volume_paras(paras):
    low_volume = paras["low_volume_veh"]
    high_volume = paras["high_volume_veh"]
    time_interval_seconds = paras["time_interval_seconds"]
    num_interval = (
        paras["simulation_steps"] * paras["delta_T"] // paras["time_interval_seconds"]
    )
    volume_type = paras["volume_type"]

    if volume_type == "symmetric":
        paras["depart_rate"] = {
            "time_interval": time_interval_seconds,
            "vol_we_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ew_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ns_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_sn_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
        }
    elif volume_type == "asymmetric":
        paras["depart_rate"] = {
            "time_interval": time_interval_seconds,
            "vol_we_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ew_main": [
                low_volume / 2
                + (high_volume / 2 - low_volume / 2) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_ns_main": [
                low_volume / 2
                + (high_volume / 2 - low_volume / 2) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
            "vol_sn_main": [
                low_volume + (high_volume - low_volume) / (num_interval - 1) * i
                for i in range(num_interval)
            ],
        }
    else:
        raise TypeError("Unknown volume type!")

def set_parameters(network_type, volume_type):
    """Get the parameters for the entire simulation.
    Args:
        network_type: Should be one of (single_intersection, corridor, 4_4_network)
    Returns:
        paras: The parameters used to build the SUMO/MPC models, and all about this project.
    """
    paras = collections.defaultdict()
    paras["network_type"] = network_type
    paras["volume_type"] = volume_type

    set_common_paras(paras)
    set_volume_paras(paras)
    return paras
