"""Upper reward terms (Sec. 7.1).

r_upper = -scale [alpha r_veh + beta r_ped + gamma r_bike] - lambda_cbf sum (a_raw - a*)^2
is assembled in upper/decision.py (UpperDecisionMixin._finish_option) from these sums,
accumulated over the option in hac_loop.py.
"""
import numpy as np


def upper_cost_terms(lane_q, lane_w, ped_n, ped_d, bike_n=None, bike_d=None):
    """Instantaneous sums  sum_j q_j w_j,  sum_m N_m D_m,  sum_n N_n D_n  (Sec. 7.1).

    q_j vehicles halting on lane j, w_j their mean waiting time; N_m pedestrians waiting
    for crossing m, D_m their mean waiting time. Cyclists are absent from the network.
    """
    veh = float(np.sum(np.asarray(lane_q) * np.asarray(lane_w)))
    ped = float(np.sum(np.asarray(ped_n) * np.asarray(ped_d)))
    bike = float(np.sum(np.asarray(bike_n) * np.asarray(bike_d))) if bike_n is not None else 0.0
    return veh, ped, bike
