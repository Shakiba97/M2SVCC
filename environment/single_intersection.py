## The single intersection environment.
import random

import traci
import collections
import os
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import math

from sumolib import checkBinary

from .sumo_network_reader import SumoNetworkReader
from .sumo_routes_generator import SumoRoutesGenerator


class SingleIntersection:
    def __init__(self, paras):
        self.paras = paras
        print("--------Building SUMO network...")
        network_reader = SumoNetworkReader(self.paras)
        self.network_graph = network_reader.read()
        print("--------SUMO network built.")

        print("--------Generating vehicles and routes")
        routes_generator = SumoRoutesGenerator(self.paras)   ## for a generalized case, we assume that the route file is already given to us
        ## TODO: generalize the route generator
        cav_ids, hdv_ids, veh_id_with_ev = routes_generator.generate_routes_for_single_intersection()
        self.paras["ped_ids"] = set()
        self.paras["bike_ids"] = set()
        self.paras["cav_ids"] = cav_ids["all"]
        self.paras["hdv_ids"] = hdv_ids["all"]
        self.paras["veh_id_with_ev"] = veh_id_with_ev
        self.fuel_total_cav_external_model = 0
        self.fuel_total_hdv_external_model = 0
        self.power_total_cav_external_model = 0
        self.power_total_hdv_external_model = 0
        self.fuel_total_cav_sumo = 0
        self.fuel_total_hdv_sumo = 0
        self.phase_list_fix_act =[]
        self.dur_list_fix_act =[]
        self.greenTimeSofar = {}
        self.next_time_to_change_to_actuated = {}
        self.stopped_peds = set()
        # ## TODO: generalize self.cross_map needed for Actuation part
        # self.cross_map = {':1_c0': 'N', ':1_c2': 'E', ':1_c4': 'S', ':1_c5': 'W', ':1_c3': 'NWSE', ':1_c1': 'NESW'}
        self.map_diag = {}
        for inter_id in self.network_graph:
            self.map_diag.setdefault(inter_id, {})
            if self.paras["ped_phasing"] == "Exclusive" or self.paras["ped_phasing"] == "Hybrid":
                for walkingarea in self.network_graph[inter_id]["walkingarea"]:
                    for crossing in self.network_graph[inter_id]["walkingarea"][walkingarea]["adjacent"]:
                        if self.network_graph[inter_id]["crossing"][crossing]["phasing"] == "Diagonal":
                            self.map_diag[inter_id].setdefault(walkingarea, set())
                            self.map_diag[inter_id][walkingarea].add(crossing)
            else:
                for walkingarea in self.network_graph[inter_id]["walkingarea"]:
                    for crossing in self.network_graph[inter_id]["walkingarea"][walkingarea]["adjacent"]:
                        if self.network_graph[inter_id]["crossing"][crossing]["phasing"] == "Straight":
                            self.map_diag[inter_id].setdefault(walkingarea, set())
                            self.map_diag[inter_id][walkingarea].add(crossing)
        self.cur_phase = {}
        self.previous_phase = {}
        self.yellow_duration=0
        self.noWALK=0
        self.clearance = False
        self.LPI = 0
        self.LPI_on = False
        self.phase_avg = None
        self.phase_ntimes = None
        self.right_turn_conflicts = {}
        self.next_pos={}
        self.next_spd={}
        print("--------Vehicles and routes generated.")

    def start_sumo(self, show_gui, control_type, network_type, volume_type):
        penetration = self.paras["penetration"]
        pedestrian_phasing = self.paras["ped_phasing"]
        pedestrian_subsetting = self.paras["ped_subsetting"]

        ## GUI setting.
        if show_gui:
            sumoBinary = checkBinary("sumo-gui")
        else:
            sumoBinary = checkBinary("sumo")

        if control_type not in ["multi_scale", "actuated", "fixed_time"]:
            raise TypeError("unknown control_type!")

        ## Check if the SUMO model exists or not.
        model_dir = os.path.dirname(os.path.realpath(__file__)) + "/network_model"
        if not os.path.exists(model_dir):
            raise TypeError("Network model is not built yet.")
        if control_type == "fixed_time":
            model_file_name = (
                model_dir + "/" + network_type + "_" + control_type + ".sumocfg"
            )
        elif control_type == "actuated":
            model_file_name = (
                    model_dir + "/" + network_type + "_" + control_type + "_" + pedestrian_phasing + "_(" + pedestrian_subsetting + ")" + ".sumocfg"
            )
        else:
            if pedestrian_phasing=="Exclusive":
                model_file_name = (
                    model_dir + "/" + network_type + "_" + control_type + "_" + pedestrian_phasing + ".sumocfg"
                )
            elif pedestrian_phasing=="Concurrent":
                model_file_name = (
                    model_dir + "/" + network_type + "_" + control_type + "_" + pedestrian_phasing  + "_(" + pedestrian_subsetting + ")" + ".sumocfg"
                )



        ## Create folder to store simulation data.
        data_dir = os.path.dirname(os.path.realpath(__file__)) + "/simulation_data"
        if network_type in ["single_intersection", "corridor", "4_4_network"]:
            data_dir_next = data_dir + "/" + network_type
        else:
            raise TypeError("unknown traffic network!")
        if not os.path.exists(data_dir_next):
            os.mkdir(data_dir_next)

        queue_file = (
            data_dir_next
            + "/queues_"
            + control_type
            + "_"
            + volume_type
            + "_pene_"
            + str(int(penetration * 100))
            + ".xml"
        )
        trip_file = (
            data_dir_next
            + "/tripinfo_"
            + control_type
            + "_"
            + volume_type
            + "_pene_"
            + str(int(penetration * 100))
            + ".xml"
        )
        SSM_file = (
            data_dir_next
            + "/SSMinfo_"
            + control_type
            + "_pene_"
            + str(int(penetration * 100))
            + ".xml"
        )

        ## Start SUMO.
        traci.start(
            [
                sumoBinary,
                "-c",
                model_file_name,
                "--start",
                "--queue-output",
                queue_file,
                "--tripinfo-output",
                trip_file,
                '--device.ssm.file=' + SSM_file,
                "--step-length",
                "0.5",
            ]
        )
        if control_type=="actuated":
            traci.trafficlight.setProgram("1", "actuated_program")

    def is_active(self):
        return traci.simulation.getMinExpectedNumber() > 0

    def get_state_cur_intersection(self, cur_step):
        ## Get the parameters needed to get the network state.
        traffic_graph = self.network_graph
        speed_limit = self.paras["speed_limit"]
        delta_T = self.paras["delta_T"]
        delta_T_faster = self.paras["delta_T_faster"]
        num_predict_steps = self.paras["num_predict_steps"]
        cav_ids = self.paras["cav_ids"]

        ## TODO: right turning conflicts is still hard-coded
        right_turning_vehs=["WS", "SE", "EN", "NW"]
        car_bike_gap = self.paras["minGap"]["Car"]/ self.paras["minGap"]["Bike"]


        self.network_state = collections.defaultdict()
        ## Iterate through each intersection.
        for inter_id in traffic_graph:
            ## State representations for each intersection
            signal_phase = int(traci.trafficlight.getPhase(str(inter_id)))
            self.phase_list_fix_act.append(signal_phase)
            self.dur_list_fix_act.append(cur_step)

            pos_vehicles, speed_vehicles, wt_vehicles, veh_id, veh_len, minGap = [], [], [], [], [], []
            pos_vehicles2, speed_vehicles2, wt_vehicles2, veh_id2 = [], [], [], []
            arrival_times, arrival_times2 = [], []
            num_vehicles_max = 0
            types=[]
            lane_id = []
            bike_lane_id=[]
            lane_length = []
            sidewalk_id=[]
            cross_demand_current={}
            cross_demand_all_steps={}
            crossing_number_map={}
            bike_demand={}
            delta_T = self.paras["delta_T"]
            delta_T_faster = self.paras["delta_T_faster"]
            num_predict_steps = self.paras["num_predict_steps"]
            bike_map={}
            i=1
            for crossing in self.paras["network_graph"][inter_id]["crossing"]:
                crossing_number_map[crossing]= i
                i +=1
            for cc in self.paras["network_graph"][inter_id]["crossing"]:
                cross_demand_current[cc] = set()  ## demand on different crossings of the current step
                for i in range(1, num_predict_steps +2):
                    cross_demand_all_steps.setdefault(crossing_number_map[cc], {})
                    cross_demand_all_steps[crossing_number_map[cc]][i]=set()
            for index, bikelane in enumerate(self.paras["network_graph"][inter_id]["incoming_bike"]):
                for i in range(1, num_predict_steps + 2):
                    bike_demand.setdefault(index+1, {})
                    bike_demand[index+1][i]=set()
            communication_range = self.paras["communication_range"]
            ## Get static information.
            for lane in traffic_graph[inter_id]["incoming_veh"]:
                lane_id.append(lane)
                lane_length.append(traffic_graph[inter_id]["incoming_veh"][lane]["length"])
            for sidewalk in traffic_graph[inter_id]["incoming_ped"]:
                sidewalk_id.append(sidewalk)
            sidewalk_id.extend(list(traffic_graph[inter_id]["walkingarea"]))
            for bikelane in traffic_graph[inter_id]["incoming_bike"]:
                bike_lane_id.append(bikelane)

            ## Get pedestrian info (Pedestrian Demand at current step):
            for i in range(len(sidewalk_id)):
                peds_lane = traci.edge.getLastStepPersonIDs(sidewalk_id[i]) #Get the IDs of the pedestrians on the lane
                self.paras["ped_ids"] = self.paras["ped_ids"].union(set(peds_lane))

                for j in range(len(peds_lane)):
                    pos=traci.person.getLanePosition(peds_lane[j])
                    speed=traci.person.getSpeed(peds_lane[j])
                    waiting_time=traci.person.getWaitingTime(peds_lane[j])
                    route=traci.person.getEdges(peds_lane[j],)
                    next_edge=traci.person.getNextEdge(peds_lane[j])
                    next_next_edge = None
                    current_edge=sidewalk_id[i]
                    if next_edge in self.paras["network_graph"][inter_id]["crossing"]:
                        if next_edge in self.map_diag[inter_id][current_edge]:
                            cross_demand_current[next_edge].add(peds_lane[j])
                            cross_demand_all_steps[crossing_number_map[next_edge]][1].add(peds_lane[j])
                        else:
                            for cross in self.map_diag[inter_id][current_edge]:
                                cross_demand_current[cross].add(peds_lane[j])
                                cross_demand_all_steps[crossing_number_map[cross]][1].add(peds_lane[j])
                    if next_edge in self.paras["network_graph"][inter_id]["walkingarea"] and speed!=0:
                        for cross in self.map_diag[inter_id][next_edge]:
                            if cross in route:
                                next_next_edge = cross
                                for step in range(2, num_predict_steps + 2):
                                    sidewalk_length = self.paras["network_graph"][inter_id]["incoming_ped"][current_edge]["length"]
                                    if (sidewalk_length - pos) / speed <= (step - 1) * delta_T and (sidewalk_length - pos) / speed >= (step - 2) * delta_T:
                                        cross_demand_all_steps[crossing_number_map[next_next_edge]][step].add(peds_lane[j])
                                        traci.person.setColor(peds_lane[j], (255, 0, 0, 255))
                        if not next_next_edge:
                            for cross in self.map_diag[inter_id][next_edge]:
                                for step in range(2, num_predict_steps + 2):
                                    sidewalk_length = self.paras["network_graph"][inter_id]["incoming_ped"][current_edge]["length"]
                                    if (sidewalk_length - pos) / speed <= (step - 1) * delta_T and (sidewalk_length - pos) / speed >= (step - 2) * delta_T:
                                        cross_demand_all_steps[crossing_number_map[cross]][step].add(peds_lane[j])
                                        traci.person.setColor(peds_lane[j], (255, 0, 0, 255))

            ## Count the Conflicts between right turning vehicles and pedestrians passing on the crossings:
            all_cars=traci.vehicle.getIDList()
            all_peds=traci.person.getIDList()
            for car in all_cars:
                for dir in right_turning_vehs:
                    if dir in car and traci.vehicle.getLaneID(car).startswith(":") and "bike" not in car:
                        for ped in all_peds:
                            if traci.lane.getEdgeID(traci.person.getLaneID(ped)) in self.paras["network_graph"][inter_id]["crossing"]:
                                position_car=traci.vehicle.getPosition(car)
                                position_ped=traci.person.getPosition(ped)
                                speed_car=traci.vehicle.getSpeed(car)
                                speed_ped=traci.person.getSpeed(ped)
                                if math.dist(position_car, position_ped) < 5 and math.dist(position_car, position_ped)/max(1,(speed_car+speed_ped))<2: ## if TTC less than 2 seconds
                                    self.right_turn_conflicts.setdefault(car, set())
                                    self.right_turn_conflicts[car].add(ped)

            ## count the bike demand
            for i in range(len(bike_lane_id)):
                bikes_lane = traci.lane.getLastStepVehicleIDs(bike_lane_id[i])
                # print(bikes_lane)
                num = 0
                for j in range(len(bikes_lane) - 1, -1, -1):
                    # if bikes_lane[j] == "WE-bike.30":
                    #     a=2
                    self.paras["bike_ids"].add(bikes_lane[j])
                    speed = traci.vehicle.getSpeed(bikes_lane[j])
                    pos = traci.vehicle.getLanePosition(bikes_lane[j])
                    # We have to estimate the position and speed of the bikes in bike lane too because they might suddenly enter the car lane
                    self.next_spd.setdefault(bikes_lane[j], 0)
                    self.next_pos.setdefault(bikes_lane[j], (lane_length[i] - self.paras["length"]["Bike"]))
                    num += 1
                    if num >= 2:
                        if self.next_pos[bikes_lane[j + 1]] - self.next_pos[bikes_lane[j]] < self.paras["minGap"]["Bike"] + self.paras["length"]["Bike"]:
                            self.next_pos[bikes_lane[j]] = self.next_pos[bikes_lane[j + 1]] - self.paras["minGap"][
                                "Bike"] - self.paras["length"]["Bike"]
                        d = self.next_pos[bikes_lane[j + 1]] - self.next_pos[bikes_lane[j]] - self.paras["length"][
                            "Bike"]
                        center_distance = d + self.paras["length"]["Bike"] / 2 + self.paras["length"]["Bike"] / 2
                        dmin = self.paras["dmin"]["Bike"]
                        if center_distance < dmin:
                            v_desired = 0
                        elif dmin < center_distance < dmin + self.paras["max_spd"]["Bike"] / self.paras["k"]["Bike"]:
                            v_desired = self.paras["k"]["Bike"] * (center_distance - dmin)
                        else:  # (center_distance> )
                            v_desired = self.paras["max_spd"]["Bike"]

                        Fij = self.paras["Ai"]["Bike"] * math.e ** ((-d) / self.paras["Bi"]["Bike"]) * (-1)
                        UM_acc = 1 / self.paras["mass"]["Bike"] * ((self.paras["mass"]["Bike"] * (
                                    v_desired - self.next_spd[bikes_lane[j]]) / 0.5) + Fij)
                        # print(UM_acc)
                        UM_acc = max(UM_acc, self.paras["min_acc"]["Bike"])
                        UM_acc = min(UM_acc, self.paras["max_acc"]["Bike"])
                        # print(UM_acc)
                        speed = max(0, UM_acc * delta_T_faster + self.next_spd[bikes_lane[j]])
                        # print(speed)
                        speed = min(speed, self.paras["max_spd"]["Bike"])
                        # print(speed)
                        self.next_spd[bikes_lane[j]] = speed

                        # print("self.next_spd[cars_lane[j]]", self.next_spd[cars_lane[j]])
                        # print(self.next_pos[bikes_lane[j]])
                        self.next_pos[bikes_lane[j]] = min(-0.1, 1 / 2 * UM_acc * delta_T_faster ** 2 + self.next_spd[
                            bikes_lane[j]] * delta_T_faster + self.next_pos[bikes_lane[j]])
                        # print(self.next_pos[bikes_lane[j]])
                    else:
                        self.next_pos[bikes_lane[j]] = min(-0.1,
                                                           1 / 2 * self.paras["max_acc"]["Bike"] * delta_T_faster ** 2 +
                                                           self.next_spd[bikes_lane[j]] * delta_T_faster +
                                                           self.next_pos[bikes_lane[j]])
                        self.next_spd[bikes_lane[j]] = min(self.paras["max_spd"]["Bike"],
                                                           self.next_spd[bikes_lane[j]] + self.paras["max_acc"][
                                                               "Bike"] * delta_T_faster)

                    # bikelane_length = self.paras["network_graph"][inter_id]["incoming_ped"][current_edge]["length"]
                    for step in range(num_predict_steps + 1):
                        if speed == 0:
                            if pos > 10:
                                bike_demand[i+1][1].add(bikes_lane[j])
                        elif (186.4 - pos)/speed <= (step + 1) * delta_T and (186.4 - pos)/speed >= (step) * delta_T:
                            bike_demand[i+1][step + 1].add(bikes_lane[j])
            # print("bike demand: ", bike_demand)

            ## Get dynamic information. Use linear interpolation to estimate HDV's and Bike's state.
            for i in range(len(lane_id)):
                pos_vehicles.append([])
                speed_vehicles.append([])
                wt_vehicles.append([])
                arrival_times.append([])
                veh_id.append([])
                veh_len.append([])
                minGap.append([])
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])

                # Number of vehicles in the current lane and also in the communication range.
                num_vehicles_lane = 0
                # Number of HDVs between two CAVs. Will be set to 0 when we find a CAV.
                current_num_hdv_bike = 0
                # HDV ids between two CAVs. Will be set to empty when we find a CAV.
                current_ids_hdv_bike = []
                rates = []
                lengths = []
                for j in range(len(cars_lane) - 1, -1, -1):
                    if cars_lane[j] in cav_ids:
                        ## This is a CAV, we need to first check if there are HDVs in front of it.
                        if current_num_hdv_bike == 0:
                            ## There is no HDV or bike between the current CAV and the CAV ahead.
                            if (
                                    lane_length[i]
                                    - traci.vehicle.getLanePosition(cars_lane[j])
                                    <= communication_range
                            ):
                                ## The current CAV is within the communication range.
                                # print(lane_id[i])
                                # print(cars_lane[j])
                                pos_vehicles[-1].append(
                                    -(
                                            lane_length[i]
                                            - traci.vehicle.getLanePosition(cars_lane[j])
                                    )
                                )
                                speed_vehicles[-1].append(
                                    traci.vehicle.getSpeed(cars_lane[j])
                                )
                                wt_vehicles[-1].append(
                                    traci.vehicle.getWaitingTime(cars_lane[j])
                                )
                                veh_id[-1].append(cars_lane[j])
                                if "bike" in cars_lane[j]:
                                    rates.append(1)
                                    lengths.append(1.6)
                                else:
                                    rates.append(car_bike_gap)
                                    lengths.append(5)
                                num_vehicles_lane += 1
                            else:
                                ## The current CAV is outside the communication range. We need to check if
                                ## it will arrive within the MPC's prediction horizon. If so, add its arrival time.
                                temp = (
                                               lane_length[i]
                                               - traci.vehicle.getLanePosition(cars_lane[j])
                                               - communication_range
                                       ) / speed_limit
                                if temp < delta_T * num_predict_steps:
                                    arrival_times[-1].append(
                                        cur_step + int(temp / delta_T_faster)
                                    )
                                    veh_id[-1].append(cars_lane[j])
                                    if "bike" in cars_lane[j]:
                                        rates.append(1)
                                        lengths.append(1.6)
                                    else:
                                        rates.append(car_bike_gap)
                                        lengths.append(5)
                                    num_vehicles_lane += 1
                        else:
                            ## There are some HDVs in front of the current CAV. Use interpolation to estimate their states.
                            # print(cars_lane[j])
                            current_ids_hdv_bike.append(cars_lane[j])
                            if "bike" in cars_lane[j]:
                                rates.append(1)
                                lengths.append(1.6)
                            else:
                                rates.append(car_bike_gap)
                                lengths.append(5)
                            # Find the right most vehicle's state.
                            no_cav_ahead = False
                            if not pos_vehicles[-1]:
                                # This is the first CAV in the lane. All the vehicles in front of it are HDVs.
                                r_pos, r_spd, r_wt = (-0.1), 0, 0
                                pos_vehicles[-1].append(r_pos)
                                speed_vehicles[-1].append(r_spd)
                                wt_vehicles[-1].append(r_wt)
                                veh_id[-1].append(current_ids_hdv_bike[0])
                                current_num_hdv_bike -= 1
                                num_vehicles_lane += 1
                                no_cav_ahead = True
                            else:
                                # There are CAVs ahead. Simply get the values.
                                rates.insert(0, car_bike_gap)
                                lengths.insert(0, 5)
                                r_pos, r_spd, r_wt = (
                                    pos_vehicles[-1][-1],
                                    speed_vehicles[-1][-1],
                                    wt_vehicles[-1][-1],
                                )

                            # Find the left most vehicle's state, i.e., the current vehicle's state.
                            l_pos = -(
                                    lane_length[i]
                                    - traci.vehicle.getLanePosition(cars_lane[j])
                            )
                            l_spd = traci.vehicle.getSpeed(cars_lane[j])
                            l_wt = traci.vehicle.getWaitingTime(cars_lane[j])

                            # Estimate HDVs' states infront and record them if in communication range or will arrive during MPC's prediction horizon.
                            for k in range(current_num_hdv_bike + 1):
                                # print(current_ids_hdv_bike)
                                # print(rates)
                                # print(veh_id[-1][-1])

                                t_pos = r_pos - lengths[0] + sum(rates[1:k + 2]) / sum(rates[1:]) * (
                                        l_pos - (r_pos - sum(lengths[0:-1]))
                                ) - sum(lengths[1:k + 1])
                                t_spd = r_spd + sum(rates[1:k + 2]) / sum(rates[1:]) * (
                                        l_spd - r_spd
                                )
                                t_wt = r_wt + sum(rates[1:k + 2]) / sum(rates[1:]) * (
                                        l_wt - r_wt
                                )
                                if -t_pos <= communication_range:
                                    pos_vehicles[-1].append(t_pos)
                                    speed_vehicles[-1].append(t_spd)
                                    wt_vehicles[-1].append(t_wt)
                                    if no_cav_ahead:
                                        veh_id[-1].append(current_ids_hdv_bike[k + 1])
                                    else:
                                        veh_id[-1].append(current_ids_hdv_bike[k])
                                    num_vehicles_lane += 1
                                else:
                                    temp = (-t_pos - communication_range) / speed_limit
                                    if temp < delta_T * num_predict_steps:
                                        arrival_times[-1].append(temp)
                                        if no_cav_ahead:
                                            veh_id[-1].append(current_ids_hdv_bike[k + 1])
                                        else:
                                            veh_id[-1].append(current_ids_hdv_bike[k])
                                        num_vehicles_lane += 1
                            # rates = []
                        current_num_hdv_bike = 0
                        current_ids_hdv_bike = []
                        rates = []
                        lengths = []
                    else:
                        ## This is a HDV or Bike. We can't get the state. Just record the ids and update the number of HDVs and Bikes.
                        current_ids_hdv_bike.append(cars_lane[j])
                        if "bike" in cars_lane[j]:
                            self.paras["bike_ids"].add(cars_lane[j])
                            rates.append(1)
                            lengths.append(1.6)
                        else:
                            rates.append(car_bike_gap)
                            lengths.append(5)
                        current_num_hdv_bike += 1

                ## There could be a corner case where the last several vehicles in a lane are all HDVs.
                ## We need to estimate the information of those vehicles. The methods are the same as above.
                if current_num_hdv_bike != 0:
                    no_cav_ahead = False
                    if not pos_vehicles[-1]:
                        r_pos, r_spd, r_wt = (-0.1), 0, 0
                        pos_vehicles[-1].append(r_pos)
                        speed_vehicles[-1].append(r_spd)
                        wt_vehicles[-1].append(r_wt)
                        veh_id[-1].append(current_ids_hdv_bike[0])
                        current_num_hdv_bike -= 1
                        num_vehicles_lane += 1
                        no_cav_ahead = True
                    else:
                        r_pos, r_spd, r_wt = (
                            pos_vehicles[-1][-1],
                            speed_vehicles[-1][-1],
                            wt_vehicles[-1][-1],
                        )
                    if not no_cav_ahead:
                        rates.insert(0, car_bike_gap)
                        lengths.insert(0, 5)
                    l_pos = -(
                            lane_length[i]
                            - traci.vehicle.getLanePosition(cars_lane[0])
                    )
                    l_spd = traci.vehicle.getSpeed(cars_lane[0])
                    l_wt = traci.vehicle.getWaitingTime(cars_lane[0])
                    for k in range(current_num_hdv_bike):
                        # print(current_ids_hdv_bike)
                        # print(rates)
                        # print(veh_id[-1][-1])
                        t_pos = r_pos - lengths[0] + sum(rates[1:k + 2]) / sum(rates[1:]) * (
                                l_pos - (r_pos - sum(lengths[0:-1]))
                        ) - sum(lengths[1:k + 1])

                        t_spd = r_spd + sum(rates[1:k + 2]) / sum(rates[1:]) * (l_spd - r_spd)
                        t_wt = r_wt + sum(rates[1:k + 2]) / sum(rates[1:]) * (l_wt - r_wt)
                        if -t_pos <= communication_range:
                            pos_vehicles[-1].append(t_pos)
                            speed_vehicles[-1].append(t_spd)
                            wt_vehicles[-1].append(t_wt)
                            if no_cav_ahead:
                                veh_id[-1].append(current_ids_hdv_bike[k + 1])
                            else:
                                veh_id[-1].append(current_ids_hdv_bike[k])
                            num_vehicles_lane += 1
                        else:
                            temp = (-t_pos - communication_range) / speed_limit
                            if temp < delta_T * num_predict_steps:
                                arrival_times[-1].append(
                                    cur_step + int(temp / delta_T_faster)
                                )
                                if no_cav_ahead:
                                    veh_id[-1].append(current_ids_hdv_bike[k + 1])
                                else:
                                    veh_id[-1].append(current_ids_hdv_bike[k])
                                num_vehicles_lane += 1
                num_vehicles_max = max(num_vehicles_max, num_vehicles_lane)

            ## Get dynamic information based on Unified Model:
            for i in range(len(lane_id)):
                pos_vehicles2.append([])
                speed_vehicles2.append([])
                arrival_times2.append([])
                veh_id2.append([])
                types.append([])
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                num_vehicles_lane = 0
                for j in range(len(cars_lane) - 1, -1, -1):
                    if cars_lane[j] == "WE-bike.7":
                        a=2
                    self.next_pos.setdefault(cars_lane[j], None)
                    self.next_spd.setdefault(cars_lane[j], None)
                    # if cars_lane[j] == "WE-bike.5":
                    #     print(self.next_pos[cars_lane[j]])
                    #     print(self.next_spd[cars_lane[j]])
                    #     a=2
                    if cars_lane[j] in cav_ids:
                        ## This is a CAV
                        if (
                                lane_length[i]
                                - traci.vehicle.getLanePosition(cars_lane[j])
                                <= communication_range
                        ):
                            ## The current CAV is within the communication range.
                            pos_vehicles2[-1].append(
                                -(
                                        lane_length[i]
                                        - traci.vehicle.getLanePosition(cars_lane[j])
                                )
                            )
                            speed_vehicles2[-1].append(
                                traci.vehicle.getSpeed(cars_lane[j])
                            )
                            veh_id2[-1].append(cars_lane[j])
                            types[-1].append("Car")

                            # Now check if there are any HDVs or Bikes in front of it,and check safety
                            if num_vehicles_lane >= 1 and pos_vehicles2[-1][-2] - pos_vehicles2[-1][-1] < \
                                    self.paras["minGap"][types[-1][-1]] + self.paras["length"][types[-1][-2]]:
                                pos_vehicles2[-1][-1] = pos_vehicles2[-1][-2] - self.paras["minGap"][
                                    types[-1][-1]] - self.paras["length"][types[-1][-2]]

                            num_vehicles_lane += 1
                        else:
                            ## The current CAV is outside the communication range. We need to check if
                            ## it will arrive within the MPC's prediction horizon. If so, add its arrival time.
                            temp = (
                                           lane_length[i]
                                           - traci.vehicle.getLanePosition(cars_lane[j])
                                           - communication_range
                                   ) / speed_limit
                            if temp < delta_T * num_predict_steps:
                                arrival_times2[-1].append(
                                    cur_step + int(temp / delta_T_faster)
                                )
                                veh_id2[-1].append(cars_lane[j])
                                types[-1].append("Car")
                                num_vehicles_lane += 1
                    else:
                        # This is an HDV or bike
                        if "bike" in cars_lane[j]:
                            type = "Bike"
                        else:
                            type = "Car"
                        if num_vehicles_lane == 0:
                            # The HDV or bike is the first on the lane
                            if not self.next_pos[cars_lane[j]]:
                                # has just arrived to the lane
                                pos_vehicles2[-1].append(
                                    -(
                                            lane_length[i]
                                            - self.paras["length"][type]
                                    )
                                )
                                speed_vehicles2[-1].append(0)
                                veh_id2[-1].append(cars_lane[j])
                                types[-1].append(type)
                                num_vehicles_lane += 1
                                self.next_spd[cars_lane[j]] = 0
                                self.next_pos[cars_lane[j]] = -(
                                            lane_length[i] - self.paras["length"][type])
                            else:
                                # The HDV or bike have entered the lane in previous time steps and are first on the lane
                                pos_vehicles2[-1].append(self.next_pos[cars_lane[j]])
                                speed_vehicles2[-1].append(self.next_spd[cars_lane[j]])
                                if traci.vehicle.getSpeed(cars_lane[j]) < 1:
                                    # The vehicle is stopping because of the traffic light (because first in the line) and cannot have free flow
                                    self.next_spd[cars_lane[j]] = 0
                                    self.next_pos[cars_lane[j]] = -0.1
                                else:
                                    # vehicle is not stopping
                                    self.next_pos[cars_lane[j]] = min(-0.1, pos_vehicles2[-1][-1] +
                                                                      speed_vehicles2[-1][-1] * delta_T_faster)
                                    # self.next_spd[cars_lane[j]] = min(self.paras["max_spd"][type],
                                    #                                   self.paras["max_acc"][
                                    #                                       type] * delta_T_faster +
                                    #                                   speed_vehicles2[-1][-1])
                                    if speed_vehicles[i][j]>0:
                                        self.next_spd[cars_lane[j]] = min(self.paras["max_spd"][type],
                                                                          self.paras["max_acc"][
                                                                              type] * delta_T_faster +
                                                                          speed_vehicles2[-1][-1], speed_vehicles[i][j])
                                    else:
                                        self.next_spd[cars_lane[j]] = min(self.paras["max_spd"][type],
                                                                          self.paras["max_acc"][
                                                                              type] * delta_T_faster +
                                                                          speed_vehicles2[-1][-1])
                                    # print("cars_lane[j]: ", cars_lane[j])
                                    # print("speed_vehicles2[-1][-1]: ", speed_vehicles2[-1][-1])
                                veh_id2[-1].append(cars_lane[j])
                                types[-1].append(type)
                                num_vehicles_lane += 1
                        else:
                            # There are already vehicles in front of the HDV or bike
                            front_veh = veh_id2[-1][-1]
                            # we should estimate the speed and position of HDV and bikes based on the Unified Model
                            if not self.next_pos[cars_lane[j]]:
                                # This vehicle has just arrived to the lane and this is for only initialization
                                # (next speed and pos are gonna be calculated again later)
                                self.next_spd[cars_lane[j]] = 0
                                self.next_pos[cars_lane[j]] = -(lane_length[i] - traci.vehicle.getLanePosition(cars_lane[j]))
                            pos_vehicles2[-1].append(self.next_pos[cars_lane[j]])
                            speed_vehicles2[-1].append(self.next_spd[cars_lane[j]])
                            veh_id2[-1].append(cars_lane[j])
                            types[-1].append(type)
                            num_vehicles_lane += 1
                            #
                            # print(veh_id2)
                            # print(types)

                            # if cars_lane[j] == "WE-bike.50":
                            #     a=2

                            if (pos_vehicles2[-1][-2] - pos_vehicles2[-1][-1] < self.paras["minGap"][type] +
                                    self.paras["length"][types[-1][-2]]):
                                # print(pos_vehicles2[-1][-1])
                                # print(pos_vehicles2[-1][-2])
                                pos_vehicles2[-1][-1] = pos_vehicles2[-1][-2] - self.paras["minGap"][type] - \
                                                        self.paras["length"][types[-1][-2]]
                                # print(pos_vehicles2[-1][-1])
                                # print(pos_vehicles2[-1][-2])

                            interaction = type
                            if types[-1][-1] != types[-1][-2]:
                                interaction = "Car-Bike"
                            d = pos_vehicles2[-1][-2] - pos_vehicles2[-1][-1] - self.paras["length"][
                                types[-1][-2]]
                            center_distance = d + self.paras["length"][types[-1][-1]] / 2 + \
                                              self.paras["length"][types[-1][-2]] / 2
                            dmin = self.paras["dmin"][interaction]
                            if center_distance < dmin:
                                v_desired = 0
                            elif dmin < center_distance < dmin + self.paras["max_spd"][type] / self.paras["k"][
                                type]:
                                v_desired = self.paras["k"][type] * (center_distance - dmin)
                            else:  # (center_distance> )
                                v_desired = self.paras["max_spd"][type]

                            Fij = self.paras["Ai"][type] * math.e ** ((-d) / self.paras["Bi"][type]) * (-1)
                            UM_acc = 1 / self.paras["mass"][type] * ((self.paras["mass"][type] * (
                                        v_desired - self.next_spd[cars_lane[j]]) / 0.5) + Fij)
                            speed = max(0, UM_acc * delta_T_faster + speed_vehicles2[-1][-1])
                            # print(speed)
                            speed = min(speed, self.paras["max_spd"][type])
                            # print(speed)
                            self.next_spd[cars_lane[j]] = speed
                            # print(self.next_pos[cars_lane[j]])
                            self.next_pos[cars_lane[j]] = min(-0.1, 1 / 2 * UM_acc * delta_T_faster ** 2 +
                                                              speed_vehicles2[-1][-1] * delta_T_faster +
                                                              pos_vehicles2[-1][-1])
                            # print(self.next_pos[cars_lane[j]])


            for ind_lane in range(len(veh_id)):
                for ind_veh in veh_id[ind_lane]:
                    if "bike" in ind_veh:
                        veh_len[ind_lane].append(self.paras["length"]["Bike"])
                        minGap[ind_lane].append(self.paras["minGap"]["Bike"])
                    else:
                        veh_len[ind_lane].append(self.paras["length"]["Car"])
                        minGap[ind_lane].append(self.paras["minGap"]["Car"])

            # print(veh_id)
            # print(pos_vehicles2)
            # print(pos_vehicles)
            # print(speed_vehicles2)
            # print(speed_vehicles)
            self.network_state[inter_id] = {
                "num_vehicles_max": num_vehicles_max,
                "pos_vehicles": pos_vehicles,
                "speed_vehicles": speed_vehicles,
                "wt_vehicles": wt_vehicles,
                "arrival_times": arrival_times,
                "signal_phase": signal_phase,
                "num_lane": len(lane_id),
                "lane_length": lane_length,
                "lane_id": lane_id,
                "vehicle_id": veh_id,
                "pedestrian_demand_current": cross_demand_current,
                "pedestrian_demand": cross_demand_all_steps,
                "bike_demand": bike_demand,
                "vehicle_length": veh_len,
                "vehicle_types": types,
                "minGap": minGap,
            }
        return self.network_state


    def apply_control_commands(
        self, should_update_signal, next_signal_phase, speed_commands
    ):
        for inter_id in self.network_graph:
            print(traci.simulation.getTime())
            # Signal phase control
            ## TODO: every added variable such as noWALK and LPI should be a dictionary for each intersection in a networkwide setting
            if self.paras["ped_LPI"]>0 and self.LPI == self.paras["ped_LPI"] +self.paras["delta_T_faster"]:
                traci.trafficlight.setPhase(inter_id, next_signal_phase)
                self.LPI_on = False
                self.LPI = 0

            cur_phase = int(traci.trafficlight.getPhase(inter_id))
            if should_update_signal:
                if next_signal_phase == -1:
                    if len(self.paras["ped_phase_map"][cur_phase])>0 and self.paras["ped_phasing"] == "Concurrent":
                        self.noWALK += self.paras["delta_T_faster"]
                        traci.trafficlight.setPhase(inter_id, cur_phase + 2*self.network_graph[inter_id]["num_phases"])
                        self.clearance = True
                    else:
                        traci.trafficlight.setPhase(inter_id, cur_phase + self.network_graph[inter_id]["num_phases"])
                        self.yellow_duration += self.paras["delta_T_faster"]
                        self.clearance = False
                else:
                    if self.paras["ped_LPI"]>0 and len(self.paras["ped_phase_map"][next_signal_phase])>0 and cur_phase != next_signal_phase:
                        traci.trafficlight.setPhase(inter_id, next_signal_phase + 3*self.network_graph[inter_id]["num_phases"])
                        self.LPI = self.paras["delta_T_faster"]
                        self.LPI_on = True
                        self.yellow_duration = 0
                    else:
                        traci.trafficlight.setPhase(inter_id, next_signal_phase)
                        self.yellow_duration = 0
            elif next_signal_phase == -1:
                if self.clearance:
                    self.noWALK += self.paras["delta_T_faster"]
                    print(self.noWALK)
                else:
                    self.yellow_duration += self.paras["delta_T_faster"]
                    self.noWALK=0


            if self.LPI_on:
                self.LPI += self.paras["delta_T_faster"]
            if self.yellow_duration == self.paras["yellow_time"] + self.paras["delta_T_faster"]:
                traci.trafficlight.setPhase(inter_id, self.paras["all_red_index"])
            if self.noWALK == self.paras["ped_FDW"] + self.paras["delta_T_faster"]:
                traci.trafficlight.setPhase(inter_id, cur_phase - self.network_graph[inter_id]["num_phases"])
                self.clearance = False
                self.yellow_duration += self.paras["delta_T_faster"]

            # Vehicles control
            network_vehs = traci.vehicle.getIDList()
            for veh_id in speed_commands:
                if veh_id in network_vehs:
                    traci.vehicle.setSpeed(veh_id, speed_commands[veh_id])


    def move_one_step_forward(self):
        traci.simulationStep()

    def performance_results(self, phase_list_multi, duration_list_multi, network_type, volume_type, control_type, step):
        data_dir = os.path.dirname(os.path.realpath(__file__)) + "/simulation_data"
        data_dir_next = data_dir + "/" + network_type
        queue_file = (data_dir_next + "/queues_" + control_type + "_" + volume_type + "_pene_" + str(int(self.paras["penetration"] * 100)) + ".xml")
        tripinfo_file = (data_dir_next + "/tripinfo_" + control_type + "_" + volume_type + "_pene_" + str(int(self.paras["penetration"] * 100)) + ".xml")
        SSMinfo_file = (data_dir_next + "/SSMinfo_" + control_type + "_pene_" + str(int(self.paras["penetration"] * 100)) + ".xml")

        self.get_average_delay_endtime(tripinfo_file)
        self.get_average_queue_length_endtime(queue_file)
        self.get_average_phase_duration(phase_list_multi, duration_list_multi, control_type)
        right_conflicts=self.right_turn_conflicts_measure()
        conflict_cnt_BikeBike, conflict_cnt_BikeVeh, conflict_cnt_VehVeh = self.get_number_of_conflicts(SSMinfo_file, "1")
        print(f"average fuel consumption (external model) for CAVs {control_type} (in mg): ",
              self.fuel_total_cav_external_model / max(len(self.paras["veh_id_with_ev"]["cav_ice"]),1))
        print(f"average fuel consumption (external model) for HDVs {control_type} (in mg): ",
              self.fuel_total_hdv_external_model / max(len(self.paras["veh_id_with_ev"]["hdv_ice"]),1))
        print(f"average power consumption (external model) for CAVs {control_type} (in Kw.s): ",
              self.power_total_cav_external_model / max(len(self.paras["veh_id_with_ev"]["cav_ev"]),1))
        print(f"average power consumption (external model) for HDVs {control_type} (in Kw.s): ",
              self.power_total_hdv_external_model / max(len(self.paras["veh_id_with_ev"]["hdv_ev"]),1))
        # print(f"average fuel consumption (SUMO output) for {control_type} (in mg): ",
        #       self.fuel_total_cav_sumo / len(self.paras["cav_ids"]["all"]))
        print(f"average waiting time for {control_type} (in s): ", self.waiting_time_avg)
        print(f"average time loss for {control_type} (in s): ", self.lost_time_avg)
        print(f"average queue for {control_type} length (in m): ", self.queue_avg)
        print(f"average bike (sep+int) waiting time for {control_type} (in s): ", self.waiting_time_avg_bike)
        print(f"average bike (sep+int) time loss for {control_type} (in s): ", self.lost_time_avg_bike)
        print(f"average bike queue (sep lane) for {control_type} length (in m): ", self.queue_avg_bike)
        print(f"average pedestrian time loss for {control_type} (in s): ", self.lost_time_avg_ped)
        print(f"number of right-turn conflicts between vehicles and pedestrians: {right_conflicts}")
        print(f"number of conflicts between vehicles and cyclists: {conflict_cnt_BikeVeh}")
        print(f"number of conflicts between vehicles and vehicles: {conflict_cnt_VehVeh}")
        print(f"number of CAVs passing through the specific intersection for {control_type}: ", len(self.paras["cav_ids"]))
        print(f"number of pedestrians passing through the specific intersection for {control_type}: ", len(self.paras["ped_ids"]))
        print(f"number of bikes passing through the specific intersection (sep lane) for {control_type}: ", len(self.paras["bike_ids"]))
        print(f"The time of simulation termination in {control_type} scenario:",step/2)


        if self.paras["poisson_gamma_pedestrian"] == 0.01:
            a = "LowPed"
        elif self.paras["poisson_gamma_pedestrian"] == 0.04:
            a = "MedPed"
        elif self.paras["poisson_gamma_pedestrian"] == 0.07:
            a = "HighPed"

        if self.paras["poisson_gamma_bike"] == 0.01:
            b = "LowBike"
        elif self.paras["poisson_gamma_bike"] == 0.02:
            b = "MedBike"
        elif self.paras["poisson_gamma_bike"] == 0.03:
            b = "HighBike"
        else:
            raise

        if control_type == "multi_scale":
            file_name = f"Results/{control_type}_penetration({self.paras["penetration"]})_EVratio({self.paras["ratio_ev"]})_{self.paras["ped_phasing"]}_{self.paras["ped_subsetting"]}_{self.paras["ped_demand_symmetry"]}_{self.paras["weight(Vehicles/Pedestrians/Bikes)"]}_{a}.txt"
        else:
            file_name = f"Results/{control_type}_penetration({self.paras["penetration"]})_EVratio({self.paras["ratio_ev"]})_{self.paras["ped_phasing"]}_{self.paras["ped_subsetting"]}_{self.paras["ped_demand_symmetry"]}_{a}.txt"
        with open(file_name, 'w') as file:
            file.write(f"average fuel consumption (external model) for CAVs {control_type} (in mg): {self.fuel_total_cav_external_model / max(len(self.paras["veh_id_with_ev"]["cav_ice"]),1)}\n")
            file.write(f"average fuel consumption (external model) for HDVs {control_type} (in mg): {self.fuel_total_hdv_external_model / max(len(self.paras["veh_id_with_ev"]["hdv_ice"]),1)}\n")
            file.write(f"average power consumption (external model) for CAVs {control_type} (in Kw.s): {self.power_total_cav_external_model / max(len(self.paras["veh_id_with_ev"]["cav_ev"]),1)}\n")
            file.write(f"average power consumption (external model) for HDVs {control_type} (in Kw.s): {self.power_total_hdv_external_model / max(len(self.paras["veh_id_with_ev"]["hdv_ev"]),1)}\n")

            #file.write(f"average fuel consumption for {control_type} scenario (SUMO output) (in mg): {self.fuel_total_cav_sumo / len(self.paras['cav_ids']['all'])}\n")
            file.write(f"average waiting time for {control_type} scenario (in s): {self.waiting_time_avg}\n")
            file.write(f"average time loss for {control_type} scenario (in s): {self.lost_time_avg}\n")
            file.write(f"average queue length for {control_type} scenario (in m): {self.queue_avg}\n")
            file.write(f"average bike (sep+int) waiting time for {control_type} scenario (in s): {self.waiting_time_avg_bike}\n")
            file.write(f"average bike (sep+int) time loss for {control_type} scenario (in s): {self.lost_time_avg_bike}\n")
            file.write(f"average bike (sep) queue length for {control_type} scenario (in m): {self.queue_avg_bike}\n")
            file.write(f"average pedestrian time loss for {control_type} scenario (in s): {self.lost_time_avg_ped}\n")
            file.write(f"number of right-turn conflicts between vehicles and pedestrians: {right_conflicts}\n")
            file.write(f"number of conflicts between vehicles and cyclists: {conflict_cnt_BikeVeh}\n")
            file.write(f"number of conflicts between vehicles and vehicles: {conflict_cnt_VehVeh}\n")
            file.write(f"number of CAVs passing through the specific intersection in {control_type} scenario: {len(self.paras['cav_ids'])}\n")
            file.write(f"number of pedestrians passing through the specific intersection in {control_type} scenario: {len(self.paras['ped_ids'])}\n")
            file.write(f"number of Bikes passing through the specific intersection (sep) in {control_type} scenario: {len(self.paras["bike_ids"])}\n")
            file.write(f"The time of simulation termination in {control_type} scenario: {step/2}\n")
            file.write(f"average phase lengths:  {self.phase_avg}\n")
            file.write(f"number of times each phase happened: {self.phase_ntimes}\n")

    def get_number_of_conflicts(self, file, inter_id):
        ## Start SUMO again for the needed traci function.
        if self.paras["ped_phasing"] == "Exclusive":
            model_file = os.path.dirname(os.path.dirname(os.path.realpath(
                __file__))) + f"/environment/network_model/single_intersection_multi_scale_{self.paras["ped_phasing"]}.sumocfg"
        else:
            model_file = os.path.dirname(os.path.dirname(os.path.realpath(
                __file__))) + f"/environment/network_model/single_intersection_multi_scale_{self.paras["ped_phasing"]}_({self.paras["ped_subsetting"]}).sumocfg"
        # traci.start(["sumo", "-c", model_file, "--start"])

        tree = ET.parse(file)
        root = tree.getroot()
        conflict_cnt_BikeVeh= []
        conflict_cnt_BikeBike= []
        conflict_cnt_VehVeh= []
        cntt = 0
        for conflict in root.findall('.//conflict'):
            cntt += 1
            ego = conflict.attrib['ego']
            foe = conflict.attrib['foe']
            for measure in conflict:
                position = measure.attrib.get("position")
                value = measure.attrib.get("value")
                if position != "NA" and measure.tag == "minTTC" and float(value) <= 1.5:  ##TTC calculates both lead-follow and merging,crossing

                    # x, y = map(float, position.split(','))
                    # edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(x, y, isGeo=False)
                    # out_edge_ids= [x[:-2] for x in self.paras["network_graph"][inter_id]["outgoing_veh"].keys()]
                    # if edgeID not in out_edge_ids:
                    # print(foe)
                    # print(ego)
                    if "bike" in foe and "bike" in ego and set([foe, ego]) not in conflict_cnt_BikeBike:
                        conflict_cnt_BikeBike.append(set([foe, ego]))
                    elif "bike" in foe or "bike" in ego and set([foe, ego]) not in conflict_cnt_BikeVeh:
                        conflict_cnt_BikeVeh.append(set([foe, ego]))
                    elif float(value) <= 1:
                        conflict_cnt_VehVeh.append(set([foe, ego]))
                    break


        # traci.close()
        return len(conflict_cnt_BikeBike), len(conflict_cnt_BikeVeh), len(conflict_cnt_VehVeh)


    def right_turn_conflicts_measure(self):
        a=0
        for key in self.right_turn_conflicts.keys():
            a+=len(self.right_turn_conflicts[key])
        return a
    def get_average_phase_duration(self, phase_list_multi, duration_list_multi, control_type):
        phase_dict = {}
        if control_type!="multi_scale":
            change_step=0
            for i, phase in enumerate(self.phase_list_fix_act[:-1]):
                next_phase=self.phase_list_fix_act[i+1]
                if next_phase!=phase:
                    phase_dict.setdefault(phase,[])
                    phase_dict[phase].append((i-change_step)*0.5)
                    change_step = i
            x = np.array(self.dur_list_fix_act)
            widths = np.diff(np.concatenate(([0], x)))
            plt.figure(figsize=(15, 7))
            phase_list_fix_act_1 = [phase + 1 for phase in self.phase_list_fix_act]

            self.phase_avg={}
            self.phase_ntimes={}
            for phase in phase_dict.keys():
                self.phase_avg[phase]=sum(phase_dict[phase])/len(phase_dict[phase])
                self.phase_ntimes[phase]=sum(phase_dict[phase])/5
            print("average phase lengths: ", self.phase_avg)
            print("number of times each phase happened: ", self.phase_ntimes)


        else:
            change_index=0
            for i, phase in enumerate(phase_list_multi[:-1]):
                next_phase = phase_list_multi[i+1]
                if phase != next_phase:
                    phase_dict.setdefault(phase, [])
                    phase_dict[phase].append(sum(duration_list_multi[change_index:i+1]))
                    change_index = i+1

            self.phase_avg={}
            self.phase_ntimes={}
            for phase in phase_dict.keys():
                self.phase_avg[phase]=sum(phase_dict[phase])/len(phase_dict[phase])
                self.phase_ntimes[phase]=sum(phase_dict[phase])/5
            print("average phase lengths: ", self.phase_avg)
            print("number of times each phase happened: ", self.phase_ntimes)


    def get_average_delay_endtime(self,file):
        tree = ET.parse(file)
        root = tree.getroot()
        cnt = 0
        cnt_ped=0
        cnt_bike=0
        wt = 0
        tl = 0
        tl_ped=0
        wt_max = 0
        wt_bike=0
        tl_bike=0
        for stu in root:
            if stu.tag == "tripinfo":
                if "bike" in stu.attrib["id"]:
                    wt_bike += float(stu.attrib["waitingTime"])
                    tl_bike += float(stu.attrib["timeLoss"])
                    cnt_bike += 1
                else:
                    wt += float(stu.attrib["waitingTime"])
                    wt_max = max(wt_max, float(stu.attrib["waitingTime"]))
                    tl += float(stu.attrib["timeLoss"])
                    cnt += 1
            elif stu.tag == "personinfo":
                walk=stu.find("walk")
                tl_ped +=float (walk.attrib["timeLoss"])
                cnt_ped += 1
        #print("wt cnt: ", cnt)
        #print("max wt: ", wt_max)
        self.waiting_time_avg = wt / cnt
        self.lost_time_avg = tl / cnt
        self.lost_time_avg_ped = tl_ped / max(1,cnt_ped)
        self.lost_time_avg_bike = tl_bike / max(1,cnt_bike)
        self.waiting_time_avg_bike = wt_bike / max(1,cnt_bike)


    def get_average_queue_length_endtime(self, file):
        tree = ET.parse(file)
        root = tree.getroot()
        cnt = 0
        queue = 0
        queue_bike = 0
        cnt_bike = 0
        for time in root:
            for lanes in time:
                for lane in lanes:
                    # TODO: generalize this by changing to multiple intersections so that you could inter_id and use self.paras[inter_id][incoming_bike]
                    if lane.attrib["id"] in self.network_graph["1"]["incoming_bike"].keys():
                        queue_bike += float(lane.attrib["queueing_length"])
                        cnt_bike +=1
                    else:
                        queue += float(lane.attrib["queueing_length"])
                        cnt += 1
        #print("queue cnt: ", cnt)
        self.queue_avg = queue / cnt
        self.queue_avg_bike = queue_bike / cnt_bike


    def calculate_extra_metrics(self):
        fuel_cav, fuel_hdv, power_cav, power_hdv = self.get_instant_fuel_external_model()
        # print("fuel_cav", fuel_cav)
        # print("fuel_hdv", fuel_hdv)
        # print("power_cav", power_cav)
        # print("power_hdv", power_hdv)
        self.fuel_total_cav_external_model += fuel_cav
        self.fuel_total_hdv_external_model += fuel_hdv
        self.power_total_cav_external_model += power_cav
        self.power_total_hdv_external_model += power_hdv
        temp_cav, temp_hdv = self.get_instant_fuel_sumo()
        self.fuel_total_cav_sumo += temp_cav
        self.fuel_total_hdv_sumo += temp_hdv


    def get_instant_fuel_external_model(self):
        fuel_cav = 0
        fuel_hdv = 0
        power_cav = 0
        power_hdv = 0
        for inter_id in self.paras["network_graph"]:
            lane_id = []
            for lane in self.paras["network_graph"][inter_id]["incoming_veh"]:
                lane_id.append(lane)
            for i in range(len(lane_id)):
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                for j in range(len(cars_lane) - 1, -1, -1):
                    speed = traci.vehicle.getSpeed(cars_lane[j])
                    acc = traci.vehicle.getAcceleration(cars_lane[j])
                    fuel_temp = (
                        0.2736
                        + 0.0599 * speed
                        + 0.3547 * acc
                        - 0.0058 * speed**2
                        + 0.0179 * speed * acc
                        + 0.0663 * acc**2
                        + 0.0002 * speed**3
                        + 0.002 * speed**2 * acc
                        + 0.0245 * speed * acc**2
                        - 0.0489 * acc**3
                    )
                    power_temp_temp = (1266 * speed * acc + 1266 * 9.8 * 0.006 * speed + 1.3 * speed ** 3) / 1000
                    # power_temp = power_temp_temp if power_temp_temp > 0 else 0.9 * power_temp_temp
                    if cars_lane[j] in self.paras["veh_id_with_ev"]['cav_ice']:
                        fuel_cav += fuel_temp
                    elif cars_lane[j] in self.paras["veh_id_with_ev"]["hdv_ice"]:
                        fuel_hdv += fuel_temp
                    elif cars_lane[j] in self.paras["veh_id_with_ev"]["cav_ev"]:
                        power_cav += power_temp_temp
                    elif cars_lane[j] in self.paras["veh_id_with_ev"]["hdv_ev"]:
                        power_hdv += power_temp_temp
        return fuel_cav, fuel_hdv, power_cav, power_hdv

    def get_instant_fuel_sumo(self):
        fuel_cav = 0
        fuel_hdv = 0
        for inter_id in self.paras["network_graph"]:
            lane_id = []
            for lane in self.paras["network_graph"][inter_id]["incoming_veh"]:
                lane_id.append(lane)
            for i in range(len(lane_id)):
                cars_lane = traci.lane.getLastStepVehicleIDs(lane_id[i])
                for j in range(len(cars_lane) - 1, -1, -1):
                    fuel_temp = traci.vehicle.getFuelConsumption(cars_lane[j])
                    if cars_lane[j] in self.paras["cav_ids"]:
                        fuel_cav += fuel_temp
                    else:
                        fuel_hdv += fuel_temp
        return fuel_cav, fuel_hdv

    def close_sumo_simulation(self):
        traci.close()


    def pedestrian_actuation(self, inter_id):
        # Only runs in Actuated Scenario
        self.cur_phase.setdefault(inter_id, None)
        self.previous_phase[inter_id]=self.cur_phase[inter_id]
        self.cur_phase[inter_id] = int(traci.trafficlight.getPhase(inter_id))
        if self.cur_phase[inter_id] != self.previous_phase[inter_id]:
            self.greenTimeSofar.setdefault(inter_id,0)
            self.greenTimeSofar[inter_id] = 0
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(inter_id)
        Vehicle_Phases = [0, 2, 4, 6]
        ped_demand={}
        Gp=0
        ped_phase_map = {0: ['E', "W"], 2: [], 4: ['N', 'S'], 6: []}
        for d in self.cross_map.keys():
            ped_demand[self.cross_map[d]] = len(self.network_state[inter_id]['pedestrian_demand_current'][d])

        if self.cur_phase[inter_id] in Vehicle_Phases:
            for program in logic:
                if program.programID == "actuated_program":
                    for index, phase in enumerate(program.phases):
                        if self.cur_phase[inter_id]  == index:
                            #phase.minDur=10
                            min_dur = phase.minDur
                            max_dur = phase.maxDur
                            # for dir in ped_phase_map[self.cur_phase]:
                            #     if ped_demand[dir] > 0:
                            #         minimum_green = 3.2 + self.paras['crossing_length'] / self.paras[
                            #             'ped_speed'] + 2.7 * ped_demand[dir] / (self.paras['crossing_width'] * 3.28)
                            #         print("minimum g: ", minimum_green)
                            #         minimum_green=20
                            #         Gp = max(Gp, minimum_green)
                            # #print("Gp: ", Gp)
                            # min_dur = max(Gp, min_dur)
                            # #print("min_dur: ", min_dur)
                            # phase.minDur=min_dur
                            # traci.trafficlight.setCompleteRedYellowGreenDefinition("1", program)
                            break

            self.greenTimeSofar[inter_id] += 0.5
            self.next_time_to_change_to_actuated.setdefault(inter_id, 0)
            if self.next_time_to_change_to_actuated[inter_id] == self.greenTimeSofar[inter_id]:
                traci.trafficlight.setProgram(inter_id, "actuated_program")
                traci.trafficlight.setPhase(inter_id, self.cur_phase[inter_id])
                ped_demand, current_phase_extension, opposite_phase_delay = self.checkPresentPersons(inter_id)
                if not current_phase_extension or self.greenTimeSofar[inter_id] == max_dur: # stop the extensions
                    traci.trafficlight.setPhase(inter_id, self.cur_phase[inter_id] + 1)
                self.next_time_to_change_to_actuated[inter_id] = 0
            ped_demand, current_phase_extension, opposite_phase_delay = self.checkPresentPersons(inter_id)
            # if current_phase_extension and self.greenTimeSofar<max_dur and self.greenTimeSofar>=min_dur:
            #     print("current phase needs extension")
            #     print("status of opposite phase delays: ", opposite_phase_delay)
            if current_phase_extension and not opposite_phase_delay:  # if current phase needs extension (gap-based) and the opposite phase does not have too much delay (delay-based)
                if self.greenTimeSofar[inter_id]<max_dur and self.greenTimeSofar[inter_id]>=min_dur:
                    # stay at the current green
                    print("current phase needs extension")
                    if traci.trafficlight.getProgram(inter_id)=="actuated_program":
                        self.next_time_to_change_to_actuated[inter_id] = min(max_dur, self.greenTimeSofar[inter_id] + 10) # note that here at least 10 seconds (minimum duration) is added to the traffic signal
                        traci.trafficlight.setProgram(inter_id, "fixed_program")
                        traci.trafficlight.setPhase(inter_id, self.cur_phase[inter_id])



    def checkPresentPersons(self,inter_id):
        current_phase_extension = False
        opposite_phase_delay = False
        ped_demand={0: 0, 2: 0, 4: 0, 6: 0}
        ped_phase_map = {0: ['E', "W"], 2: [], 4: ['N', 'S'], 6: []}
        ## the plan is to extend the phase if there are any pedestrian for the current phase but not if the opposing phase is facing too much pedestrian delay
        for walking_area in self.network_graph[inter_id]["walkingarea"].keys():
            peds=traci.edge.getLastStepPersonIDs(walking_area)
            for ped in peds:
                next_edge = traci.person.getNextEdge(ped)
                directions= ped_phase_map[self.cur_phase[inter_id]]
                for dir in directions:  # check gap-based extension of current phase
                    if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == dir:
                        ped_demand[self.cur_phase[inter_id]] += 1
                        current_phase_extension = True
                opposite_directions = ped_phase_map[(self.cur_phase[inter_id]+4)%8]
                for opposite_dir in opposite_directions: # check delay-based termination according to opposite phase
                    if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == opposite_dir:
                        ped_demand[(self.cur_phase[inter_id] + 4) % 8] += 1
                        if traci.person.getWaitingTime(ped)>=25:  # if the pedestrians on the opposite phase are waiting for at least 15s, don't do the extension
                            #print("delay: ", traci.person.getWaitingTime(ped))
                            opposite_phase_delay = True

        return ped_demand, current_phase_extension, opposite_phase_delay


    # def pedestrian_movement_control(self):
    #     # only runs in Multi-Scale scenario (for now)
    #     self.cur_phase = int(traci.trafficlight.getPhase("1"))
    #     if should_update_signal:
    #         self.greenTimeSofar = 0
    #         for ped in self.stopped_peds:
    #             traci.person.setSpeed(ped, -1)
    #         self.stopped_peds=set()
    #
    #     Veh_phases_multiscale = [0, 1, 2, 3, 4, 5, 6, 7]
    #     ped_phase_map = {0:['E','W'], 1:['W'], 2:['E'], 3:[], 4:['N','S'], 5:['S'], 6:['N'], 7:[]}
    #
    #     if self.cur_phase in Veh_phases_multiscale:
    #         self.greenTimeSofar += 0.5
    #         for walking_area in self.walking_areas:
    #             peds = traci.edge.getLastStepPersonIDs(walking_area)
    #             directions = ped_phase_map[self.cur_phase]
    #             for ped in peds:
    #                 if ped not in self.stopped_peds:
    #                     next_edge = traci.person.getNextEdge(ped)
    #                     for dir in directions:
    #                         if next_edge in self.cross_map.keys() and self.cross_map[next_edge] == dir and self.greenTimeSofar>=20:
    #                             traci.person.setSpeed(ped, 0)
    #                             self.stopped_peds.add(ped)
    #
    #                             ## a problem with the behavior control is that we might assign 0 to the speed of the corresponding
    #                             # phase pedestrians but then the phase gets extended (which is unreasonable for pedestrians to stop)