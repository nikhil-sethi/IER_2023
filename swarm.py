"""
====================== swarm.py ==========================
features:
- flocking
- subswarming
- schooling
- search

Author : Nikhil Sethi
"""
import argparse
from enum import IntEnum
import time
from math import *
import threading

from tools.helper import *
import time
import socket
import json
import threading
import time
import logging
import os
from mission import Mission, Event, Role # for pickling skeleton
from tools.vector_math import Position, Velocity, clamp_vel
from tools.geodesy import get_distance, get_distance_3d
from tools.autopilot import Autopilot

class FlightMode(IntEnum):
    GUIDED = 1
    RTL = 2
    LAND = 3
    STABILIZE = 4

def vel_transfer(pos_t:list, pos_c:list, gain:float, d_t:float, d_c:float, algo:str, **kwargs):

    if algo == "prop":
        w = abs(gain * (d_c/d_t - 1)) # weight depends on gain and relative distance
        
    elif algo == "prop_max":
        w = abs(gain * (d_c - d_t)/(kwargs["d_m"] - d_t))
    elif algo == "nonlin_idx":
        if d_c > d_t:
            w = gain * ((1 - d_c/d_t)**kwargs["idx"])
        else:
            w = gain * ((1 - d_c/d_t)**kwargs["idx"])
    elif algo == "nonlin_idx_max":
        w = gain * (abs(d_c - d_t)/(kwargs["d_m"] - d_t))**kwargs["idx"]
    elif algo == "constant":
        w = gain
    if abs(d_c-d_t)<kwargs["slack"]:
        w = 0
    # print(d_c, d_t, w)
    v_del = pos_t - pos_c
    v_mag = (v_del[0]**2 + v_del[1]**2)**0.5
    v_cap = v_del/v_mag
    # need to manage z direction independently becuase alt is in different units
    return Velocity([w*v_cap[0], w*v_cap[1],0]) 
    
def calc_consensus_vel(pos_t:list, pos_c:list, gain:float, d_t:float, d_c:float, type = None):
    """ pos_t: Target position
        pos: Current positon
        gain: multiplying factor 
        d_t: Target/desired distance
        d_c: Current distance
        
        Generate a velocity towards/awayfrom target position such that current distance tends to target distance
        """
    w = gain * (d_c/d_t - 1) # weight depends on gain and relative distance
    v_x = pos_t[0] - pos_c[0]
    v_y = pos_t[1] - pos_c[1]
    if type == 'unidir':
        w = abs(w)
    if abs(w)<70:
        w=0
    # print(d_c, d_t, w)
    # print(v_x, v_y)
    return Velocity([w*v_x, w*v_y,0]) 
    

    
class Swarm:
    def __init__(self, id, sid,  simulation):
        self.id = int(id)
        self.sid = sid

        # Logging
        root_path = os.path.dirname(os.path.realpath(__file__))
        ctime = time.ctime().replace(' ', '_')[:-8].replace(':', '.')
        save_path = f"{root_path}/save/{ctime}/"
        try:
            os.mkdir(save_path)
        except FileExistsError:
            pass
        # logging defined here because parent (SwarmBot) needs it in it's init function. And creating it later doesn not work. tested.
        logging.basicConfig(filename = f"{save_path}/{self.id}_swarm.log", format = "%(module)s %(lineno)d %(message)s", filemode = "w")
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)

        try:
            # self.mission:Mission = load_pickle(f"missions_{sid}")
            self.mission:Mission = load_pickle(f"missions")[sid-1] #need to fix
        except:
            print(f"Mission 'missions_' + {sid} not found. Aborting")
            self.logger.error(f"Mission 'mission_{sid} not found. Aborting")
            exit(0)
        
        assert self.sid == self.mission.id, "Mission ID does not match with the swarm id! Check the mission file or file arguments."
        
        # Simulation addresses
        if simulation:
            sim_id = id
        else:
            sim_id = 1

        ap_addr = ("127.0.0.1", 14551+(sim_id-1)*10)    
        self.autopilot = Autopilot(ap_addr)
        self.shared_dict={"uid":0, "sid":0, "pos":[], "role":512, "tag":[]}
        self.sleep_time= .1 # 10 hz refresh rate use if doing multithreading
        self.human_lt = []
        self.search_rect = () # tuple of four gps points defining a rectangle

        
        # Multi-UAV communication
        try:
            # Client
            swarm_send_addr = ("127.0.0.1", 10000 + sim_id*10 ) # Send to multicast.py
            swarm_recv_addr = ("127.0.0.1", 10001 + sim_id*10 ) # Recv from multicast.py
            self.swarm_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.swarm_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.swarm_send_sock.connect(swarm_send_addr)

            # Server
            self.swarm_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.swarm_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.swarm_recv_sock.bind(swarm_recv_addr)  # bind address
        except Exception as err:
            print("Error in establishing connection with multicast server", err)
            raise
        
        self.heal_mask_old = [1]*5

    def send_mcast(self) -> None:
        while True:
            try:
                app_json = json.dumps(self.shared_dict, sort_keys=True).encode("UTF-8")
                self.swarm_send_sock.send(app_json)
                time.sleep(self.sleep_time)

            except Exception as err:
                self.swarm_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                print(f"[Multicast][Comms] Can't send packet ({err}). Trying again..")
                time.sleep(self.sleep_time)

    def recv_mcast(self) -> dict:
        # self.swarm_recv_sock.settimeout(0.1)
        try: 
            return json.loads(self.swarm_recv_sock.recv(10240).decode('UTF-8'))
            
        except Exception as err:
            print(f"[Multicast][Comms] Can't receive packet ({err}). Trying again..")

    def main(self):
        print ("STARTING MISSION")
        self.shared_dict["sid"] = self.sid
        self.shared_dict["role"] = self.mission.events[0].role  
        send_thread = threading.Thread(target=self.send_mcast,args=[])
        send_thread.daemon=True
        send_thread.start()
        
        min_failsafe_alti=15

        # FLIGHT ROLES
        self_healing = True
        separation = True
        cohesion = True
        alignment = False


        # params
        formation_gain = 10000  # Formation control parameter

        self.termination_modes = ['RTL', 'LAND']

        # humanrecv_thrd.start()

        N = self.mission.max_bots    # total number of global bots        
        
        # ARM AND TAKEOFF THE UAV                     
        self.autopilot.arm_and_takeoff(self.mission.takeoff_alt)

        # Create the home event. Can only be done after arming
        self.mission.events[-1].role = self.mission.events[-1].role|Role.WAYPOINT
        home_location = self.autopilot.vehicle.home_location
        self.mission.events[-1].wp_rad = 0.5
        if home_location is None:
            print("Home location is None. Defaulting to current position.")
            home_location = self.autopilot.pos
        self.mission.events[-1].waypoint = Position([home_location.lat, home_location.lon, self.mission.takeoff_alt])

        # LOOPING TILL ALL UAVS REACH THE TAKEOFF ALTITUDE
        self.wait_all_takeoff(altitude = self.mission.takeoff_alt, timeout = 20)    

        # just some initing
        self.max_id = max(max(self.mission.bots), self.mission.max_bots)
        heal_mask = [1]*self.max_id
        self.local_id = self.id
        reached = True
        self.eid = 0    # Event id
        self.ixs = 0
        self.ixe = self.max_id
        time.sleep(2)
        
        while (self.autopilot.mode not in self.termination_modes):   # while the current waypoint is not the last 
            
            # print(" ")
            self.logger.info(" ")

            pos = self.autopilot.pos # current position
            vel = self.autopilot.vel   # current velocity
            com = Position(pos[:])
            self.v_des = Velocity([0, 0, 0])    # desired velocity

            # Some state information
            self.logger.debug(f"[State][Autopilot] Position: {pos}")
            self.logger.debug(f"[State][Autopilot] Velocity: {vel}")            

            # added at the very top so that we can combine some initial operations and checks here
            if reached:  # Carry out all static updates for the next waypoint here
                try:
                    event:Event = self.mission.events[self.eid]
                    self.eid +=1
                    self.shared_dict["role"] = event.role
                    self.log_event(f"Reached event {self.eid-1}. Moving towards event {self.eid}:\n{event.__dict__}")
                except IndexError:   # if there are no more events 
                    self.log_event(f"Reached event {self.eid}. No more events left")
                    break
                if event.wait:
                    time.sleep(event.wait)
                if event.sync:
                    print("Waiting for all UAVs to sync up...")
                    self.logger.info("Waiting for all UAVs to sync up...")
                    while True:
                        
                        swarm_dict = self.recv_mcast()
                        
                        for j, uav in swarm_dict.items():
                            if heal_mask[int(j)-1] and int(j)!=self.id:
                                if uav["role"] != event.role:
                                    # time.sleep(0.2)   # DO NOT SLEEP!
                                    break
                        else:
                            print("Synced!! Continuing..")
                            self.logger.info("Synced!! Continuing..")
                            time.sleep(1)
                            break
                
                if Role.FORMATION in event.role:
                    formation = event.formation

                reached = False #reset
               
            # reached = True  # assume we reached. Check later
            swarm_dict = self.recv_mcast()  # dictionary with data from all uavs. BOTTLENECK! takes almost 0.1 seconds
            if swarm_dict == {}:    # if no data from multicast server
                continue            # skip everything

            # print(swarm_dict)

            if self_healing:
                
                heal_mask_old = heal_mask[:] # indexing copies the entire array instead of reference. TODO could make this better 
                mask_changed = self.update_heal_mask(heal_mask, heal_mask_old, self.max_id, swarm_dict, self.termination_modes, pos)    # O(N(N))                
                self.num_active_bots = sum(heal_mask)  # number of legit uavs in subswarm
                if mask_changed:
                    print("[Self-healing][Comm] Configuration of some UAVs changed!! Healing..")
                    self.logger.info("[Self-healing][Comm] Configuration of some UAVs changed!! Healing..")
                    
                    self.local_id = sum(heal_mask[:self.id])  # id the local subswarm
                    print(f"[Self-healing][Comm] Online UAVs: {heal_mask}")
                    self.logger.debug(f"[Self-healing][Comm] Online UAVs: {heal_mask}")            

            self.logger.debug(f"[Swarm][Comm] Swarm data: {swarm_dict}")    # healed swarm data

            if self.num_active_bots == 0:
                continue

            # SWARM FUNCTIONS
            for j, uav in enumerate(swarm_dict.values()):
                # j = int(j)
                if self.id == uav["uid"] or not uav:
                    continue

                lat,lon,_,alt = uav["pos"]
                pos_j = Position([lat, lon, alt])

                dij_c = get_distance_3d(pos, pos_j) # current distance between i'th and j'th uav

                # GLOBAL SWARM FUNCTIONS
                
                # collision avoidance
                if dij_c <= 10:  # ALWAYS DO THIS AND JUST THIS. COMPLETELY INDEPENDENT FAILSAFE
                    v_coll = vel_transfer(pos, pos_j, 10, event.min_sep_dist, dij_c, algo="prop", slack = 0)
                    print(f"Too close to UAV {j}!! Separating like Ross and Rachel..")
                    self.v_des += v_coll
                    # self.autopilot.update_vel(self.v_des*2)
                    clamp_vel(self.v_des, 0.5, event.speed)
                    break

                # LOCAL/SUB SWARM FUNCTIONS
                if heal_mask[uav["uid"]-1]:
                    
                    if Role.FLOCK in event.role:    # Separation, Cohesion, alignment
                        
                            if separation:
                                if dij_c <= event.min_sep_dist:  # for neighbors only
                                    v_sep = vel_transfer(pos, pos_j, event.speed, event.min_sep_dist, dij_c, algo="prop", slack = 0) # Dont change acceleration if you don't know what you're doing. Increase gain if you think the maximum seperation velocity is too less even at small iteragent distances
                                    # print(f"v_sep: {v_sep} | {v_sep.mag()}")
                                    self.v_des += v_sep
                            if cohesion:
                                if 35 <= dij_c <= 60:  # for neighbors only
                                    v_cohere = vel_transfer(pos_j, pos, event.speed, 35, dij_c, algo="prop", slack = 0) 
                                    self.v_des += v_cohere
                                    # print(f"v_coh: {v_cohere} | {v_cohere.mag()}")
                            if alignment:
                                pass
                            
                    elif Role.FORMATION in event.role:
                        local_id_j = sum(heal_mask[:uav["uid"]])
                        # v_formation = vel_transfer(pos_j, pos, 10, dij_t_healed[self.id-1][j-1], dij_c, algo= "prop")

                        dij_form = event.form_min_dist * formation[self.local_id-1][local_id_j-1].mag()
                        pij_form = formation[local_id_j-1][self.local_id-1]
                        theta_ij_form =  pij_form.bearing()
                        
                        v_formation_dist = calc_consensus_vel(pos_j, pos, formation_gain, dij_form, dij_c)

                        # print(v_formation_dist.mag())
                        self.v_des += v_formation_dist

                    if Role.SCHOOL in event.role:
                        com += pos_j

           # velocity post processing for all roles
            clamp_vel(self.v_des, 0, event.speed)


            ## ================================================================ ##
            ##                         FAILSAFES                                ##
            ## ================================================================ ##

            if pos.alt < min_failsafe_alti:
                self.autopilot.mode = "LAND"
                print(f"[Failsafe][Autopilot] Altitude less than {min_failsafe_alti}! Landing the UAV.")
                self.logger.critical(f"[Failsafe][Autopilot] Altitude less than {min_failsafe_alti}! Landing the UAV.")
                break


            ## ================================================================ ##
            ##                 UPDATE FINAL VELOCITY TO THE UAV                 ##
            ## ================================================================ ##
            # print(f"v_des: {self.v_des} | {self.v_des.mag()}")
            self.autopilot.update_vel(self.v_des)
            # print("\n")
            
        ## ================================================================ ##
        ##                       MAIN LOOP ENDS OVER HERE                   ##
        ## ================================================================ ##

        self.log_event("MISSION COMPLETED!")
        
        time.sleep(2)   # To make all the UAVs land at the same time
        if self.autopilot.mode not in self.termination_modes:
            self.autopilot.mode = "LAND"
        self.autopilot.vehicle.close()

    def wait_all_takeoff(self, altitude, timeout):
        """wait till all uavs reach 'altitude' or until 'timeout' """
        start = time.time()
        print("[Takeoff][Swarm] Waiting for other UAVs to takeoff...")
        self.logger.info("[Takeoff][Swarm] Waiting for all UAVs to takeoff...")
        while (time.time() - start) < timeout:
            # time.sleep(0.5) # DO NOT SLEEP!! for some reason the recv mcast function doesnt give right values if we do so. 
            swarm_dict = self.recv_mcast()
            # swarm_dict = filter_remove_LAND_RTL_data(swarm_dict)
            for id, uav in swarm_dict.items():
                if self.sid != uav["sid"] or uav["mode"] in self.termination_modes:
                    continue
                _, _, _, alt = uav["pos"]
                if float(alt) < altitude * 0.95:    # if takeoff is still undergoing, skip this uav
                    break   # break from for loop                
            else:   # if all uavs are above takeoff altitude
                print(f"[Takeoff][Autopilot] All UAVs at altitude = {altitude} metres. Moving towards first event.")
                self.logger.info(f"[Takeoff][Swarm] All UAVs at altitude = {altitude} metres. Moving towards first event.")
                return   # break from while loop 
        print(f"[Takeoff][Swarm] Timed out after {timeout} seconds. Moving towards first event.")
        self.logger.info(f"[Takeoff][Swarm] Timed out after {timeout} seconds. Moving towards first event.")

    @staticmethod
    def check_reached(target, current, tolerance)-> bool:
        dist = get_distance_3d(target, current)
        if dist < tolerance:
            return True
        return False

    def log_event(self, str):
        print(" ================================================================ ")
        print(f"                     {str}")
        print(" ================================================================ ")
        self.logger.info(" ================================================================ ")
        self.logger.info(f"                     {str}")
        self.logger.info(" ================================================================ ")

    
    def update_heal_mask(self, heal_mask, heal_mask_old, N, swarm_dict, heal_modes, pos):
        """returns a logical array with 0 corresponding to ids that need to be healed
            nikhil_old speed with return (N=5, 2 heal modes): 2.63 µs ± 45.5 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
            nikhil_new speed with in place (N=5, 2 heal modes): 1.87 µs ± 5.47 ns per loop (mean ± std. dev. of 7 runs, 1000000 loops each)
        """
        # heal_mask = [1]*N   # try not to create this everytime
        changed = False
        for id in range(N):
            if id == self.id-1:
                continue
            # heal_bool = heal_mask[id]
            
            if str(id+1) not in swarm_dict.keys():  # CONDITION #1 (communication healing): <=O(N)
                heal_mask[id] = 0   # AND gate
            else:
                uav = swarm_dict[str(id+1)]
                if uav["mode"] in heal_modes:         # CONDITION #2 (mode healing): O(1)
                    heal_mask[id] = 0  # AND gate
                    # del swarm_dict[str(id+1)]
                elif uav["sid"] != self.sid:            # CONDITION #3 (subswarm healing): O(1)
                    heal_mask[id] = 0
                elif Role.HOME in Role(uav["role"]):    # CONDITION #4 (RTL healing): O(1)
                    heal_mask[id] = 0
                elif get_distance( Position(uav["pos"]), pos) > 200 and (Role.SCHOOL in Role(uav["role"]) or Role.SEARCH not in Role(uav["role"])):
                    heal_mask[id] = 0
                else:
                    heal_mask[id] = 1   # so that it can heal back from a previously terminating condition

            if heal_mask[id]^ heal_mask_old[id]:    # XOR gate, if mask val changed
                print(f"[Self-healing][Comm] Config of UAV {id+1} changed!!")
                self.logger.debug(f"[Self-healing][Comm] Config of UAV {id+1} changed!!")
                changed =  True # shows change in entire mask however small
        return changed

    def min_time_to_event(self, event, pos):
        return get_distance_3d(Position(event.waypoint), pos)/event.speed()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", type=int, nargs="?", default=0, const=1)
    parser.add_argument("--id", type = int, default = 1)
    parser.add_argument("--sid", type = int, default = 1)
    args = parser.parse_args()
    swarm = Swarm(id = args.id, sid = args.sid, simulation = args.simulation)
    
    swarm.main()
    