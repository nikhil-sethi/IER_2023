import time
import multiprocessing
import json
import threading
import logging
import os
import argparse

from tools.networking import createUDPLink, createMulticastLink
from tools.autopilot import Autopilot

class Multicast:
    def __init__(self, id, simulation = False):
        # Bio
        self.id = id

        self.public_dict = {}
        # make these variables private. HAHAHAHAHAHA dis is python vro

        # Logging
        root_path = os.path.dirname(os.path.realpath(__file__))
        ctime = time.ctime().replace(' ', '_')[:-8].replace(':', '.')
        save_path = f"{root_path}/save/{ctime}/"
        try:
            os.mkdir(save_path)
        except FileExistsError:
            pass
        logging.basicConfig(filename = f"{save_path}/{self.id}_multicast.log",format=' %(module)s %(lineno)d %(asctime)s %(message)s', filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)

        # Communication variables
        self.sleep_time = .05
        self.refresh_rate = 1
        self.packet_counter = 0

        if simulation:
            self.sim_id = id
        else:
            self.sim_id = 1

        # Autopilot. Will be created in a separate process
        self.ap_addr = ("127.0.0.1",14550 + (self.sim_id-1)*10)

        # Create links among servers

        # Vision link only receives
        classifier_rx_addr = ("127.0.0.1", 10002 + self.sim_id*10)  # client
        self.classifier_rx_sock = createUDPLink(classifier_rx_addr, is_server = False)

        # client
        swarm_tx_addr = ("127.0.0.1", 10001 + self.sim_id*10)   

        print(f"[Swarm][Init] Creating client at {swarm_tx_addr}..")
        self.logger.info(f"[Swarm][Init] Creating client at {swarm_tx_addr}..")
        self.swarm_tx_sock = createUDPLink(swarm_tx_addr, is_server = False)
        print(f"[Swarm][Status] Done.")
        self.logger.info(f"[Swarm][Status] Done.")

        # Multicast links
        multicast_addr = ("239.0.0.0", 60001)
        bind_addr = ("0.0.0.0", 60001)
        self.mcast_rx_sock = createMulticastLink(multicast_addr, bind_addr, is_server = True)
        self.mcast_rx_sock.settimeout(0.1)
        self.mcast_tx_sock = createMulticastLink(multicast_addr, bind_addr, is_server = False)
        self.mcast_tx_sock.settimeout(0.1)

        self.local_ip = "127.0.0.1"
        self.max_human_limit = 2
        self.was_filled = False
        
    
        self.main_thread()

    def get_encoded_telemetry(self):
        pos = self.autopilot.pos
        heading = self.autopilot.vehicle.heading
        mode  = self.autopilot.vehicle.mode.name
        battery = self.autopilot.vehicle.battery.voltage
        attitude = [self.autopilot.vehicle.attitude.pitch, self.autopilot.vehicle.attitude.roll, self.autopilot.vehicle.attitude.yaw]
        armed = self.autopilot.vehicle.armed
        # print(mode)
        if self.autopilot.vehicle._heartbeat_timeout :
                mode = "LAND"
        else:
            self.packet_counter += 1
        return self.id, mode, pos.lat, pos.lon, heading, pos.alt, self.packet_counter, battery, attitude, armed

    def recv_packets(self, dt):
        print(f"[Multicast][Init] Starting server..")
        self.logger.info(f"[Multicast][Init] Starting server..")

        static_dict = {}
        modes = ["GUIDED","RTL","LAND"]
        while True:

            shared_dt = dt["DICT"]
            arr = set(dt["NEW_UAV"])

            try:
                message = self.mcast_rx_sock.recv(2048)
                data = json.loads(message.decode('utf-8'))
                uav_id = int(data["uid"])
                data["rx_t"] = time.time()

                if uav_id not in static_dict.keys():
                    static_dict[uav_id] = data
                    static_dict[uav_id]["rx_t"] = time.time()
                    if data["mode"] in modes:

                        if uav_id not in shared_dt.keys():
                            shared_dt[uav_id] = data###jo keys static dictionary mein nahi hain wo shared data dict mein store karaya hai
                            shared_dt[uav_id]["rx_t"] = time.time()
                            static_dict[uav_id] = data
                            static_dict[uav_id]["rx_t"] = shared_dt[uav_id]["rx_t"]
                            arr.add(uav_id)

                        elif data["tx_t"] > shared_dt[uav_id]["tx_t"]:
                            arr.add(uav_id)
                            shared_dt[uav_id] = data
                            shared_dt[uav_id]["rx_t"] = time.time()
                            static_dict[uav_id] = data
                            static_dict[uav_id]["rx_t"] = shared_dt[uav_id]["rx_t"]
                            # print("newpacket", key)
                        else:
                            # print("Neglecting Late packet")
                            pass
                else:

                    if data["tx_t"] > static_dict[uav_id]["tx_t"]:
                        static_dict[uav_id] = data
                        static_dict[uav_id]["rx_t"] = time.time()
                        if data["mode"] in modes:
                            if uav_id not in shared_dt.keys():
                                shared_dt[uav_id] = data
                                shared_dt[uav_id]["rx_t"] = time.time()
                                static_dict[uav_id] = data
                                static_dict[uav_id]["rx_t"] = shared_dt[uav_id]["rx_t"]
                                arr.add(uav_id)

                            elif data["tx_t"] > shared_dt[uav_id]["tx_t"]:
                                arr.add(uav_id)
                                shared_dt[uav_id] = data
                                shared_dt[uav_id]["rx_t"] = time.time()
                                static_dict[uav_id] = data
                                static_dict[uav_id]["rx_t"] = shared_dt[uav_id]["rx_t"]
                                # print("newpacket", key)
                            else:
                                # print("Neglecting Late packet")
                                pass

                    else:
                        # data not adding in shared list if mode land or RTL
                        pass
            except Exception as err:
                print("[Multicast][Error] Data not received: ", err)
                pass

            # check if a data is in the dictionary for more than refresh sec
            try:

                shared_dt = {key: data for (key, data) in shared_dt.items() if time.time() - data["rx_t"] < self.refresh_rate}
            except Exception as err:
                # print("Error in recv pack",err)
                pass

            dt["DICT"] = shared_dt
            dt["STATIC_DICT"] = static_dict

    def send_packets(self, dt, was_filled):
        
        print("[Multicast][Init] Starting client.")
        self.logger.info("[Multicast][Init] Starting client..")
        print(f"[Autopilot][Init] Starting autopilot at {self.ap_addr}..")
        self.logger.info(f"[Autopilot][Init] Starting autopilot at {self.ap_addr}..")
        try:
            self.autopilot = Autopilot(self.ap_addr)    # created here becuase this function exists in a seperate process. Otherwise references would not update.
            # self.vehicle = self.autopilot.vehicle
        except: 
            # wait loop for 
            while True:
                try:
                    uav_id, mode, lat, lon, heading, alti, packet_counter, battery, attitude, armed = self.get_encoded_telemetry()  # from gps_package
                    break
                except Exception as err:
                    print("Autopilot not ready yet. Trying again..:", err)
                    time.sleep(1)

        print(f"[Autopilot][Status] Connected.")
        self.logger.info(f"[Autopilot][Status] Connected.")

        # swarm link (both send and receive)
        swarm_rx_addr = ("127.0.0.1", 10000 + self.sim_id*10)    # server

        print(f"[Swarm][Init] Creating server at {swarm_rx_addr}..")
        self.logger.info(f"[Swarm][Init] Creating server at {swarm_rx_addr}..")
        self.swarm_rx_sock = createUDPLink(swarm_rx_addr, is_server = True)
        print(f"[Swarm][Status] Done.")
        self.logger.info(f"[Swarm][Status] Done.")
        self.recv_dt = {}


        # start Receiving from swarm 
        threading.Thread(target=self.recv_from_swarm).start()
        

        temp = {"uid": 0,"sid":0, "mode":"","pos": [],"pid": 0, "tx_t": 0.0,"rx_t": 0.0, "bat": 0.0, "att":[], "arm": False}

        
        while True:
            shared_dt = dt["DICT"]
            try:
                uav_id, mode, lat, lon, heading, alti, packet_counter, battery, attitude, armed  = self.get_encoded_telemetry()  # from gps_package
                if shared_dt == {}:
                    temp["uid"] = uav_id
                    temp["mode"] = mode
                    temp["pos"] = [lat, lon, heading, alti]
                    temp["pid"] = packet_counter
                    temp["tx_t"] = time.time()
                    temp["rx_t"] = 0.0
                    temp["bat"] = battery
                    temp["att"] = attitude
                    temp["arm"] = armed
                    shared_dt[uav_id] = temp
            except Exception as err:
                print("error in encoded_telemtry", err)
                continue ##agar error aata hain  then again repeat while True loop.

            try:
                uav_id, mode, lat, lon, heading, alti, packet_counter, battery, attitude, armed  = self.get_encoded_telemetry()  # from gps_package
                # order is very imp
                temp["uid"] = uav_id
                temp["mode"] = mode
                temp["pos"] = [lat, lon, heading, alti]
                temp["pid"] = packet_counter
                temp["tx_t"] = time.time()
                temp["rx_t"] = 0.0
                temp["bat"] = battery
                temp["att"] = attitude
                temp["arm"] = armed
                if self.recv_dt != {}:  # from swarm.py
                    temp["tag"] = self.recv_dt["tag"]
                    temp["role"] = self.recv_dt["role"]
                    temp["sid"] = self.recv_dt["sid"]


            except Exception as err:
                print("error in assigment in recv_data", err)
            shared_dt[uav_id] = temp
            try:
                temp = shared_dt[uav_id]
                if temp is None or temp == {}:
                    continue
                else:
                    try:
                        # print(temp["mode"])
                        app_json = json.dumps(temp).encode("UTF-8")
                        self.mcast_tx_sock.send(app_json)
                        # print("...............SENT MESSAGE TO OTHER UAVS............")
                    except Exception as err:
                        # self.send_sock.close()
                        # self.send_sock = self.create_sender_socket()
                        print("Error in send packet:", err)
            except Exception as err:
                print("Error in Send Intel ", err)

            time.sleep(self.sleep_time) ###sleep time may be for interval

    def send_to_swarm(self, dt):
        # print(dt)
        try:
            packet = json.dumps(dt, sort_keys=True).encode("UTF-8")
            self.swarm_tx_sock.send(packet)
        except Exception as err:
            print("[Swarm][Error] Data not sent to Swarm: " + str(err))
            self.logger.critical("[Swarm][Error] Data not semt to Swarm: " + str(err))
    
    def recv_from_swarm(self):
        print("[Swarm][Init] Starting server..")
        self.logger.info("[Swarm][Init] Starting server..")
        while True:
            try:
                packet = self.swarm_rx_sock.recv(2048)
                self.recv_dt = json.loads(packet.decode('utf-8'))

            except Exception as err:
                print("[Swarm][Error] Data not received from Swarm: " + str(err))
                self.logger.critical("[Swarm][Error] Data not received from Swarm: " + str(err))

    def clear_shared_list(self, dt, threshlod_time, was_filled):
        while True:
            shared_dt = dt["DICT"]
            arr = set(dt["NEW_UAV"])
            if shared_dt is None or shared_dt == {}:
                time.sleep(1)
                continue
            try:
                was_filled.value = 1
                for key, data in shared_dt.items():
                    if time.time() - data["rx_t"] < threshlod_time:
                        shared_dt[key] = data
                    else:
                        if int(key) in arr:
                            arr.remove(int(key))
                # print("Clearing DICT",arr)
            except Exception as err:
                print("Error in clearing list ", err)
            dt["DICT"] = shared_dt
            dt["NEW_UAV"] = arr
            time.sleep(self.sleep_time)

    def main_thread(self):
        """
        - spawn threads
            - Receiver
            - Sender: 
            - clearer: Clears the shared dictionary
        """

        with multiprocessing.Manager() as manager: ###manager banaya hai.###multiprocessing is used for parallel processing with multiple input.

            shared_dict = manager.dict({"DICT": {}, "STATIC_DICT": {}, "NEW_UAV": []})
            self.was_filled = multiprocessing.Value("i", 0)

            send_process = multiprocessing.Process(target=self.send_packets, args=[shared_dict, self.was_filled])
            recv_process = multiprocessing.Process(target=self.recv_packets, args=[shared_dict])

            send_process.start()
            recv_process.start()
            
            clear_thread = threading.Thread(target=self.clear_shared_list, args=(shared_dict, self.refresh_rate, self.was_filled))
            clear_thread.start()

            log_time = time.time() #time taken to do so.

            print("[Swarm][Init] Starting client..")
            self.logger.info("[Swarm][Init] Starting client..")
            while True:
                temp = shared_dict["DICT"].copy()
                # print(temp)
                if time.time() - log_time > 1: # log in intervals of 1 second
                    self.logger.info("\n")
                    print("[Multicast][Comms] Online bots: ", sorted(list(temp.keys())))
                    self.logger.info(f"[Multicast][Comms] Online bots: {sorted(list(temp.keys()))}")
                    self.logger.debug(temp)
                    
                    log_time = time.time()

                self.send_to_swarm(temp)
                time.sleep(.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", type=int, nargs="?", default=0, const=1)
    parser.add_argument("--id", type = int, default = 1)

    try:
        args = parser.parse_args()
        Multicast(id = args.id, simulation = args.simulation)
    except Exception as err:
        print("[Multicast][Global] Could not start multicast server: ", err)
