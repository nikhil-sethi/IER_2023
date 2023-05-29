import dronekit
import time
from .vector_math import Position4D, Position, Velocity
from pymavlink import mavutil

class Autopilot:
    def __init__(self, address:tuple) -> None:
        # self.conn = mavutil.mavudp(device = f"{address[0]}:{address[1]}")
        self.vehicle = dronekit.connect(f"{address[0]}:{address[1]}", wait_ready=False)  
        
    @property
    def pos(self):
        return Position([self.vehicle._location._lat, self.vehicle._location._lon, self.vehicle._location._relative_alt])

    @property
    def vel(self):
        return Velocity([self.vehicle._vx, self.vehicle._vy, self.vehicle._vz])

    @property
    def pos_4d(self):
        return Position4D([self.vehicle._location._lat, self.vehicle._location._lon, self.vehicle._location._relative_alt, self.vehicle.heading])

    @property
    def mode(self):
        return self.vehicle.mode
    
    @mode.setter
    def mode(self, val:str):
        self.vehicle.mode = dronekit.VehicleMode(val.upper())

    def update_bearing(self,heading_of_wp):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading_of_wp,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def update_vel(self, vel):
        """
        vel: new velocity vector
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            vel[0],  # X velocity in NED frame in m/s
            vel[1],  # Y velocity in NED frame in m/s
            vel[2],  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)  

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Performs arming checks, arms and takes-off to required altitude
        """
        while not self.vehicle.is_armable:
            print("[Init][Autopilot] Waiting for autopilot to initialise...")
            time.sleep(1)

    
        # Copter should arm in GUIDED mode
        while self.mode != "GUIDED":
            print("[Init][Autopilot] Changing the vehicle mode to GUIDED")
            self.mode = "GUIDED"
            time.sleep(1)

        print("[Init][Autopilot] Arming motors.")
        self.vehicle.armed = True
        self.vehicle.flush()

        while not self.vehicle.armed:
            print("[Init][Autopilot] Waiting for arming...")
            self.vehicle.armed = True
            self.vehicle.flush()
            time.sleep(1)

        ## ====================================================================== ##
        ##                           ARM AND TAKEOFF TESTING                      ##
        ## ====================================================================== ##
        
        assert 20 <= aTargetAltitude, f"[Check][Autopilot] Target altitude({aTargetAltitude}) must be at least 20 meters"
        self.vehicle.simple_takeoff(aTargetAltitude)

        while self.pos.alt <= aTargetAltitude * 0.90:
            print("[Takeoff][Autopilot] Altitude: ", self.pos.alt)
            time.sleep(1)
        print("[Takeoff][Autopilot] Reached target altitude")
