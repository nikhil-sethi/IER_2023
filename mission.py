import argparse
import pickle
from enum import IntEnum, IntFlag
import os
from tools.helper import load_pickle
import pprint
pp = pprint.PrettyPrinter(indent=4)
import json
from types import SimpleNamespace
from settings import Project_path


class FlightMode(IntEnum):
    GUIDED = 1
    RTL = 2
    LAND = 3
    STABILIZE = 4

class Role(IntFlag):    # allows composition i.e. Multi-Role
    FORMATION = 1   # A specific shape defined by the dij matrix
    WAYPOINT =  2
    SEARCH =    4  # Cover a search area defined by typle of 2d coordinates
    FLOCK =     8   # Maintain separation, cohesion and alignment
    STRIKE =    16  # strike a target that is being followed
    SCHOOL =    32  # Follow a waypoint as a complete swarm
    TRACK =     64  # Track an object of interest until it further notice or out of view
    FOLLOW =    128 # Follow target which is being tracked with constant distance
    DETECT =    256 # object detection
    HOME =      512
    ALLOCATE =  1024    # ongoing task allocation for 

    def __sub__(self, __x: int) -> int:
        return self.__class__(super().__sub__(__x))

class Event:
    """An event defines a change in configuration in the mission of the uav
    when an event is given, the uav follows the given config upto the waypoint 
    """
    # id = 0
    min_speed = 5
    max_speed = 20

    def __init__(self, role:int, speed:int = 5, wait = 1, sync = False, min_sep_dist = 30, **kwargs) -> None:
        # Event.id += 1
        self.role = role    # defines whether this event marks search, free-flocking or formation flocking etc.
        self.speed = speed  # speed(m/s) to pursue upto this waypoint   
        self.wait = wait   # seconds to wait before startng this event. Adds stability to the mission
        self.min_sep_dist = min_sep_dist
        self.sync = sync

        if Role.FORMATION in self.role:
            
            self.formation = kwargs["formation"].copy()  # dij_t_matrix obtained from coordinates plotted in gui
            try:
                self.form_min_dist = kwargs["form_min_dist"]

            except KeyError:
                self.form_min_dist = 30 # Kilometers
                # print(f"Minimum distance between formation not provided for event number {self.id}. Defaulting to {30} meters")

        if Role.SEARCH in self.role:
            try:
                self.search_poly = kwargs["search_poly"]  # list of tuples as 2D coordinates (TL, TR, BR, BL)
                self.d_row = kwargs["d_row"] # meters to degrees
                self.search_alt = kwargs["search_alt"]
            except KeyError:
                print(f"Please provide a valid arguments for event number {self.id}")
                raise
        # if Role.FLOCK in role:
        #     try:
        #         # self.sep_flag = kwargs["sep_flag"]
        #         self.sep_gain_factor = event.speed
        #     except KeyError:
        #         print(f"Please provide a valid search polygon for event number {self.id}")
        #         raise
        
        # add any further requirements based on roles over here
   

class Mission():
    def __init__(self,id, bots, events, max_bots, takeoff_alt = 20) -> None:
        self.id = id
        self.events:list[Event] = events.copy()   # list of event objects. Need the copyy if were maintaining multiple missions in files
        # self.search_rect:tuple = () # length (m), breadth (m), heading (deg)
        self.bots:list = bots  # list<int> of uav sysids who have subscribed to this mission
        # self.bots.sort()    # DO NOT REMOVE THIS!!
        self.max_bots = max_bots
        # self.interuav_zdiff:int = 5  # difference in height(m) between each uav. usually done for even uavs only  TODO make this dynamic not same for entire mission
        self.takeoff_alt = takeoff_alt

        self.events.append(Event(id = len(events)+1, role=Role.HOME, speed = 2, min_sep_dist = 5, sep_gain_factor = 2))    # The default home event. Will be managed more carefully by the swarm file

    def parse_search_rect(self):
        "Convert user specified search rectangle into gps coordinates"
        pass

    def __repr__(self):
        events_str = "".join(f"\n   {i+1}: {e.__dict__}" for i, e in enumerate(self.events))
        return f"\nMission {self.id}: \n bots: {self.bots} \n max_bots: {self.max_bots} \n takeoff_alt: {self.takeoff_alt}\n events: {events_str} \n"


def parse_mission_json(data):
    # Parse JSON into an object with attributes corresponding to dict keys.
    x = json.loads(data, object_hook=lambda d: SimpleNamespace(**d))

    mission_events=[]

    for single_event in x.events:
        e = Event(role=Role(single_event.role), waypoint=single_event.waypoint, wp_rad=single_event.wp_rad, speed=single_event.speed,
                 min_dist=single_event.min_dist, search_poly = single_event.search_poly,id = single_event.id, wait=single_event.wait)
        mission_events.append(e)
    
    mission = [
                Mission(id=x.id, bots=x.bots, events=mission_events, max_bots=x.max_bots, takeoff_alt=x.takeoff_alt)
                ]

    return mission, x.id
    

 
if __name__ == "__main__":

    test_formation = pickle.load(open('formation', 'rb'))

    events1 = [ 
                Event(role=Role.FORMATION, waypoint = (28.32713117, 77.41700224, 20), wp_rad = 20, speed = 10, form_min_dist=50, formation = test_formation),
            ]


    missions = [Mission(id = 1, bots= list(range(1,6)), events = events1, max_bots = 5, takeoff_alt = 30), 
             ] 

    # temporary stuff to customise for lack of GUI/GCS

    parser = argparse.ArgumentParser()
    parser.add_argument("--write", nargs='?', const=True)
    parser.add_argument('--writeJson', nargs='?', const=True)

    args = parser.parse_args()
    if args.write:
        with open('missions', 'wb') as f:
            pickle.dump(missions, f)
        # os.system(f"cp missions {Project_path.UAV}/.")
    elif args.writeJson:
        new_missions , file_id= parse_mission_json(args.writeJson)
        miss_path = os.path.join(Project_path.GCS, f'missions_{file_id}')
        with open(miss_path, 'wb') as f:
            pickle.dump(new_missions, f)
        os.system(f"cp {miss_path} {Project_path.UAV}/.")
        print("write misiion success")

    
        
"""
swarm1=28.32652130,77.41895210,0,0
swarm2=28.32682130,77.41849156,0,0
swarm3=28.32655410,77.41849156,0,0
swarm4=28.32682130,77.41895210,0,0
swarm5=28.32712130,77.41895210,0,0
"""