import argparse
import os
import pickle
from enum import IntEnum
import time
from mission import Mission,Event, Role

class TmuxUAVPane(IntEnum):
    AUTOPILOT = 0
    MULTICAST = 1
    MISSION = 2

class TmuxGCSPane(IntEnum):
    BACK = 0
    FRONT = 1
    SYSTEM = 2 

parser = argparse.ArgumentParser()

# Things to manage
parser.add_argument("--gcs", type=bool, nargs="?", default=False, const=True)
parser.add_argument("--server", type=bool, nargs="?", default=False, const=True)
parser.add_argument("--mission", type=bool, nargs="?", default=False, const=True)
parser.add_argument("--project",  type=bool, nargs="?", default=False, const=True)

# Optional arguments
parser.add_argument("--simulation", type=int, nargs="?", default=0, const=1)
parser.add_argument("--uid", type=int, default=1)
parser.add_argument("--sid", type=int, default=1)
parser.add_argument("--purge", type=bool, nargs="?", default=False, const=True) # for --project
parser.add_argument("--write", type=bool, nargs="?", default=False, const=True) # for --project
parser.add_argument("--get_logs", type=bool, nargs="?", default=False, const=True) # for --project
parser.add_argument("--kill", type=bool, nargs="?", default=False, const=True)


args = parser.parse_args()


def load_pickle(file_name):
    with open(file_name, 'rb') as f:
        file = pickle.load(f)
    return file

def manage_server(id, is_sim, ap_if = None, obc = None, kill=False, uname='', host=''):

    if kill:
        cmd = f"tmux kill-session -t 'uav{id}'"
        if is_sim:
            os.system(cmd)
            os.system("killall xterm") # for ardupilot binary
        else:
            obc.exec_command(cmd)
    else:   
        gcs_cmd = f"gnome-terminal --title 'UAV {id}' --tab -- "

        server_cmd = f"tmux new -d -s uav{id} 'tmux source-file ./.tmux_uav.conf'"  # need to start as detached process coz it won't run the processes otherwise
        multicast = f"tmux send-keys -t uav{id}:1.{TmuxUAVPane.MULTICAST} 'python3 multicast.py --id {id} --simulation {int(is_sim)}' Enter"

        if is_sim:
            gcs_cmd += f"tmux attach-session -t uav{id}"
            autopilot = f"tmux send-keys -t uav{id}:1.{TmuxUAVPane.AUTOPILOT} 'sim_vehicle.py --model=x -v ArduCopter --out=127.0.0.1:{14554+(id-1)*10} --out=127.0.0.1:{14553+(id-1)*10} --out=127.0.0.1:{14552+(id-1)*10} --sysid {id} -I{id-1} -L swarm{id}' Enter"
            
            os.system(server_cmd)  # start servers first then gcs
            time.sleep(1)
            os.system(f"{multicast};{autopilot}")
            
        else:
            gcs_cmd += f"ssh -t {uname}@{ip} tmux attach-session -t uav{id}"
             
            autopilot = f"tmux send-keys -t uav{id}:1.{TmuxUAVPane.AUTOPILOT}  'mavproxy.py --master {ap_if} --out=127.0.0.1:14554 --out=127.0.0.1:14553 --out=192.168.2.60:{15000+id} --out=127.0.0.1:14551 --out=127.0.0.1:14550' Enter "

            # run cmd on ssh id
            _in,_out,_err = obc.exec_command(f"{server_cmd};{multicast};{autopilot}")
            print(_err.read().decode())
            print(_out.read().decode())
        os.system(gcs_cmd)


def manage_gcs(simulation = False, kill = False):
    if kill:
        os.system(f"killall xterm; tmux kill-server;killall -9 mavproxy.py")
        exit(0)
    else:
        mavp_cmd = "mavproxy.py --map --console "
        for i in range(5):
            mavp_cmd += f"--master udp:127.0.0.1:{14552+10*i} "

        cmd = f"gnome-terminal --title 'GCS' --tab -- tmux new -s gcs 'tmux source-file ./.tmux_gcs.conf'"
        gui_backend = f"tmux send-keys -t gcs:1.{TmuxGCSPane.BACK} '{mavp_cmd}' Enter"
        gui_frontend = f"tmux send-keys -t gcs:1.{TmuxGCSPane.FRONT}  Enter"

        os.system(f"{cmd};{gui_backend};{gui_frontend}")

def manage_mission(id, sid, is_sim):
    swarm = f"tmux send-keys -t uav{id}:1.{TmuxUAVPane.MISSION} '^Z' 'python3 -i swarm.py --id {id} --sid {sid} --simulation {int(is_sim)}' Enter"
    os.system(swarm)

obc = None
ap_if = None
username = None
ip = None

if args.sid:
    mission:Mission = load_pickle("missions")[args.sid-1]

if args.gcs:
    manage_gcs(kill = args.kill)

if args.mission:
    for id in mission.bots:
        manage_mission(id, args.sid, args.simulation)

if args.server:
    for id in mission.bots:
        manage_server(id, args.simulation, obc=obc, kill = args.kill, ap_if=ap_if, uname=username, host=ip)
