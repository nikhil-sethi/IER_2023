import socket
from math import *
import json

from .geodesy import pointRadialDistance, get_distance
import pickle

def rectangle(m,n,lat,lon,head):
    """
              (lat, lon)
        p2 ------ x ------ p1
         |  m/2   |   m/2   |                    
         |        |         |                    
         |        |         |   |                   
         |      n |         |   |
         |        |         |   | (head)                 
         |        |         |   |                 
         |        |         |   *                 
         |        |         |                    
         p3 ----- e ------ p4
                  
    """
    heading=float(head)
    heading=radians(heading)
    e=pointRadialDistance(lat,lon,heading,n)
    rheading=heading+pi/2
    lheading=heading-pi/2
    p1=pointRadialDistance(lat,lon,lheading,m/2)
    p2=pointRadialDistance(lat,lon,rheading,m/2)
    p3=pointRadialDistance(e[0],e[1],rheading,m/2)
    p4=pointRadialDistance(e[0],e[1],lheading,m/2)
    return p1,p2,p3,p4

def rectangle2(pos1,heading,fol,fow):
    u=pointRadialDistance(pos1[0],pos1[1],heading-pi,fow/2)
    n=[]
    n=rectangle(fol,fow,u[0],u[1],heading)
    return n


def initialwaypoints(n,p1,p2,p3,p4,x):
    x=float(x)
    x=radians(x)
    dis=float(get_distance(p1[0],p1[1],p2[0],p2[1])/n)  # equal space
    formation_waypoints=[]
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p1[0],p1[1],x,(dis/2))
            formation_waypoints.append(u)
        else:
            lat_lon=pointRadialDistance(formation_waypoints[i-1][0],formation_waypoints[i-1][1],x,(dis))
            formation_waypoints.append(lat_lon)
    return formation_waypoints

def get_search_wp(num_uavs:int, id:int, p1:list, p2:list):
    assert num_uavs >= id, f"Id ({id}) passed exceeded the active number of UAVs ({num_uavs})"
    # dist = (get_distance(p1[0],p1[1],p2[0],p2[1])/(num_uavs+1))*id  # distance away from point p1 (for i'th uav)
    
    del_x = (id + 1) * (p2[0] - p1[0]) / (num_uavs + 1)   # gps distance from p1 in x direction --
    del_y = (id + 1) * (p2[1] - p1[1]) / (num_uavs + 1)   # gps distance from p1 in y direction |
    return  [p1[0] + del_x, p1[1] + del_y]

def finalwaypoins(n,p1,p2,p3,p4,x):
    x=float(x)
    x=radians(x)
    b=[]
    dis=float(get_distance(p1[0],p1[1],p2[0],p2[1])/n)
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p4[0],p4[1],x,(dis/2))
            b.append(u)
        else:
            nmmm=pointRadialDistance(b[i-1][0],b[i-1][1],x,(dis))
            b.append(nmmm)
    return b

def inside_circle(vel, v_max, v_min):
        #print(vel)
        x=vel[0]
        y=vel[1]
        vc=[]
        if (x*x)+(y*y)-(v_max**2)<=0 and (x*x)+(y*y)-(v_min**2)>=0 :
            vc.append(x)
            vc.append(y)
            vc.append(0)
        elif (x*x)+(y*y)-(v_max**2)>=0:
            if x!=0:
                vc.append(v_max*cos(atan2(y,x)))
                vc.append(v_max*sin(atan2(y,x)))
                vc.append(0)
            elif y!=0:
                vc.append(v_max*sin(atan2(x,y)))
                vc.append(v_max*cos(atan2(x,y)))
                vc.append(0)
            else:
                vc.append(x)
                vc.append(y)
                vc.append(0)
        elif (x*x)+(y*y)-(v_min**2)<=0:
            if x!=0:
                vc.append(v_min*cos(atan2(y,x)))
                vc.append(v_min*sin(atan2(y,x)))
                vc.append(0)
            elif y!=0:
                vc.append(v_min*sin(atan2(x,y)))
                vc.append(v_min*cos(atan2(x,y)))
                vc.append(0)
            else:
                vc.append(x)
                vc.append(y)
                vc.append(0)
        return vc


def create_uav_ports():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return sock

def create_and_bind_uav_ports(address):

    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind(address)
        sock.settimeout(0.1)
    except:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(address)
        sock.settimeout(0.1)
    return sock

def send_data(sock, address, port, data):
    data_json = json.dumps(data).encode("UTF-8")
    sock.sendto(data_json, (address, port))

def recv_data(sock):
    try:
        data = sock.recv(1200)
        data = json.loads(data.decode('utf-8'))
        return data
    except:
        print("Exception occured in recv_data in helper functions file ")
        empty_data = {"SYSID":0,"GEOLOCATION":[],"HUMANS":[],"G":0.0,"P":0.0,"BESTLOC":(0,0),"GBESTLOC":(0,0),"PD":(0,0),"PAYLOAD":0,"DROPLOCATION":(0,0)}
        return empty_data


def T_max_calc_func(SAL, v):
    return SAL/v + 100

def fn(onelist,disitance_between_two_humans):
    #twice the error in human detection

    final_list = [] 
    a=[]
    b=[]
    for num in onelist: 
        for i in onelist:
            if i!=num:
                if get_distance(num[0],num[1],i[0],i[1]) <disitance_between_two_humans:
                    a.append([num,i]) 
    for i in a:
        j=[i[1],i[0]]
        if j in a:
            a.remove(j)
    for i in a:
        b.append(i[0])
        b.append(i[1])
    for i in a:
        if i[0][2]>=i[1][2]:
            final_list.append(i[0])
        else:
            final_list.append(i[1])
    for i in onelist:
        if i not in b:
            final_list.append(i)
    return final_list 

def filterhumans(loooopy,disitance_between_two_humans):
    while True:
        b=fn(loooopy,disitance_between_two_humans)
        if len(b)==len(loooopy):
            break
        loooopy=b
    temp_list=[]
    for i in b:
        if i not in temp_list:
            temp_list.append(i)

    return temp_list
def Extract(lst):
    # get the first column from a list of lists
    # what a useless function. just. bad. code 
    return [item[0] for item in lst]


def filter_remove_LAND_RTL_data(u):

    a =[]
    for key,datalist in u.items():
        if datalist["MODE"] == "LAND" or datalist["MODE"] == "RTL":
            a.append(key)

    for i in a:
         del u[i]       

    return u


def load_pickle(file_name):
    with open(file_name, 'rb') as f:
        file = pickle.load(f)
    return file
