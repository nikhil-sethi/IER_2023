from geopy import distance, Point
from math import *
from .vector_math import Position


def bearing(lat1,lon1,lat2,lon2):
    lon1=radians(lon1)
    lon2=radians(lon2)
    lat1=radians(lat1)
    lat2=radians(lat2)
    dlon=abs(lon2-lon1)
    bearing=atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    return (degrees(bearing))


def rel_bearing(pos1, pos2):
    del_lat = pos1.lat-pos2.lat
    del_lon = pos1.lon-pos2.lon
    return degrees(atan2(del_lon,del_lat))

def abs_bearing(pos1, pos2):
    rel = rel_bearing(pos1, pos2)
    if rel <0:
        return 360-abs(rel)
    return rel

def get_distance(pos1:Position,pos2:Position): 
    """returns geodesic distance between two gps points in meters"""
    return distance.distance((pos1.lat,pos1.lon),(pos2.lat,pos2.lon)).m

def get_distance_3d(pos1:Position,pos2:Position): 
    """returns geodesic distance between two gps points in meters"""
    z_diff = pos1.alt- pos2.alt
    xy_diff = get_distance(pos1, pos2)
    return (xy_diff**2 + z_diff**2)**0.5

def pointRadialDistance(lat1,lon1,angle,d):
    """
    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    R=6371
    d=d/1000
    ad=d/R
    lat2 = asin(sin(lat1)*cos(ad) +cos(lat1)*sin(ad)*cos(angle))
    lon2 = lon1 + atan2(sin(angle)*sin(ad)*cos(lat1),cos(ad)-sin(lat1)*sin(lat2))
    lat = degrees(lat2)
    lon = degrees(lon2)
    return [lat,lon]

def compute_gps(px, py, fov_x, fov_y, pos, frame_shape):
    """Calculate true GPS position of object in an image frame"""
    height,width,_=frame_shape

    x_dist_per_pixle = 2 *pos.alt* tan(radians(fov_x / 2)) / width
    y_dist_per_pixle = 2 *pos.alt* tan(radians(fov_y / 2)) / height
    x = float((px - (width/ 2)))
    y = float((py - (height / 2)))

    theta = atan(x / y)
    theta = degrees(theta)
    newbear = pos.bear
    if (y < 0):
        newbear = pos.bear - theta

    if (y > 0):
        newbear = pos.bear + 180 - theta

    # bearing has to be between 0 and 360
    if (newbear > 360):
        newbear -= 360
    if (newbear < 0):
        newbear += 360
    #comment next line for onboard code
    #newbear=bear

    x *= x_dist_per_pixle
    y *= y_dist_per_pixle
    dist = sqrt(pow(x, 2) + pow(y, 2))
    dist /= 1000

    pt = Point(pos.lat,pos.lon)
    obj = distance.geodesic(kilometers=dist)
    target_li = list(obj.destination(point=pt, bearing=newbear))

    return target_li[0],target_li[1]