from math import *

class Vector2(list):
    @property
    def x(self):
        return self.__getitem__(0)

    @property
    def y(self):
        return self.__getitem__(1)

    def __sub__(self, other):
        return self.__class__([self[0] - other[0], self[1] - other[1]])
    def cap(self):
        return self/self.mag()        
    def mag(self):
        return (self[0]**2+self[1]**2)**0.5
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    def __add__(self, other):
        if isinstance(other, float):
            return self.__class__([self[0] + other, self[1] + other])     
        return self.__class__([self[0] + other[0], self[1] + other[1]])

    def angle(self, other):
        return acos(self.dot(other)/(self.mag() * other.mag()))

    def __mul__(self, scalar):
        return self.__class__([self[0]*scalar, self[1]*scalar])
    def dot(self, other):
        return self[0]*other[0] + self[1]*other[1]
    def __iadd__(self, other):
        return self.__class__([self[0] + other[0], self[1] + other[1]])
    def __truediv__(self, scalar):
        if scalar == 0:
            scalar = 1
        return self.__class__([self[0]/scalar, self[1]/scalar])
    def __itruediv__(self, scalar):
        if scalar == 0:
            scalar = 1
        return self.__class__([self[0]/scalar, self[1]/scalar])

    def rel_bearing(self):
        return degrees(atan2(self.y,self.x))

    def bearing(self):
        rel = self.rel_bearing()
        if rel < 0:
            return 360 - abs(rel)
        return rel
class Vector3(Vector2):
    @property
    def x(self):
        return self.__getitem__(0)
    @property
    def y(self):
        return self.__getitem__(1)
    @property
    def z(self):
        return self.__getitem__(2)

    def __add__(self, other):
        if isinstance(other, float):
            return self.__class__([self[0] + other, self[1] + other])     
        return self.__class__([self[0] + other[0], self[1] + other[1], self[2] + other[2]])

    def __sub__(self, other):
        return self.__class__([self[0] - other[0], self[1] - other[1], self[2] - other[2]])
    
    def __iadd__(self, other):
        return self.__class__([self[0] + other[0], self[1] + other[1], self[2] + other[2]])

    def __itruediv__(self, scalar):
        if scalar == 0:
            scalar = 1
        return self.__class__([self[0]/scalar, self[1]/scalar, self[2]/scalar])
    
    def __mul__(self, scalar):
        return self.__class__([self[0]*scalar, self[1]*scalar, self[2]*scalar])

    def mag(self):
        return (self[0]**2+self[1]**2+self[2]**2)**0.5

    def __truediv__(self, scalar):
        if scalar == 0:
            scalar = 1
        return self.__class__([self[0]/scalar, self[1]/scalar, self[2]/scalar])

class Position(Vector3):
    @property
    def lat(self):
        return self.__getitem__(0)
    @property
    def lon(self):
        return self.__getitem__(1)
    @property
    def alt(self):
        return self.__getitem__(2)

    def mag(self):
        # 2D because these are gps coordinates and alt is in different units.
        return (self[0]**2 + self[1]**2 + (self[2]/111139)**2)**0.5

class Position4D(Position):
    @property
    def bear(self):
        return self.__getitem__(3)

    def __add__(self, other):
        return Position4D([self[0] + other[0], self[1] + other[1], self[2] + other[2], self[3] + other[3]])

    def __truediv__(self, scalar):
        if scalar == 0:
            scalar = 1
        return Position4D([self[0]/scalar, self[1]/scalar, self[2]/scalar, self[3]/scalar])

class Velocity(Vector3):
    pass


def mag(vel):
    """2D vector magnitude"""
    return (vel[0]**2 + vel[1]**2 + vel[2]**2)**0.5

def clamp_vel(v, v_min, v_max):
    """ Limits velocity magnitude between v_min and v_max
    replaces inside_circle"""
    v_mag = mag(v)
    if v_mag == 0:
        return
    c = 1
    if v_mag > v_max:
        c = v_max/v_mag
    elif v_mag < v_min:
        c = v_min/v_mag
    v[0] *= c
    v[1] *= c

def Limit_vel(v, v_max):
    return (v / abs(v)) * min(abs(v), abs(v_max))

def point_on_line_seg(vec, point, v_i):
  p_a = v_i 
  p_b = v_i + vec
  p_c = point

  d_ab = (p_b-p_a).mag()
  d_ac = (p_c-p_a).mag()
  d_bc = (p_c-p_b).mag()

  return (d_ac + d_bc)-d_ab <0.000001 

def intersec_vec(vec1, p1, vec2, p2):
  """Finds the intersection of a vector v1 passing through point p1
  and another vector v2 passing through point p2"""
  m1 = vec1[1]/vec1[0]  # slope
  m2 = vec2[1]/vec2[0]
  a1 = -m1
  a2 = -m2
  c1 = m1*p1[0]-p1[1]
  c2 = m2*p2[0]-p2[1]

  xi = (c2-c1)/(a1-a2)
  yi = (a2*c1-a1*c2)/(a1-a2)
  return Vector2([xi, yi])
