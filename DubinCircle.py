from graphics import Point
import numpy as np
import math
from g_tools import scale_coordinate, scale_points, scale_vectors, get_sign
from Node import Node

class DubinCircle:
    """A class for Dubins Circles"""


    """
    def __init__(self, arcpoint1, arcpoint2, steer, type = 0):
        self.r = abs(1 / steer)
        self.dir = steer
        side = math.sqrt((arcpoint1.x - arcpoint2.x)**2 + (arcpoint1.y - arcpoint2.y)**2)
        angle = math.acos(side/(2*self.r))
        sign = np.sign(steer)
        rotation_matrix = np.array([math.cos(angle), sign * math.sin(angle)],
                                  [sign * math.sin(angle), math.cos(angle)])
        centre = rotation_matrix*(np.array([arcpoint2.x, arcpoint2.y]) -
                                  np.array([arcpoint1.x, arcpoint1.y]))
        centre = np.array([arcpoint1.x, arcpoint1.y]) + \
                 (self.r*centre)/np.sum(np.dot(centre, centre))
        self.c = Node(centre[0], centre[1])
        print(self.c.x, self.c.y)
    """


    def __init__(self, centre, radius, direction):
        self.c = centre
        self.r = radius
        self.dir = direction
        if radius != abs(1/direction):
            print("Danger")

    @classmethod
    def fromArc(cls, arcpoint1, arcpoint2, steer):
        r = abs(1 / steer)

        sign = get_sign(steer)
        side = math.sqrt((arcpoint1.x - arcpoint2.x) ** 2 + (arcpoint1.y - arcpoint2.y) ** 2)
        new_point = Node((arcpoint1.x + arcpoint2.x)/2, (arcpoint1.y + arcpoint2.y)/2)
        bisect = math.sqrt(r**2 - (side/2)**2)
        dx = arcpoint1.x - arcpoint2.x
        dy = arcpoint1.y - arcpoint2.y
        slope_sign = get_sign(dy/dx)
        dist = math.sqrt(dx * dx + dy * dy)
        dx /= dist
        dy /= dist
        x3 = new_point.x + slope_sign*bisect*dy
        y3 = new_point.y - slope_sign*bisect*dx
        c = Node(x3, y3)

        return cls(c, r, steer)

    @classmethod
    def fromVel(cls, arcpoint1, steer, vel):
        r = abs(1 / steer)
        slope_sign = get_sign(-math.tan(vel[0]/vel[1]))
        dir = -get_sign(steer)
        dx = vel[0]
        dy = vel[1]
        mag = math.sqrt(vel[0] * vel[0] + vel[1] * vel[1])
        dx /= mag
        dy /= mag
        rotation_mat = np.array([[0, dir],
                                 [-dir, 0]])
        to_centre = (r*np.array([dx, dy])) @ rotation_mat
        #x3 = arcpoint1.x + slope_sign * dir * r * dy
        #y3 = arcpoint1.y - slope_sign * dir * r * dx
        x3 = to_centre[0] + arcpoint1.x
        y3 = to_centre[1] + arcpoint1.y
        c = Node(x3, y3)

        return cls(c, r, steer)

    def get_centre(self):
        """Returns the x coordinate of the node"""
        return self.c

    def get_radius(self):
        """Returns the y coordinate of the node"""
        return self.r

    def get_dir(self):
        """Returns the y coordinate of the node"""
        return self.dir

    def get_point(self):
        return Point(self.get_centre().x, self.get_centre().y)

    def get_scaled_centre(self):
        return Point(scale_coordinate(self.c.x), scale_coordinate(self.c.y))

    def arclength(self, p1, p2):
        v1 = np.array([p1.x - self.c.x,
                       p1.y - self.c.y])
        v2 = np.array([p2.x - self.c.x,
                       p2.y - self.c.y])

        theta = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])

        if theta<0 and self.dir < 0:
            theta = theta + 2*math.pi
        elif theta>0 and self.dir > 0:
            theta = theta - 2*math.pi
        return abs(theta*self.r)

    def arcangle(self, p1, p2):
        v1 = np.array([p1.x - self.c.x,
                       p1.y - self.c.y])
        v2 = np.array([p2.x - self.c.x,
                       p2.y - self.c.y])
        theta = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        if theta<0 and self.dir < 0:
            theta = theta + 2*math.pi
        elif theta>0 and self.dir > 0:
            theta = theta - 2*math.pi

        return theta