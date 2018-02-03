from graphics import Point
import numpy as np
from math import atan2, pi

class DubinCircle:
    """A class for Dubins Circles"""

    def __init__(self, centre, radius, direction):
        self.c = centre
        self.r = radius
        self.dir = direction

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

    def arclength(self, p1, p2):
        v1 = np.array([p1.x - self.c.x,
                       p1.y - self.c.y])
        v2 = np.array([p2.x - self.c.x,
                       p2.y - self.c.y])
        theta = atan2(v2[1], v2[0]) - atan2(v1[1], v1[0])

        if theta<0 and self.dir == -1:
            theta = theta + 2*pi
        elif theta>0 and self.dir == 1:
            theta = theta - 2*pi

        return abs(theta*self.r)