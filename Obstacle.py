from graphics import *
import numpy as np
import g_tools as g

class Obstacle:
    """
        An obstacle has both a graphical and a
        numerical representation
    """
    def __init__(self, points, win):
        # points is a list of lists where each inner list
        # is in a [x,y] configuration.
        self.points = points
        self.segments = self.gen_segment()
        self.graphical_points = g.gen_point_list(g.scale_points(points))
        self.win = win

    def set_graphicals(self):
        """Returns the graphical representation"""
        poly = Polygon(self.graphical_points)
        poly.setFill('green')
        poly.draw(self.win)

    def gen_segment(self):
        # segments is like [[[x_1, y_1],[x_2,y_2]], ...]
        segments = []
        for i in range(len(self.points)):
            segments.append([self.points[i], self.points[(i+1) % len(self.points)]])
        return segments


    def contains(self, x, y):
        intersects = 0
        for s in self.segments:
            x_1 = s[0][0]
            y_1 = s[0][1]
            x_2 = s[1][0]
            y_2 = s[1][1]
            # this is a simple ray casting method
            # tha is NOT always accurate
            if x < min(x_1, x_2):
                if min(y_1, y_2) < y and y < max(y_1, y_2):
                    intersects+=1
        return intersects % 2 != 0