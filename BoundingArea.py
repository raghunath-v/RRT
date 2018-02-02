from graphics import *
import g_tools as g
import numpy as np

class BoundingArea:
    def __init__(self, points, win):
        self.points = points
        self.graphical_points = g.gen_point_list(g.scale_points(points))
        self.win = win
        self.segments = self.gen_segment()

    def gen_segment(self):
        # segments is like [[[x_1, y_1],[x_2,y_2]], ...]
        segments = []
        for i in range(len(self.points)):
            segments.append([self.points[i], self.points[(i+1) % len(self.points)]])
        return segments

    def contains(self, x, y):
        intersects = 0
        for s in self.segments:
            if g.ray_intersects_segment(x,y,s):
                intersects+=1
        return intersects % 2 != 0

    def set_graphicals(self):
        """Returns the graphical representation"""
        poly = Polygon(self.graphical_points)
        poly.setFill(color_rgb(254,254,254))
        poly.draw(self.win)