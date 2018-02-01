from graphics import *
import g_tools as g
import numpy as np

class BoundingArea:
    def __init__(self, points):
        self.points = points
        self.graphical_points = g.gen_point_list(g.scale_points(points))
    
    def get_graphical(self):
        """Returns the graphical representation"""
        poly = Polygon(self.graphical_points)
        poly.setFill(color_rgb(254,254,254))
        return poly