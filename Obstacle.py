from graphics import *
import numpy as np
import g_tools as g

class Obstacle:
    """
        An obstacle has both a graphical and a
        numerical representation
    """
    def __init__(self, points):
        # points is a list of lists where each inner list
        # is in a [x,y] configuration.
        self.points = points
        self.graphical_points = g.gen_point_list(g.scale_points(points))
    
    def get_graphical(self):
        """Returns the graphical representation"""
        poly = Polygon(self.graphical_points)
        poly.setFill('green')
        return poly