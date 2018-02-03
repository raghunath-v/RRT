from graphics import Point
import numpy as np
from g_tools import scale
class Node:
    """A simple node class"""
    def __init__(self, loc_x, loc_y):
        self.x = loc_x
        self.y = loc_y
        self.parent = None
        self.goal_dist = 0

    def set_goal_dist(self, dist):
        self.goal_dist = dist

    def set_parent(self, parent):
        self.parent = parent
    
    def get_point(self):
        return Point(self.x, self.y)

    def get_scaled_point(self):
        return Point(scale(self.x), scale(self.y))

    def get_close(self, node, delta_q):
        """
            Returns a new node that is on the line
            between this node and the input node in
            a delta_q distance from this node
        """
        # Dist is the distance between the two nodes
        dist = self.dist_to(node)
        new_x = self.x + (delta_q/dist)*(node.x - self.x)
        new_y = self.y + (delta_q/dist)*(node.y - self.y)
        return Node(new_x, new_y)

    def dist_to(self, node):
        """Returns the distance from this node to another"""
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)

    def slope_to(self, node):
        """Returns the slope from this node to another"""
        return (node.y - self.y) / (node.x - self.x)

    def __repr__(self):
        return "( " + str(self.x) + ", " + str(self.y) + " )"