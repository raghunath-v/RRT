from graphics import Point
import numpy as np
from g_tools import scale
class Node:
    """A simple node class"""
    def __init__(self, loc_x, loc_y):
        self._x = loc_x
        self._y = loc_y
        self.parent = None

    def set_parent(self, parent):
        self.parent = parent

    def get_x(self):
        """Returns the x coordinate of the node"""
        return self._x

    def get_y(self):
        """Returns the y coordinate of the node"""
        return self._y
    
    def get_point(self):
        return Point(self._x, self._y)

    def get_scaled_point(self):
        return Point(scale(self._x), scale(self._y))

    def get_close(self, node, delta_q):
        """
            Returns a new node that is on the line
            between this node and the input node in
            a delta_q distance from this node
        """
        # Dist is the distance between the two nodes
        dist = self.dist_to(node)
        new_x = self._x + (delta_q/dist)*(node.get_x() - self._x)
        new_y = self._y + (delta_q/dist)*(node.get_y() - self._y)
        return Node(new_x, new_y)

    def dist_to(self, node):
        """Returns the distance from this node to another"""
        return np.sqrt((self._x - node.get_x())**2 + (self._y - node.get_y())**2)

    def __repr__(self):
        return "( " + str(self._x) + ", " + str(self._y) + " )"