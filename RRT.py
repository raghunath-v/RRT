"""An implementation of the RRT algorithm in python"""
import random
from collections import defaultdict
from graphics import *
import numpy as np

class Graph:
    """Simple graph class"""
    def __init__(self, edges=[], directed=False):
        self._graph = defaultdict(set)
        self._directed = directed
        self._bottom = 0
        self._right = 0
        self.add_edges(edges)

    def add_edges(self, edges):
        """ Adds edges (list of tuple pairs) to the graph"""
        for node_1, node_2 in edges:
            self.add(node_1, node_2)

    def add(self, node_1, node_2):
        """"Add an edge between node_1 and node_2"""
        self._graph[node_1].add(node_2)
        if not self._directed:
            self._graph[node_2].add(node_1)
        self.set_area([node_1, node_2])

    def set_area(self, nodes):
        """Sets the bounding box of graph"""
        for node in nodes:
            self._bottom = max(self._bottom, node.get_y())
            self._right = max(self._right, node.get_x())

    def add_node(self, node):
        """Adds a single node without edges"""
        self.set_area([node])
        self._graph[node].add(None)

    def add_to_nearest(self, node):
        """Adds a node to the neighbor that is closest"""
        self.set_area([node])
        min_dist = np.sqrt(self._bottom**2 + self._right**2)
        contender = None
        for neighbor in self._graph:
            if node.dist_to(neighbor) < min_dist:
                contender = neighbor
                min_dist = node.dist_to(contender)
        self.add(node, contender)

    def add_between_nearest(self, node, delta_q):
        """
            Adds a node in q_delta distance from the
            node that is closest to the input node
        """
        min_dist = np.sqrt(self._bottom**2 + self._right**2)
        contender = None
        for neighbor in self._graph:
            if node.dist_to(neighbor) < min_dist:
                contender = neighbor
                min_dist = node.dist_to(contender)
        self.add(contender.get_close(node, delta_q), contender)

    def remove(self, node):
        """ Remove all references to a node"""
        for ind, edges in self._graph.items():
            try:
                edges.remove(node)
            except KeyError:
                pass
        try:
            del self._graph[node]
        except KeyError:
            pass

    def find_path(self, node_1, node_2, path=[]):
        """Find any path between node_1 and node_2"""
        path = path + [node_1]
        if node_1 == node_2:
            return path
        if node_1 not in self._graph:
            return None
        for node in self._graph[node_1]:
            if node not in path:
                new_path = self.find_path(node, node_2, path)
                if new_path:
                    return new_path
        return None

    def draw(self):
        """Draws the graph"""
        win = GraphWin("Graph", self._right+10, self._bottom+10)
        for node in self._graph:
            curr_loc = Point(node.get_x(), node.get_y())
            Circle(curr_loc, 5).draw(win)
            for neighbor in self._graph[node]:
                if neighbor:
                    Line(curr_loc, Point(neighbor.get_x(), neighbor.get_y())).draw(win)
        win.getMouse()
        win.close()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self._graph))

class Node:
    """A simple node class"""
    def __init__(self, loc_x, loc_y):
        self._x = loc_x
        self._y = loc_y

    def get_x(self):
        """Returns the x coordinate of the node"""
        return self._x

    def get_y(self):
        """Returns the y coordinate of the node"""
        return self._y

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

def build_rrt(q_init, K, delta_q):
    """Builds an RRT"""
    g = Graph()
    g.add_node(q_init)

    for k in range(K):
        q_rand = Node(random.uniform(0, 1000), random.uniform(0, 1000))
        g.add_between_nearest(q_rand, delta_q)
    g.draw()

#1. Place an inital node at (500,500)
#2. Set K (num iterations) to 1000
#3. Set delta_q (see RRT alg. for more) to 20
build_rrt(Node(500, 500), 1000, 20)
