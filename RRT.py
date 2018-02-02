"""An implementation of the RRT algorithm in python"""
import random
from collections import defaultdict
from graphics import *
import numpy as np
from Obstacle import Obstacle
from KinematicPoint import KinematicPoint
from BoundingArea import BoundingArea
from Goal import Goal
import json
import g_tools as g_tools

class RRT:
    def __init__(self, bounding_area, obstacles, player, goal, delta_q, K, win, edges=[], directed=False):
        # related to the program at a whole
        self.win = win
        self.bounding_area = bounding_area
        self.obstacles = obstacles
        self.player = player
        self.goal = goal
        # regarding the graph part
        self.graph = defaultdict(set)
        self.directed = directed
        self.add_edges(edges)
        # Relating specifically to RRT
        self.delta_q = delta_q 
        self.K = K # number of its
        self.start_node = None
        self.goal_node = None
        self.path = None

    def generate(self):
        """Builds an RRT"""
        #q_init will be the starting location of the player
        self.start_node = Node(self.player.pos_x, self.player.pos_y)
        self.add_node(self.start_node)
        for k in range(self.K):
            q_rand = self.gen_random_node()
            while not self.is_valid(q_rand):
                q_rand = self.gen_random_node()
            self.add_between_nearest(q_rand)
        self.goal_node = Node(self.goal.pos_x, self.goal.pos_y)
        self.add_to_nearest(self.goal_node)
        self.find_path()
        self.draw()

    def find_path(self):
        current_node = self.goal_node
        self.path = [current_node]
        while current_node != self.start_node:
            current_node = current_node.parent
            self.path.append(current_node)

    def add_edges(self, edges):
        """ Adds edges (list of tuple pairs) to the graph"""
        for node_1, node_2 in edges:
            self.add(node_1, node_2)

    def add(self, node_1, node_2):
        """"Add an edge between node_1 and node_2"""
        self.graph[node_1].add(node_2)
        if not self.directed:
            self.graph[node_2].add(node_1)

    # currently only used for adding the inital node
    def add_node(self, node):
        """Adds a single node without edges"""
        self.graph[node].add(None)

    # currently only used to connect the goal node
    def add_to_nearest(self, node):
        """Adds a node to the neighbor that is closest"""
        min_dist = float('inf')
        nearest = None
        for graph_node in self.graph:
            if node.dist_to(graph_node) < min_dist:
                nearest = graph_node
                min_dist = node.dist_to(nearest)
        node.set_parent(nearest)
        self.add(nearest, node)

    # currently used to add all other nodes in general
    def add_between_nearest(self, node):
        """
            Adds a node in q_delta distance from the
            node that is closest to the input node
        """
        min_dist = float('inf')
        closest = None
        # find the closest node
        for graph_node in self.graph:
            if node.dist_to(graph_node) < min_dist:
                closest = graph_node
                min_dist = node.dist_to(closest)
        # validate the to-be-added node
        new_node = closest.get_close(node, self.delta_q)
        if self.is_valid(new_node):
            new_node.set_parent(closest)
            self.add(closest, new_node)
    
    def is_valid(self, new):
        '''
            Checks if the edge between new and old intersects
            with any obstacle or bounding area and checks if 
            new node appears inside valid playing area
            TODO: Need to change this implementation for Dynamic
        '''
        # Check first of node is valid by doing ray casting to the right
        # on all obstacles and bounding area. Validate if intersects are odd
        for obs in self.obstacles:
            if obs.contains(new.get_x(), new.get_y()):
                return False
        if not self.bounding_area.contains(new.get_x(), new.get_y()):
            return False
        else:
            return True
    
    def remove(self, node):
        """ Remove all references to a node"""
        for ind, edges in self.graph.items():
            try:
                edges.remove(node)
            except KeyError:
                pass
        try:
            del self.graph[node]
        except KeyError:
            pass

    def gen_random_node(self):
        #
        if random.randint(0,5) == 0:
            return Node(self.goal.pos_x, self.goal.pos_y)
        else:
            return Node(random.uniform(-2,60), random.uniform(-2,60))

    def draw(self):
        """Draws the graph"""
        for node in self.graph:
            curr_loc = node.get_scaled_point()
            draw_node = Circle(curr_loc, 1)
            draw_node.setFill('red')
            draw_node.draw(self.win)
            for neighbor in self.graph[node]:
                if neighbor:
                    Line(curr_loc, neighbor.get_scaled_point()).draw(self.win)

        for i in range(1,len(self.path)-1):
            node_1 = self.path[i]
            node_2 = self.path[i+1]
            cir = Circle(node_1.get_scaled_point(), 2)
            cir.setFill('Red')
            cir.setOutline('Red')
            cir.draw(win)
            lin = Line(node_1.get_scaled_point(), node_2.get_scaled_point())
            lin.setOutline('Red')
            lin.draw(win)

        win.getMouse()
        win.close()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.graph))

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
        return Point(g_tools.scale(self._x), g_tools.scale(self._y))

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

if __name__ == "__main__":
    #run stuff here
    with open("P1.json") as json_file:
        desc = json.load(json_file)
    
    bounding_poly = desc['bounding_polygon']
    pos_start = desc['pos_start']
    pos_goal = desc['pos_goal']
    vel_start = desc['vel_start']
    vel_goal = desc['vel_goal']
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']
    win = GraphWin("area", 600, 600)
    win.yUp()
    p = KinematicPoint(vel_start, pos_start, dt, v_max, win)
    g = Goal(vel_goal, pos_goal, win)
    obs = [desc[key] for key, val in desc.items() if key.startswith('obs')]
    obstacles = [Obstacle(o,win) for o in obs]
    bounding_area = BoundingArea(bounding_poly, win)
    bounding_area.set_graphicals()
    for obs in obstacles:
        obs.set_graphicals()
    g.set_graphicals()
    p.set_graphicals()
    r = RRT(bounding_area, obstacles, p, g, 2, 500, win)
    r.generate()