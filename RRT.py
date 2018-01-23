"""An implementation of the RRT algorithm in python"""
import random
from collections import defaultdict
from graphics import *
import numpy as np

class Graph:
    """Simple graph class"""
    def __init__(self, width, height, edges=[], directed=False):
        self._graph = defaultdict(set)
        self._directed = directed
        self._width = width
        self._height = height
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

    def add_node(self, node, init=False):
        """Adds a single node without edges"""
        self._graph[node].add(None)

    def add_to_nearest(self, node):
        """Adds a node to the neighbor that is closest"""
        #TODO: Does not consider obstacles
        min_dist = np.sqrt(self._height**2 + self._width**2)
        contender = None
        for neighbor in self._graph:
            if node.dist_to(neighbor) < min_dist:
                contender = neighbor
                min_dist = node.dist_to(contender)
        self.add(node, contender)

    def add_between_nearest(self, node, delta_q, obstacles=[]):
        """
            Adds a node in q_delta distance from the
            node that is closest to the input node
        """
        min_dist = np.sqrt(self._height**2 + self._width**2)
        contender = None
        for neighbor in self._graph:
            if node.dist_to(neighbor) < min_dist:
                contender = neighbor
                min_dist = node.dist_to(contender)
        new = contender.get_close(node, delta_q)
        is_ok = True
        for obs in obstacles:
            is_ok = is_ok and not obs.encapsulates(new)
        if is_ok:
            self.add(contender.get_close(node, delta_q), contender)
        else:
            new = Node(random.uniform(0, 1000), random.uniform(0, 1000))
            self.add_between_nearest(new, delta_q, obstacles)
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

    def draw(self, init, goal, path, obstacles=[]):
        """Draws the graph"""
        win = GraphWin("Graph", self._width, self._height)
        for obs in obstacles:
            g_obs = obs.get_graphical()
            g_obs.setOutline('Blue')
            g_obs.setFill('Blue')
            g_obs.draw(win)
        for node in self._graph:
            curr_loc = node.get_point()
            Circle(curr_loc, 5).draw(win)
            for neighbor in self._graph[node]:
                if neighbor:
                    Line(curr_loc, neighbor.get_point()).draw(win)

        # draw path
        for i in range(1,len(path)-1):
            node_1 = path[i]
            node_2 = path[i+1]
            cir = Circle(node_1.get_point(), 5)
            cir.setFill('Red')
            cir.setOutline('Red')
            cir.draw(win)
            lin = Line(node_1.get_point(), node_2.get_point())
            lin.setOutline('Red')
            lin.draw(win)

        # draw start and goal
        g_init = Circle(init.get_point(), 5)
        g_init.setOutline('Green')
        g_init.setFill('Green')
        g_init.draw(win)
        g_goal = Circle(goal.get_point(), 5)
        g_goal.setOutline('Green')
        g_goal.setFill('Green')
        g_goal.draw(win)
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
    
    def get_point(self):
        return Point(self._x, self._y)
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

class Obstacle:
    """
        An obstacle has both a graphical and a
        numerical representation
    """
    def __init__(self, x_left, x_right, y_top, y_bottom):
        # currently only supports rectangles
        # x_left < x_right
        self.x_left = x_left
        self.x_right = x_right
        # y_top < y_bottom
        self.y_top = y_top
        self.y_bottom = y_bottom
    
    def get_graphical(self):
        """Returns the graphical representation"""
        return Rectangle(Point(self.x_left, self.y_bottom),
            Point(self.x_right, self.y_top))
    
    def encapsulates(self, node):
        """checks if node is within rectangle"""
        return (self.x_left < node.get_x() and node.get_x() < self.x_right)\
            and (self.y_top < node.get_y() and node.get_y() < self.y_bottom)
    
    def __repr__(self):
        return "[ "+self.x_left+", "+self.y_bottom+" ], [ "+self.x_right+", "+self.y_top+" ]"

def build_rrt(q_init, K, delta_q, obs):
    """Builds an RRT"""
    g = Graph(WIDTH, HEIGHT)
    g.add_node(q_init, init=True)

    for k in range(K):
        q_rand = Node(random.uniform(0, 1000), random.uniform(0, 1000))
        g.add_between_nearest(q_rand, delta_q, obs)
    return g

WIDTH = 1000
HEIGHT = 1000
obs_1 = Obstacle(400,600, 400,600)
obs_2 = Obstacle(400, 600, 600,1000)
obs_3 = Obstacle(400,600, 0,400)
obs_4 = Obstacle(0,300, 200,300)
obs_5 = Obstacle(400,600, 800,1000)
obs_6 = Obstacle(700, 1000, 200, 300)
env_1 = [obs_1]
env_2 = [obs_1, obs_2]
env_3 = [obs_1, obs_3, obs_4, obs_5, obs_6]
env = env_3
intial_node = Node(100,500)
goal_node = Node(900,100)

g = build_rrt(intial_node, 1000, 20, env)
# add the goal node to the nearest neihbor
g.add_to_nearest(goal_node)
g.draw(intial_node, goal_node, g.find_path(intial_node, goal_node), env)
