"""An implementation of the RRT algorithm in python"""
import random
import json
from collections import defaultdict
from graphics import Circle, Line, GraphWin
from Obstacle import Obstacle
from Node import Node
from KinematicPoint import KinematicPoint
from BoundingArea import BoundingArea
from Goal import Goal

class RRT:
    def __init__(self, bounding_area, obstacles, player, goal, rrt_setup, win):
        # related to the program at a whole
        self.win = win
        self.drawables = None
        self.drawable_path = None
        self.bounding_area = bounding_area
        self.obstacles = obstacles
        self.player = player
        self.goal = goal
        # regarding the graph part
        self.graph = defaultdict(set)
        # Relating specifically to RRT
        self.delta_q = rrt_setup['delta_q']
        self.min_x, self.max_x = rrt_setup['x_range'][0], rrt_setup['x_range'][1]
        self.min_y, self.max_y = rrt_setup['y_range'][1], rrt_setup['y_range'][1]
        self.K = rrt_setup['k']
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

    def find_path(self):
        current_node = self.goal_node
        self.path = [current_node]
        while current_node != self.start_node:
            current_node = current_node.parent
            self.path.append(current_node)
    
    def get_path(self):
        return self.path

    def add_edges(self, edges):
        """ Adds edges (list of tuple pairs) to the graph"""
        for node_1, node_2 in edges:
            self.add(node_1, node_2)

    def add(self, node_1, node_2):
        """"Add an edge between node_1 and node_2"""
        self.graph[node_1].add(node_2)
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
        for obs in self.obstacles:
            if obs.contains(new.get_x(), new.get_y()):
                return False
        if not self.bounding_area.contains(new.get_x(), new.get_y()):
            return False
        else:
            return True
    
    def remove(self, node):
        """ Remove all references to a node"""
        for _, edges in self.graph.items():
            try:
                edges.remove(node)
            except KeyError:
                pass
        try:
            del self.graph[node]
        except KeyError:
            pass

    def gen_random_node(self):
        if random.randint(0, 5) == 0:
            return Node(self.goal.pos_x, self.goal.pos_y)
        else:
            return Node(random.uniform(self.min_x, self.max_x), random.uniform(self.min_x, self.max_x))

    def set_graphicals(self):
        """Draws the graph"""
        self.drawables = []
        self.drawable_path = []
        for node in self.graph:
            curr_loc = node.get_scaled_point()
            draw_node = Circle(curr_loc, 1)
            draw_node.setFill('red')
            self.drawables.append(draw_node)
            for neighbor in self.graph[node]:
                if neighbor:
                    line = Line(curr_loc, neighbor.get_scaled_point())
                    self.drawables.append(line)
        for i in range(0,len(self.path)-1):
            node_1 = self.path[i]
            node_2 = self.path[i+1]
            cir = Circle(node_1.get_scaled_point(), 2)
            cir.setFill('Red')
            cir.setOutline('Red')
            self.drawable_path.append(cir)
            lin = Line(node_1.get_scaled_point(), node_2.get_scaled_point())
            lin.setOutline('Red')
            self.drawable_path.append(lin)

        for drawable in self.drawables:
            drawable.draw(self.win)
        for drawable in self.drawable_path:
            drawable.draw(self.win)

    def remove_graphicals(self, remove_path=False):
        for drawable in self.drawables:
            drawable.undraw()
        if remove_path:
            for drawable in self.drawable_path:
                drawable.undraw()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.graph))


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
    #g.set_graphicals()
    #p.set_graphicals()
    r = RRT(bounding_area, obstacles, p, g, 2, 500, [-2,60], [-2,60], win)
    r.generate()