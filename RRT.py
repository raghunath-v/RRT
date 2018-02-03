"""An implementation of the RRT algorithm in python"""
import random
import json
import time
from collections import defaultdict
from graphics import Circle, Line, GraphWin
from Obstacle import Obstacle
from Node import Node
from KinematicPoint import KinematicPoint
from BoundingArea import BoundingArea
from Goal import Goal
from g_tools import emit_verbose
class RRT:
    def __init__(self, bounding_area, obstacles, player, goal, rrt_setup, win, verbose=True):
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
        self.optimal_path = None
        # Other stuff
        self.verbose = verbose

    def generate(self):
        #q_init will be the starting location of the player
        self.start_node = Node(self.player.pos_x, self.player.pos_y)
        self.add_node(self.start_node)
        t = time.time()
        self.simple_rrt()
        emit_verbose("Generating graph for RRT took", self.verbose, var=time.time()-t)
        self.find_path()

    def simple_rrt(self):
        """Builds an RRT"""
        for _ in range(self.K):
            q_rand = self.gen_random_node()
            while not self.is_valid(q_rand):
                q_rand = self.gen_random_node()
            self.add_between_nearest(q_rand)
        self.goal_node = Node(self.goal.pos_x, self.goal.pos_y)
        self.add_to_nearest(self.goal_node)

    def rrt_star(self):
        for _ in range(self.K):
            q_rand = self.gen_random_node()
            while not self.is_valid(q_rand):
                q_rand = self.gen_random_node()
            q_near = self.find_nearest(q_rand)

        self.goal_node = Node(self.goal.pos_x, self.goal.pos_y)
        self.add_to_nearest(self.goal_node)

    def find_path(self):
        current_node = self.goal_node
        self.path = [current_node]
        while current_node != self.start_node:
            current_node = current_node.parent
            self.path.append(current_node)
        self.path.reverse()
        self.optimize_path()
    
    def optimize_path(self):
        # Iterate over all edges in path
        # and check if they can be removed
        # without any collision
        self.optimal_path = [self.path[0]]
        finished = False
        node_idx = 1
        has_found = False
        next_segment = None
        t = time.time()
        while not finished:
            if node_idx >= len(self.path):
                self.optimal_path.append(next_segment)
                finished = True
                break
            if self.is_segment_valid(self.optimal_path[-1], self.path[node_idx]):
                has_found = True
                next_segment = self.path[node_idx]
                next_idx = node_idx
                node_idx+=1
            else:
                if not has_found:
                    self.optimal_path.append(self.path[node_idx])
                    node_idx+=1
                else:
                    self.optimal_path.append(next_segment)
                    node_idx = next_idx + 1
                    has_found = False
        emit_verbose("Optimizing path took", self.verbose, var=time.time()-t)
    def get_path(self):
        return self.path

    def get_optimal_path(self):
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

    def find_nearest(self, target_node):
        """
            Finds a node in the graph that is nearest
            to a node (euclidian distance)
        """
        min_dist = float('inf')
        nearest = None
        for graph_node in self.graph:
            if target_node.dist_to(graph_node) < min_dist:
                nearest = graph_node
                min_dist = target_node.dist_to(nearest)
        return nearest

    def find_all_near(self, target_node):
        """
            Finds all nodes that are within a circle
            of certain radius from node
        """

    def select_best_parent(self, near_nodes, target_node):
        """
            From the nodes (list), select the node
            that, through itself, results in the least
            distance from the goal node to the target node
        """
    
    def rewire(self, near_nodes, target_node):
        """
            For all nodes that are near the target node,
            check if their cost to goal is reduced by choosing
            the target node as parent instead of their current
            parent. If it is, set their parent to the target.
        """

    
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
            if obs.contains(new.x, new.y):
                return False
        return self.bounding_area.contains(new.x, new.y)
    
    def is_segment_valid(self, n_1, n_2):
        for obs in self.obstacles:
            if obs.intersects_with_segment(n_1.x, n_1.y, n_2.x, n_2.y):
                return False
        return not self.bounding_area.intersects_with_segment(n_1.x, n_1.y, n_2.x, n_2.y)
    
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
        t = time.time()
        '''
        for node in self.graph:
            curr_loc = node.get_scaled_point()
            draw_node = Circle(curr_loc, 1)
            draw_node.setFill('red')
            #self.drawables.append(draw_node)
            for neighbor in self.graph[node]:
                if neighbor:
                    line = Line(curr_loc, neighbor.get_scaled_point())
                    line.draw(self.win)
                    self.drawables.append(line)
        '''
        for i in range(0,len(self.path)-1):
            node_1 = self.path[i]
            node_2 = self.path[i+1]
            cir = Circle(node_1.get_scaled_point(), 2)
            cir.setFill('Red')
            cir.setOutline('Red')
            self.drawable_path.append(cir)
            lin = Line(node_1.get_scaled_point(), node_2.get_scaled_point())
            lin.setOutline('Red')
            lin.draw(self.win)
            self.drawable_path.append(lin)
        
        for i in range(0,len(self.optimal_path)-1):
            node_1 = self.optimal_path[i]
            node_2 = self.optimal_path[i+1]
            cir = Circle(node_1.get_scaled_point(), 2)
            cir.setFill('Blue')
            cir.setOutline('Blue')
            self.drawable_path.append(cir)
            lin = Line(node_1.get_scaled_point(), node_2.get_scaled_point())
            lin.setOutline('Blue')
            lin.draw(self.win)
            self.drawable_path.append(lin)

        #for drawable in self.drawables:
        #    drawable.draw(self.win)
        #for drawable in self.drawable_path:
        #    drawable.draw(self.win)
        emit_verbose("Drawing RRT took", self.verbose, var=time.time()-t)

    def remove_graphicals(self, remove_path=False):
        for drawable in self.drawables:
            drawable.undraw()
        if remove_path:
            for drawable in self.drawable_path:
                drawable.undraw()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.graph))
