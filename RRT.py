"""An implementation of the RRT algorithm in python"""
import random
import json
import time
import sys
from collections import defaultdict
from graphics import Circle, Line, GraphWin
from Obstacle import Obstacle
from Node import Node
from KinematicPoint import KinematicPoint
from BoundingArea import BoundingArea
from Goal import Goal
from g_tools import emit_verbose
class RRT:
    def __init__(self, bounding_area, obstacles, player, goal, rrt_setup, win, goal_rate=3, verbose=True):
        # related to the program at a whole
        self.win = win
        self.drawables = None
        self.drawable_path = None
        self.bounding_area = bounding_area
        self.obstacles = obstacles
        self.player = player
        self.goal = goal
        self.goal_rate = goal_rate
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
        self.strategy = rrt_setup['strategy']
        self.closest_to_goal = None
        self.closest_dist_to_goal = sys.float_info.max
        # Regarding RRT*
        self.star_dist = 2
        # Other stuff
        self.verbose = verbose

    def generate(self, data_keep=False):
        #q_init will be the starting location of the player
        t = time.time()
        if(self.strategy == 0):
            self.simple_rrt()
        elif(self.strategy == 1):
            self.rrt_star()
        emit_verbose("Generating graph for RRT took", self.verbose, var=time.time()-t)
        self.find_path()
        if data_keep:
            return time.time()-t 
    def simple_rrt(self):
        """Builds an RRT"""
        self.start_node = Node(self.player.pos_x, self.player.pos_y)
        self.add_node(self.start_node)
        self.closest_to_goal = self.start_node
        self.goal_node = Node(self.goal.pos_x, self.goal.pos_y)
        k=0
        while (not self.is_path_to_goal() and k < self.K):
            q_rand = self.gen_random_node()
            while not self.is_valid(q_rand):
                q_rand = self.gen_random_node()
            self.add_between_nearest(q_rand)
            k+=1
        if not k < self.K:
            self.set_graphicals(quick_draw=False)
            print('Could not reach goal, exiting')
            self.win.getMouse()
            self.win.close()
            sys.exit(0)
        self.add_to_nearest(self.goal_node)

    def rrt_star(self):
        self.start_node = Node(self.player.pos_x, self.player.pos_y)
        self.start_node.set_start_dist(0)
        self.closest_to_goal = self.start_node
        self.goal_node = Node(self.goal.pos_x, self.goal.pos_y)
        self.add_node(self.start_node)
        k = 0
        while not self.is_path_to_goal() and k < self.K:
            q_rand = self.gen_random_node()
            while not self.is_valid(q_rand):
                q_rand = self.gen_random_node()
                q_nearest = self.find_nearest(q_rand)
                q_new = q_nearest.get_close(q_rand, self.delta_q)
                k+=1
                if(self.is_valid(q_new)):
                    q_all_near = self.find_all_near(q_new)
                    if( len(q_all_near) != 0):
                        q_best_parent = self.select_best_parent(q_new, q_all_near)
                        if self.is_segment_valid(q_new, q_best_parent):
                            q_new.set_start_dist(q_best_parent.goal_dist + q_new.dist_to(q_best_parent))
                            q_new.set_parent(q_best_parent)
                            for q in q_all_near:
                                self.rewire(q, q_new)
                            self.add(q_new, q_best_parent)
        if not k < self.K:
            self.set_graphicals(quick_draw=False)
            print('Could not reach goal, exiting')
            self.win.getMouse()
            self.win.close()
            sys.exit(0)
        self.add_to_nearest(self.goal_node)

    def is_path_to_goal(self):
        return self.is_segment_valid(self.closest_to_goal, self.goal_node)

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
                if(next_segment is not None):
                    self.optimal_path.append(next_segment)
                    finished = True
                    break
                else:
                    print('Too few nodes, k too low or no valid way to goal')
                    print('Optimal path: ', self.optimal_path)
                    sys.exit(0)
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
        self.re_optimize_path()
    
    def re_optimize_path(self):
        # Iterate over all edges in path
        # and check if they can be removed
        # without any collision
        re_optimal_path = [self.optimal_path[0]]
        finished = False
        node_idx = 1
        has_found = False
        next_segment = None
        t = time.time()
        while not finished:
            if node_idx >= len(self.optimal_path):
                if(next_segment is not None):
                    re_optimal_path.append(next_segment)
                    finished = True
                    break
                else:
                    print('Too few nodes, k too low or no valid way to goal')
                    print('Optimal path: ', re_optimal_path)
                    sys.exit(0)
            if self.is_segment_valid(re_optimal_path[-1], self.optimal_path[node_idx]):
                has_found = True
                next_segment = self.optimal_path[node_idx]
                next_idx = node_idx
                node_idx+=1
            else:
                if not has_found:
                    re_optimal_path.append(self.optimal_path[node_idx])
                    node_idx+=1
                else:
                    re_optimal_path.append(next_segment)
                    node_idx = next_idx + 1
                    has_found = False
        emit_verbose("Optimizing path took", self.verbose, var=time.time()-t)
        self.optimal_path = re_optimal_path
    def get_path(self):
        return self.path

    def get_optimal_path(self):
        return self.path

    def add_edges(self, edges):
        """ Adds edges (list of tuple pairs) to the graph"""
        for node_1, node_2 in edges:
            self.add(node_1, node_2)

    def add(self, new_node, parent):
        """"Add an edge between node_1 and node_2"""
        if(new_node.dist_to(self.goal_node) < self.closest_dist_to_goal):
                self.closest_to_goal = new_node
                self.closest_dist_to_goal = new_node.dist_to(self.goal_node) 
        self.graph[new_node].add(parent)
        self.graph[parent].add(new_node)

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
                if self.is_segment_valid(graph_node, node):
                    nearest = graph_node
                    min_dist = node.dist_to(nearest)
        if nearest is None:
            return False
        else:
            node.set_parent(nearest)
            self.add(nearest, node)
            return True

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
        near_nodes = []
        for graph_node in self.graph:
            if target_node.dist_to(graph_node) < self.star_dist:
                near_nodes.append(graph_node)
        return near_nodes


    def select_best_parent(self, target_node, near_nodes):
        """
            From the nodes (list), select the node
            that, through itself, results in the least
            distance from the goal node to the target node
        """
        best_dist = float('inf')
        best_parent = None
        for node in near_nodes:
            if node.goal_dist + target_node.dist_to(node) < best_dist:
                best_parent = node
                best_dist = node.goal_dist + target_node.dist_to(node)
        return best_parent
    
    def rewire(self, near_node, target_node):
        """
            For a node that appears near the target node,
            check if its cost to goal is reduced by choosing
            the target node as parent instead of their current
            parent. If it is, set its parent to the target.
        """
        if target_node.goal_dist + near_node.dist_to(target_node) < near_node.goal_dist:
            # Do rewiring
            self.remove_as_child(near_node)
            near_node.set_start_dist(target_node.goal_dist + near_node.dist_to(target_node))
            near_node.set_parent(target_node)
            self.add(target_node, near_node)
    
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
            if(new_node.dist_to(self.goal_node) < self.closest_dist_to_goal):
                self.closest_to_goal = new_node
                self.closest_dist_to_goal = new_node.dist_to(self.goal_node) 
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

    def remove_as_child(self, node):
        """ Remove all references to a node"""
        for _, edges in self.graph.items():
            try:
                edges.remove(node)
            except KeyError:
                pass

    # A random node (with bias) is generated,
    # it is not neceserally a legal node
    def gen_random_node(self):
        if random.randint(0, self.goal_rate) == 0:
            return Node(self.goal.pos_x, self.goal.pos_y)
        else:
            return Node(random.uniform(self.min_x, self.max_x), random.uniform(self.min_x, self.max_x))

    def set_graphicals(self, quick_draw):
        """Draws the graph"""
        self.drawables = []
        self.drawable_path = []
        t = time.time()
        for node in self.graph:
            curr_loc = node.get_scaled_point()
            draw_node = Circle(curr_loc, 1)
            draw_node.setFill('red')
            draw_node.draw(self.win)
            self.drawables.append(draw_node)
            if not quick_draw:
                for neighbor in self.graph[node]:
                    if neighbor:
                        line = Line(curr_loc, neighbor.get_scaled_point())
                        line.draw(self.win)
                        self.drawables.append(line)
        if self.path is not None:
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
                cir = Circle(node_1.get_scaled_point(), 5)
                cir.setFill('Blue')
                cir.setOutline('Blue')
                cir.draw(self.win)
                self.drawable_path.append(cir)
                lin = Line(node_1.get_scaled_point(), node_2.get_scaled_point())
                lin.setOutline('Blue')
                lin.draw(self.win)
                self.drawable_path.append(lin)
        emit_verbose("Drawing RRT took", self.verbose, var=time.time()-t)

    def remove_graphicals(self, remove_path=False):
        for drawable in self.drawables:
            drawable.undraw()
        if remove_path:
            for drawable in self.drawable_path:
                drawable.undraw()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.graph))
