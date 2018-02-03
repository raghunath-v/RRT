import random
from collections import defaultdict
from graphics import *
import numpy as np
import math as math

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
        new_x = self._x + (delta_q / dist) * (node.get_x() - self._x)
        new_y = self._y + (delta_q / dist) * (node.get_y() - self._y)
        return Node(new_x, new_y)

    def dist_to(self, node):
        """Returns the distance from this node to another"""
        return np.sqrt((self._x - node.get_x()) ** 2 + (self._y - node.get_y()) ** 2)

    def slope_to(self, node):
        """Returns the slope from this node to another"""
        return (node.get_y() - self._y) / (node.get_x() - self._x)

    def __repr__(self):
        return "( " + str(self._x) + ", " + str(self._y) + " )"


def get_slope(p1, p2):
    slope = (p2[1]-p1[1])/(p2[0]-p1[0]);
    return slope

def get_dist(p1, p2):
    distance = math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[1])**2)
    return distance

def get_radius(v,acc):
    return math.sqrt(v[0]**2+v[1]**2)/acc

def get_tangents(Circle1, Circle2):
    C1 = Circle1.get_centre()
    R1 = Circle1.get_radius()
    dir1 = Circle1.get_dir()
    C2 = Circle2.get_centre()
    R2 = Circle2.get_radius()
    dir2 = Circle2.get_dir()

    check_swap = True
    swapped = False

    tangents = []

    if check_swap and (C1.get_x() > C2.get_x()):  #To get
        temp = R2
        R2 = R1
        R1 = temp
        temp = C2
        C2 = C1
        C1 = temp
        swapped = True
        print("Swapping")

    #Tangent 1

    theta = math.atan(C1.slope_to(C2))
    d = C1.dist_to(C2)
    alpha = math.acos((R1-R2)/d)
    beta = math.acos((R1+R2)/d)
    #Outer tangents only
    T1_1 = Node(math.ceil(C1.get_x() + R1 * (math.cos(alpha + theta))),
                math.ceil(C1.get_y() + R1 * (math.sin(alpha + theta))))
    T1_2 = Node(math.ceil(C2.get_x() + R2 * (math.cos(math.pi - alpha + theta))),
                math.ceil(C2.get_y() + R2 * (math.sin(math.pi - alpha + theta))))
    T2_1 = Node(math.ceil(C1.get_x() + R1 * (math.cos(-alpha + theta))),
                math.ceil(C1.get_y() + R1 * (math.sin(-alpha + theta))))
    T2_2 = Node(math.ceil(C2.get_x() + R2 * (math.cos(-(math.pi - alpha) + theta))),
                math.ceil(C2.get_y() + R2 * (math.sin(-(math.pi - alpha) + theta))))
    if check_swap and swapped:
        if (dir1 == 'L' and dir2 == 'L'):
            tangents.append((T1_1, T1_2))
        if (dir1 == 'R' and dir2 == 'R'):
            tangents.append((T2_1, T2_2))
    else:
        if (dir1 == 'L' and dir2 == 'L'):
            tangents.append((T1_2, T1_1))
        if (dir1 == 'R' and dir2 == 'R'):
            tangents.append((T2_2, T2_1))

    #Inner tangents
    T1_1 = Node(math.ceil(C1.get_x() + R1 * (math.cos(beta + theta))),
                math.ceil(C1.get_y() + R1 * (math.sin(beta + theta))))
    T1_2 = Node(math.ceil(C2.get_x() + R2 * (math.cos(-(math.pi - beta) + theta))),
                math.ceil(C2.get_y() + R2 * (math.sin(-(math.pi - beta) + theta))))
    T2_1 = Node(math.ceil(C1.get_x() + R1 * (math.cos(-beta + theta))),
                math.ceil(C1.get_y() + R1 * (math.sin(-beta + theta))))
    T2_2 = Node(math.ceil(C2.get_x() + R2 * (math.cos(math.pi - beta + theta))),
                math.ceil(C2.get_y() + R2 * (math.sin(math.pi - beta + theta))))
    if check_swap and swapped:
        if (dir1 == 'L' and dir2 == 'R'):
            tangents.append((T1_1, T1_2))
        if (dir1 == 'R' and dir2 == 'L'):
            tangents.append((T2_1, T2_2))
    else:
        if (dir1 == 'L' and dir2 == 'R'):
            tangents.append((T1_2, T1_1))
        if (dir1 == 'R' and dir2 == 'L'):
            tangents.append((T2_2, T2_1))

    return tangents


def get_dubin_paths(P1, V1, P2, V2):
    paths=['RSR', 'LSL', 'RSL', 'LSR']

    return paths

class DubinCircle:
    """A class for Dubins Circles"""

    def __init__(self, centre, radius, direction):
        self._c = centre
        self._r = radius
        self._dir = direction

    def get_centre(self):
        """Returns the x coordinate of the node"""
        return self._c

    def get_radius(self):
        """Returns the y coordinate of the node"""
        return self._r

    def get_dir(self):
        """Returns the y coordinate of the node"""
        return self._dir

    def get_point(self):
        return Point(self.get_centre().get_x(), self.get_centre().get_y())



if __name__=='__main__':
    goal = Node(400, 300)
    init = Node(400, 30)
    v_in = np.array([30, 30])
    v_fin = np.array([20, -20])
    a_max = 0.5

    orient = math.atan(v_in[1]/v_in[0])

    WIDTH = 600
    HEIGHT = 600

    g = Graph(WIDTH, HEIGHT)

    direction = ''
    radius_init = get_radius(v_in, a_max)
    radius_goal = get_radius(v_fin, a_max)
    #print(radius_init)

    init_slope = -v_in[0]/v_in[1]
    print(init_slope)

    k_init = radius_init / math.sqrt(1 + init_slope**2)
    # Left circle
    C1_cen = Node(init.get_x() + k_init, init.get_y() + k_init * init_slope)
    if v_in[1]>0:
        direction = 'L'
    else:
        direction = 'R'
    C1_init = DubinCircle(C1_cen, radius_init, direction)

    # Right circle
    C2_cen = Node(init.get_x() - k_init, init.get_y() - k_init * init_slope)
    if v_in[1]>0:
        direction = 'L'
    else:
        direction = 'R'
    C2_init = DubinCircle(C2_cen, radius_init, direction)


    goal_slope = -v_fin[0] / v_fin[1]
    k_goal = radius_goal / math.sqrt(1 + goal_slope**2)
    #Left circle
    C1_cen = Node(goal.get_x() + k_goal, goal.get_y() + k_goal * goal_slope)
    if v_fin[1]>0:
        direction = 'L'
    else:
        direction = 'R'
    C1_goal = DubinCircle(C1_cen, radius_goal, direction)

    #Right circle
    C2_cen = Node(goal.get_x() - k_goal, goal.get_y() - k_goal * goal_slope)
    if goal_slope>0:
        direction = 'R'
    else:
        direction = 'L'
    C2_goal = DubinCircle(C2_cen, radius_goal, direction)

    print(C1_init.get_dir(), C1_goal.get_dir())
    tangents = get_tangents(C1_init, C1_goal)


    win = GraphWin("Graph", WIDTH, HEIGHT)
    g_init1 = Circle(init.get_point(), 5)
    g_init1.setOutline('Green')
    g_init1.setFill('Green')
    g_init1.draw(win)

    g_init1 = Circle(C1_init.get_point(), radius_init)
    g_init1.setOutline('Black')
    #g_init1.setFill('Black')
    g_init1.draw(win)

    g_init1 = Circle(C2_init.get_point(), radius_init)
    g_init1.setOutline('Black')
    #g_init1.setFill('Black')
    g_init1.draw(win)

    g_goal = Circle(goal.get_point(), 5)
    g_goal.setOutline('Red')
    g_goal.setFill('Red')
    g_goal.draw(win)

    g_goal = Circle(C1_goal.get_point(), radius_goal)
    g_goal.setOutline('Black')
    #g_goal.setFill('Red')
    g_goal.draw(win)

    g_goal = Circle(C2_goal.get_point(), radius_goal)
    g_goal.setOutline('Black')
    #g_goal.setFill('Red')
    g_goal.draw(win)


    #draw tangents
    for tan in tangents:
        g_goal = Circle(tan[0].get_point(), 5)
        g_goal.setOutline('Blue')
        g_goal.setFill('Blue')
        g_goal.draw(win)

        g_goal = Circle(tan[1].get_point(), 5)
        g_goal.setOutline('Blue')
        g_goal.setFill('Blue')
        g_goal.draw(win)

        Line(tan[0].get_point(),tan[1].get_point()).draw(win)
    new = Point(init.get_x() + v_in[0], init.get_y() + v_in[1])
    Line(init.get_point(), new).draw(win)
    new = Point(goal.get_x() + v_fin[0], goal.get_y() + v_fin[1])
    Line(goal.get_point(), new).draw(win)

    #print(T1.get_point())
    #print(T2.get_point())

    #Circle(T1.get_point(), 5).draw(win)
    #Circle(T2.get_point(), 5).draw(win)
    win.getMouse()
    win.close()

    #g.draw(init, goal, [])

    #print(T1,T2)
